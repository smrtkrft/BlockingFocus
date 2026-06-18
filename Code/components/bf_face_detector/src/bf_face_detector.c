#include "bf_face_detector.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bf_lis3dsh.h"
#include "sk_capabilities.h"
#include "sk_cli.h"
#include "sk_errors.h"
#include "sk_event_bus.h"

static const char *TAG = "bf_face";

// Adaptive polling: in LOW_POWER (no countdown for 10 min) the device
// is mostly waiting for the user to come back, so dropping from 10 Hz
// to 1 Hz cuts I2C traffic 10× AND lets FreeRTOS tickless idle keep the
// CPU in light-sleep for ~990 ms out of every 1000 ms. Worst-case wake
// latency when the user picks up the cube: 1 sec for the next poll +
// debounce, after which bf_power's face.changed handler kicks us back
// to ACTIVE_IDLE and polling jumps back to 10 Hz.
//
// SCREEN_OFF (5 min idle) sits in the middle: 4 Hz keeps the response
// snappy when the user reaches for the cube without burning a full
// 10 Hz I2C cycle.
#define POLL_PERIOD_ACTIVE_MS    100      // 10 Hz — countdown running or recently active
#define POLL_PERIOD_SCREEN_MS    250      // 4  Hz — display off but device "warm"
#define POLL_PERIOD_LOW_MS       1000     // 1  Hz — deep idle, tickless light-sleep dominant
#define DEBOUNCE_MS              1000     // 1 s of stable orientation before commit.
                                          // Time-based (not sample-count) so the
                                          // debounce window stays the same
                                          // regardless of current poll period.

// Classification uses RELATIVE axis dominance rather than an absolute
// threshold. Real-world IMUs (especially the MPU-6050 prototype IMU)
// have asymmetric per-axis bias — e.g. our chip reads +1.4 g face-up
// but only -0.65 g face-down on z (a ~+0.35 g z-axis offset). A plain
// "|reading| ≥ 0.7 g" gate would lose the face-down case while still
// seeing face-up. Ratio-based dominance handles asymmetric bias for
// free, and works the same on the production LIS3DSH.
//
//   * MIN_DOMINANT_G   = 0.40 — guarantees the cube is resting on a
//                                face (not in free-fall / mid-shake).
//                                User-tuned (was 0.45) to accept more
//                                "mostly upright but slightly tilted"
//                                hand-holds.
//   * DOMINANCE_RATIO  = 1.2  — winning axis must lead second-place by
//                                at least 1.2×. User-tuned (was 1.4) so
//                                the cube doesn't have to sit perfectly
//                                flat. A true corner (three axes ≈ 0.577)
//                                still has ratio 1.0 and is rejected.
#define MIN_DOMINANT_G      0.40f
#define DOMINANCE_RATIO     1.2f

// TILTED overlay: when the cube has been resting on a corner / edge for
// a sustained period (no axis dominant), we publish `face.tilted` so the
// UI can show a "place the cube flat" hint. The committed face is left
// untouched — bf_timer_engine never sees the tilt, so an active
// countdown keeps running. 5 s rejects natural hand jitter while still
// catching genuine "user picked it up" cases quickly. Definition lives
// below the state vars (with the rest of the time-based thresholds).

static int s_committed_face = BF_FACE_NONE;
static int s_pending_face   = BF_FACE_NONE;
static int64_t s_pending_started_us = 0;  // 0 = no pending; otherwise wall-clock
                                          // timestamp when s_pending_face was
                                          // first observed. Time-based commit
                                          // means we no longer care how many
                                          // samples were taken in between.
static int64_t s_tilt_started_us = 0;     // same idea for tilted overlay
static bool s_tilted        = false;      // currently in tilted overlay

// Tilt overlay activates when the cube has been ambiguous for this long.
#define TILT_TRIGGER_MS     5000

// Current poll period — written by the power.state subscriber, read by
// poll_task. Single writer / single reader, word-aligned: a plain
// volatile read is sufficient on RISC-V (no atomic ops needed).
static volatile int s_poll_period_ms = POLL_PERIOD_ACTIVE_MS;

// ---------------------------------------------------------------------
// Axis-to-face mapping
// ---------------------------------------------------------------------

// Returns 0 if no axis is dominant enough, otherwise the matching face.
static int classify(float x, float y, float z)
{
    float ax = fabsf(x), ay = fabsf(y), az = fabsf(z);

    int   axis;       // 0 = x, 1 = y, 2 = z
    float largest, second, sign;

    if (az >= ax && az >= ay) {
        axis = 2; largest = az; sign = z;
        second = (ax > ay) ? ax : ay;
    } else if (ax >= ay) {
        axis = 0; largest = ax; sign = x;
        second = (ay > az) ? ay : az;
    } else {
        axis = 1; largest = ay; sign = y;
        second = (ax > az) ? ax : az;
    }

    if (largest < MIN_DOMINANT_G)               return BF_FACE_NONE;
    if (largest < DOMINANCE_RATIO * second)     return BF_FACE_NONE;

    switch (axis) {
    case 0: return (sign > 0) ? BF_FACE_X_UP : BF_FACE_X_DOWN;
    case 1: return (sign > 0) ? BF_FACE_Y_UP : BF_FACE_Y_DOWN;
    case 2: return (sign > 0) ? BF_FACE_Z_UP : BF_FACE_Z_DOWN;
    }
    return BF_FACE_NONE;
}

static const char *face_label(int face)
{
    switch (face) {
    case BF_FACE_Z_UP:   return "Z+";
    case BF_FACE_Z_DOWN: return "Z-";
    case BF_FACE_X_UP:   return "X+";
    case BF_FACE_X_DOWN: return "X-";
    case BF_FACE_Y_UP:   return "Y+";
    case BF_FACE_Y_DOWN: return "Y-";
    default:             return "none";
    }
}

// ---------------------------------------------------------------------
// Debounce + event publishing
// ---------------------------------------------------------------------

static void commit(int face)
{
    int prev = s_committed_face;
    s_committed_face = face;
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"face\":%d,\"previous\":%d}", face, prev);
    sk_event_bus_publish("face.changed", buf);
    ESP_LOGI(TAG, "face → %d (%s), was %d", face, face_label(face), prev);
}

static void process_sample(int candidate)
{
    int64_t now = esp_timer_get_time();

    // Tilt tracking runs on every sample, regardless of debounce state.
    // A flat orientation (any committed face candidate) clears the tilt
    // timer; sustained ambiguity raises the overlay after TILT_TRIGGER_MS.
    if (candidate == BF_FACE_NONE) {
        if (s_tilt_started_us == 0) s_tilt_started_us = now;
        if (!s_tilted &&
            (now - s_tilt_started_us) >= (int64_t)TILT_TRIGGER_MS * 1000) {
            s_tilted = true;
            sk_event_bus_publish("face.tilted", "{\"tilted\":true}");
            ESP_LOGW(TAG, "tilted overlay ON (no dominant axis %d ms)",
                     TILT_TRIGGER_MS);
        }
    } else {
        if (s_tilted) {
            s_tilted = false;
            sk_event_bus_publish("face.tilted", "{\"tilted\":false}");
            ESP_LOGI(TAG, "tilted overlay OFF (axis returned: %s)",
                     face_label(candidate));
        }
        s_tilt_started_us = 0;
    }

    if (candidate == BF_FACE_NONE) {
        // Ambiguous orientation — reset pending so the cube has to settle
        // again before any new commit. Doesn't affect committed face.
        s_pending_face       = BF_FACE_NONE;
        s_pending_started_us = 0;
        return;
    }
    if (candidate == s_committed_face) {
        // Already on this face — no work, also clear pending so we don't
        // accidentally commit a stale candidate later.
        s_pending_face       = BF_FACE_NONE;
        s_pending_started_us = 0;
        return;
    }
    if (candidate == s_pending_face) {
        // Time-based commit: regardless of how many samples we took (this
        // matters because in LOW_POWER we sample at 1 Hz, so the old
        // sample-count debounce would commit on the very first match
        // after only 1 second of stability — but at 1 Hz "1 sample"
        // happens to be a full second of physical stability, which is
        // exactly the policy we want anyway).
        if ((now - s_pending_started_us) >= (int64_t)DEBOUNCE_MS * 1000) {
            commit(candidate);
            s_pending_face       = BF_FACE_NONE;
            s_pending_started_us = 0;
        }
    } else {
        s_pending_face       = candidate;
        s_pending_started_us = now;
    }
}

// ---------------------------------------------------------------------
// Polling task
// ---------------------------------------------------------------------

static void poll_task(void *arg)
{
    (void)arg;
    for (;;) {
        float x, y, z;
        if (bf_lis3dsh_read_g(&x, &y, &z) == ESP_OK) {
            // Chip mount kompenzasyonu: yeni PCB rev'inde LIS3DSH
            // kompleks 90° rotasyonla mount edilmiş. Empirik veriden:
            //   5dk üstte (cube_x+)  → chip_z ≈ -1
            //   15dk üstte (cube_x-) → chip_z ≈ +1
            //   ekran üstte (cube_z+) → chip_y ≈ +1
            // Chip ekseni → küp ekseni eşlemesi:
            //   chip_x+ = cube_y-   (sağ el kuralı)
            //   chip_y+ = cube_z+
            //   chip_z+ = cube_x-
            // Sınıflandırıcının beklediği küp eksenlerine remap:
            //   x_cube' = -z, y_cube' = -x, z_cube' = y
            float rx = -z;
            float ry = -x;
            float rz = y;
            process_sample(classify(rx, ry, rz));
        }
        // If the read fails (sensor absent, I2C error), we just skip this
        // tick. Committed face stays 0; downstream sees "no face".
        //
        // Re-read s_poll_period_ms each loop so a power.state transition
        // (LOW_POWER ↔ ACTIVE_IDLE) takes effect on the very next tick.
        // The vTaskDelay here is what releases the CPU back to the IDF
        // PM layer; longer delays = more tickless light-sleep coverage.
        vTaskDelay(pdMS_TO_TICKS(s_poll_period_ms));
    }
}

// ---------------------------------------------------------------------
// Power-state subscriber: throttles the poll period in idle states.
// ---------------------------------------------------------------------

static void on_power_state(const sk_event_t *evt, void *user)
{
    (void)user;
    if (!evt || !evt->payload_json) return;
    // Lightweight string match — payload is small ({"state":"...","since_ms":N}).
    const char *p = evt->payload_json;
    int new_period;
    if (strstr(p, "\"low_power\"")) {
        new_period = POLL_PERIOD_LOW_MS;
    } else if (strstr(p, "\"screen_off\"")) {
        new_period = POLL_PERIOD_SCREEN_MS;
    } else {
        // active_timer / active_idle → full responsiveness
        new_period = POLL_PERIOD_ACTIVE_MS;
    }
    if (new_period != s_poll_period_ms) {
        ESP_LOGI(TAG, "poll period %d → %d ms (power.state)",
                 s_poll_period_ms, new_period);
        s_poll_period_ms = new_period;
    }
}

// ---------------------------------------------------------------------
// Public API + CLI
// ---------------------------------------------------------------------

int bf_face_detector_current(void) { return s_committed_face; }

static sk_err_t cmd_face_status(sk_cli_ctx_t *ctx)
{
    int64_t now = esp_timer_get_time();
    int64_t pending_ms = s_pending_started_us
                            ? (now - s_pending_started_us) / 1000 : 0;
    int64_t tilt_ms    = s_tilt_started_us
                            ? (now - s_tilt_started_us) / 1000    : 0;
    char buf[192];
    snprintf(buf, sizeof(buf),
             "{\"face\":%d,\"label\":\"%s\","
             "\"pending\":%d,\"pending_ms\":%lld,"
             "\"tilted\":%s,\"tilt_ms\":%lld,"
             "\"poll_period_ms\":%d}",
             s_committed_face, face_label(s_committed_face),
             s_pending_face, (long long)pending_ms,
             s_tilted ? "true" : "false", (long long)tilt_ms,
             s_poll_period_ms);
    sk_cli_ok(ctx, buf);
    return SK_OK;
}

static const sk_cli_command_t s_cmd_status = {
    .name    = "face.status",
    .summary = "Current cube face + pending debounce state",
    .usage   = "face status",
    .handler = cmd_face_status,
};

esp_err_t bf_face_detector_init(void)
{
    static bool s_inited = false;
    if (s_inited) return ESP_OK;
    s_inited = true;

    // CLI face.status removed — face detection is fully internal; users see
    // the result on screen, no need to expose raw state via CLI.
    (void)s_cmd_status;
    sk_capabilities_register_book("bf_face_detector", "0.2.0");

    // Stack sized for the publish-call chain: face.changed subscribers
    // run synchronously in this task's context. With ESP_LOGI inside
    // multiple subscribers the chain can use ~3 KB, so we provision more
    // than 1.5× headroom.
    BaseType_t r = xTaskCreate(poll_task, "bf_face", 5120, NULL, 4, NULL);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "poll task spawn failed");
        return ESP_FAIL;
    }

    // Adapt poll rate to power state. A subscribe failure here would
    // pin us at 10 Hz forever — wastes battery but doesn't break
    // function — so we log and continue rather than fail init.
    int sub;
    esp_err_t serr = sk_event_bus_subscribe("power.state", on_power_state,
                                            NULL, &sub);
    if (serr != ESP_OK) {
        ESP_LOGW(TAG, "power.state subscribe failed: %s — staying at %d Hz",
                 esp_err_to_name(serr), 1000 / POLL_PERIOD_ACTIVE_MS);
    }

    ESP_LOGI(TAG, "ready (poll=%d/%d/%d ms, debounce=%d ms, sensor=%s)",
             POLL_PERIOD_ACTIVE_MS, POLL_PERIOD_SCREEN_MS, POLL_PERIOD_LOW_MS,
             DEBOUNCE_MS, bf_lis3dsh_present() ? "ok" : "absent");
    return ESP_OK;
}
