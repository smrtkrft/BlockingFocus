// =====================================================================
// bf_timer_engine — implementation. See header + docs/state_machine.md.
//
// All state lives in this single task. Inputs come in via s_evt_q
// (face change, periodic tick, cooldown timeout, CLI cancel). The task
// loop blocks on the queue; nobody else mutates state, so no locking.
// =====================================================================

#include "bf_timer_engine.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "bf_vibration.h"
#include "bf_face_detector.h"
#include "sk_api.h"
#include "sk_capabilities.h"
#include "sk_cli.h"
#include "sk_errors.h"
#include "sk_event_bus.h"
#include "sk_identity.h"

static const char *TAG = "bf_timer";

// ---------------------------------------------------------------------
// Face → duration mapping (project-tunable)
//
// Indices match BF_FACE_* values from bf_face_detector.h.
//   > 0  → countdown duration in seconds (timer face)
//   = 0  → display face: no timer, just arms the next one
//   = -1 → USB face: low-power state
// =====================================================================

#define FACE_DISPLAY_DURATION   0
#define FACE_USB_DURATION      (-1)

static const int s_face_duration_sec[7] = {
    [BF_FACE_NONE]   = 0,
    [BF_FACE_Z_UP]   = FACE_DISPLAY_DURATION,   // display face (top)
    [BF_FACE_Z_DOWN] = FACE_USB_DURATION,       // USB face (bottom)
    [BF_FACE_X_UP]   =  5 * 60,                 // 5 min
    [BF_FACE_X_DOWN] = 15 * 60,                 // 15 min
    [BF_FACE_Y_UP]   = 30 * 60,                 // 30 min
    [BF_FACE_Y_DOWN] = 60,                      // TEST: 60 sn (was 60 min)
};

#define LOCK_REMAINING_SEC      60
#define COOLDOWN_SEC            10
#define MAX_API_ENDPOINTS       SK_API_MAX_ENDPOINTS

// Final-warning vibration: bu eşiğe geldiğinde (kalan saniye == eşik)
// titreşim motoru tek pulse şeklinde WARN_VIBRATION_MS boyunca çalışır.
#define WARN_REMAINING_SEC      5
#define WARN_VIBRATION_MS       1000

// Geri sayım bittiğinde "gel-git" titreşim deseni: 4 darbe × 500 ms ON
// + 250 ms OFF ≈ 2750 ms toplam. Her ON pencereesi pancake motorun
// ramp-up süresinden (~100 ms) yeterince uzun olmalı ki motor gerçekten
// dönsün — 250 ms denenmiş, motor sadece titriyor ama dönmüyordu.
#define EXPIRE_BURST_COUNT      4
#define EXPIRE_BURST_ON_MS      500
#define EXPIRE_BURST_OFF_MS     250

// ---------------------------------------------------------------------
// Internal event queue
// ---------------------------------------------------------------------

typedef enum {
    EVT_FACE_CHANGED,
    EVT_TICK,
    EVT_COOLDOWN_DONE,
    EVT_CANCEL,
    EVT_API_TRIGGER_DONE,   // posted by api_trigger_task once the whole
                            // sequential fire-+-delay chain finishes
} evt_type_t;

typedef struct {
    evt_type_t type;
    int        face;       // EVT_FACE_CHANGED only
} evt_t;

static QueueHandle_t      s_evt_q   = NULL;
static esp_timer_handle_t s_tick_t  = NULL;   // 1 Hz while counting
static esp_timer_handle_t s_cool_t  = NULL;   // one-shot, COOLDOWN_SEC

// ---------------------------------------------------------------------
// State + countdown bookkeeping
// ---------------------------------------------------------------------

static bf_timer_state_t s_state          = BF_TIMER_BOOT;
static int              s_active_face    = BF_FACE_NONE;
static int              s_active_duration_sec = 0;
static int64_t          s_deadline_us    = 0;     // esp_timer_get_time when timer hits 0
static int              s_paused_remaining_sec = 0;

static const char *state_str(bf_timer_state_t s)
{
    switch (s) {
    case BF_TIMER_BOOT:                     return "boot";
    case BF_TIMER_IDLE_NEEDS_ARM:           return "idle_needs_arm";
    case BF_TIMER_ARMED:                    return "armed";
    case BF_TIMER_RESETTABLE:               return "timer_resettable";
    case BF_TIMER_LOCKED:                   return "timer_locked";
    case BF_TIMER_PAUSED:                   return "paused";
    case BF_TIMER_EXPIRED:                  return "expired";
    case BF_TIMER_API_TRIGGERING:           return "api_triggering";
    case BF_TIMER_COOLDOWN:                 return "cooldown";
    case BF_TIMER_COOLDOWN_NEEDS_FACE_CHG:  return "cooldown_needs_face_chg";
    case BF_TIMER_LOW_POWER:                return "low_power";
    default:                                return "?";
    }
}

static int compute_remaining_sec(void)
{
    // While paused, the live deadline has been cleared — surface the
    // saved value instead so timer.state listeners (UI especially) see
    // the actual remaining time on the pause edge, not 0.
    if (s_state == BF_TIMER_PAUSED) return s_paused_remaining_sec;
    if (s_deadline_us == 0) return 0;
    int64_t now = esp_timer_get_time();
    int64_t left_us = s_deadline_us - now;
    if (left_us <= 0) return 0;
    return (int)(left_us / 1000000);
}

// ---------------------------------------------------------------------
// State publish helpers
// ---------------------------------------------------------------------

static void publish_state(void)
{
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"state\":\"%s\",\"remaining_sec\":%d,\"face\":%d}",
             state_str(s_state),
             compute_remaining_sec(),
             s_active_face);
    sk_event_bus_publish("timer.state", buf);
    ESP_LOGI(TAG, "→ %s (face=%d, remaining=%d s)",
             state_str(s_state), s_active_face, compute_remaining_sec());
}

static void publish_tick(int remaining)
{
    char buf[48];
    snprintf(buf, sizeof(buf), "{\"remaining_sec\":%d}", remaining);
    sk_event_bus_publish("timer.tick", buf);
}

static void publish_expired(void)
{
    char buf[80];
    snprintf(buf, sizeof(buf),
             "{\"face\":%d,\"duration_sec\":%d}",
             s_active_face, s_active_duration_sec);
    sk_event_bus_publish("timer.expired", buf);
}

// ---------------------------------------------------------------------
// Tick / cooldown timer callbacks (run in esp_timer task — just enqueue)
// ---------------------------------------------------------------------

static void on_tick(void *arg)
{
    (void)arg;
    if (!s_evt_q) return;
    evt_t e = { .type = EVT_TICK };
    xQueueSend(s_evt_q, &e, 0);
}

static void on_cooldown_done(void *arg)
{
    (void)arg;
    if (!s_evt_q) return;
    evt_t e = { .type = EVT_COOLDOWN_DONE };
    xQueueSend(s_evt_q, &e, 0);
}

static void start_tick_timer(void)
{
    if (s_tick_t && !esp_timer_is_active(s_tick_t)) {
        esp_timer_start_periodic(s_tick_t, 1000000);  // 1 s
    }
}

static void stop_tick_timer(void)
{
    if (s_tick_t && esp_timer_is_active(s_tick_t)) {
        esp_timer_stop(s_tick_t);
    }
}

// ---------------------------------------------------------------------
// API trigger — sequential fire with per-endpoint inter-call delays
//
// Endpoints fire in NVS slot order (0..MAX-1, in_use only). Between two
// endpoints we wait `delay_after_sec` of the one that just fired. The
// last endpoint's delay is skipped — pointless to delay the cooldown
// state after the chain ends. The delay is time-based: we don't wait
// for the HTTP response, we just sleep, so a misbehaving endpoint can't
// stall the chain (sk_api itself handles retries + 10 s timeout).
// ---------------------------------------------------------------------

static void api_trigger_task(void *arg)
{
    int duration_sec = (int)(intptr_t)arg;

    sk_api_endpoint_t *list = calloc(MAX_API_ENDPOINTS, sizeof(*list));
    if (!list) {
        ESP_LOGW(TAG, "api_trigger_task OOM — skipping all sends");
        if (s_evt_q) {
            evt_t e = { .type = EVT_API_TRIGGER_DONE };
            xQueueSend(s_evt_q, &e, 0);
        }
        vTaskDelete(NULL);
        return;
    }

    int n = sk_api_endpoint_list(list, MAX_API_ENDPOINTS);

    // The bus payload is sent as the HTTP body for **every** endpoint type
    // we ship (generic POST/PUT, webhook_post, IFTTT). All three default to
    // Content-Type: application/json — and IFTTT specifically expects the
    // {"value1":...,"value2":...,"value3":...} contract on the Maker
    // webhook. Until now we sent a plain "Focus session ended (...)" line,
    // which the server saw as malformed JSON, returned 400, and surfaced
    // as `SK_ERR_API_BAD_STATUS` → the OOPS scene with no fix path on the
    // user's end. Switching to a valid JSON object satisfies both
    // generic JSON listeners and the IFTTT preset in one go; servers that
    // want plain text can flip Content-Type on the endpoint config.
    // `event` is the canonical BLE event-bus name (`timer.expired`) so
    // SKAPP-side bindings whose eventFilter == "timer.expired" match
    // the same way they do over BLE/TCP CLI. Earlier this said
    // `focus_session_ended` which was IFTTT-shaped but caused the new
    // bond-signed webhook → SKAPI binding pipeline to never match.
    //
    // The IFTTT preset still needs the human trigger label so we keep
    // `value1=focus_session_ended` for backward-compat with existing
    // Maker applets (their "this happened" string must match).
    char payload[224];
    snprintf(payload, sizeof(payload),
             "{\"event\":\"timer.expired\","
             "\"duration_min\":%d,"
             "\"face\":%d,"
             "\"device\":\"%s\","
             "\"value1\":\"focus_session_ended\","
             "\"value2\":\"%d\","
             "\"value3\":\"%s\"}",
             duration_sec / 60, s_active_face, sk_identity_get(),
             duration_sec / 60, sk_identity_get());

    int fired = 0;
    for (int i = 0; i < n; i++) {
        if (!list[i].in_use) continue;
        // Fire log: surfaces the exact endpoint kind + URL that's about
        // to be hit. Critical for diagnosing webhook routing issues —
        // when SKAPP shows zero received webhooks despite BF reporting
        // "fired N", this line tells us where each request actually
        // went (orphan URL? wrong path? user-added USER endpoint?).
        ESP_LOGW(TAG, "fire kind=%s name=%s url=%s",
                 list[i].kind == SK_API_KIND_SYSTEM ? "system" : "user",
                 list[i].name, list[i].url);
        esp_err_t err = sk_api_send(list[i].name, payload);
        if (err == ESP_OK) {
            fired++;
        } else {
            ESP_LOGW(TAG, "sk_api_send(%s) failed: %s",
                     list[i].name, esp_err_to_name(err));
        }
        bool more_to_come = (i + 1 < n);
        if (more_to_come && list[i].delay_after_sec > 0) {
            ESP_LOGI(TAG, "delay %u s before next endpoint",
                     (unsigned)list[i].delay_after_sec);
            vTaskDelay(pdMS_TO_TICKS((TickType_t)list[i].delay_after_sec * 1000));
        }
    }
    ESP_LOGI(TAG, "fired %d API endpoint(s) sequentially", fired);
    free(list);

    if (s_evt_q) {
        evt_t e = { .type = EVT_API_TRIGGER_DONE };
        xQueueSend(s_evt_q, &e, 0);
    }
    vTaskDelete(NULL);
}

// ---------------------------------------------------------------------
// Transitions
// ---------------------------------------------------------------------

static void enter(bf_timer_state_t s)
{
    s_state = s;
    publish_state();
}

static void start_countdown(int duration_sec)
{
    s_active_duration_sec = duration_sec;
    s_deadline_us = esp_timer_get_time() + (int64_t)duration_sec * 1000000;
    bf_timer_state_t target = (duration_sec <= LOCK_REMAINING_SEC)
                              ? BF_TIMER_LOCKED
                              : BF_TIMER_RESETTABLE;
    enter(target);
    start_tick_timer();
}

static void clear_countdown(void)
{
    stop_tick_timer();
    s_deadline_us = 0;
    s_active_duration_sec = 0;
    s_active_face = BF_FACE_NONE;
}

static void pause_countdown(void)
{
    s_paused_remaining_sec = compute_remaining_sec();
    stop_tick_timer();
    s_deadline_us = 0;
    enter(BF_TIMER_PAUSED);
}

static void resume_countdown(void)
{
    if (s_paused_remaining_sec <= 0) {
        clear_countdown();
        enter(BF_TIMER_IDLE_NEEDS_ARM);
        return;
    }
    // Capture the saved remaining BEFORE we zero it. compute_remaining_sec
    // returns s_paused_remaining_sec while s_state is still PAUSED, so if
    // we zeroed first the LOCKED threshold check would see 0 and wrongly
    // drop us into LOCKED on every resume — which then froze the cube
    // because LOCKED ignores all face changes (60 s rule).
    int saved = s_paused_remaining_sec;
    bf_timer_state_t target = (saved <= LOCK_REMAINING_SEC)
                              ? BF_TIMER_LOCKED
                              : BF_TIMER_RESETTABLE;
    s_deadline_us = esp_timer_get_time() + (int64_t)saved * 1000000;
    s_paused_remaining_sec = 0;
    enter(target);
    start_tick_timer();
}

// NOTE: BF_TIMER_LOW_POWER (header enum) is currently unreachable from
// handle_face — bf_power has its own state machine and we deliberately
// don't escalate timer state on USB-face anymore (see comment in
// handle_face). Enum + state_str mapping kept so a future re-enable
// stays a one-line change; the helper that used to set it has been
// removed to silence -Wunused-function and avoid bit-rot.

static void enter_cooldown(void)
{
    clear_countdown();
    enter(BF_TIMER_COOLDOWN);
    if (s_cool_t) {
        esp_timer_start_once(s_cool_t, (uint64_t)COOLDOWN_SEC * 1000000);
    }
}

static void on_expired(void)
{
    // FAZ0_DIAG: gecikme teşhisi için T0 anchor (us-precision, monotonic).
    ESP_LOGW(TAG, "FAZ0_DIAG T0_expire us=%lld", esp_timer_get_time());

    // Geri sayım bitiminde "gel-git" titreşim — sürekli motor yerine
    // 6 × 250 ms ON / 250 ms OFF (3 sn toplam). Pulsing pattern net
    // "süre bitti" feedback'i, sürekli sürmekten daha algılanır.
    bf_vibration_burst(EXPIRE_BURST_COUNT,
                       EXPIRE_BURST_ON_MS,
                       EXPIRE_BURST_OFF_MS);
    publish_expired();
    enter(BF_TIMER_EXPIRED);

    enter(BF_TIMER_API_TRIGGERING);
    // Spawn the sequential trigger as a background task so this state-
    // machine task remains responsive (CLI commands, cancel button,
    // etc.). Cooldown is entered later via EVT_API_TRIGGER_DONE.
    // HTTPS/TLS handshake from sk_api is stack-heavy (mbedtls verifies
    // certificate bundle). 4096 occasionally crashed during simultaneous
    // outbound triggers — 8192 keeps TLS comfortable.
    BaseType_t r = xTaskCreate(api_trigger_task, "bf_api_seq",
                               8192,
                               (void *)(intptr_t)s_active_duration_sec,
                               4, NULL);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "api_trigger_task spawn failed — entering cooldown directly");
        enter_cooldown();
    }
}

// ---------------------------------------------------------------------
// Per-state event handling
// ---------------------------------------------------------------------

static void handle_face(int face)
{
    int dur = bf_timer_engine_face_duration(face);
    ESP_LOGD(TAG, "handle_face face=%d dur=%d state=%s", face, dur, state_str(s_state));

    // Display-down face. bf_power independently turns the OLED off via
    // its s_face_down override — bf_timer's only job here is to decide
    // what to do with the countdown:
    //   RESETTABLE → pause and save remaining
    //   everything else → leave state unchanged
    //
    // We deliberately do NOT enter BF_TIMER_LOW_POWER from PAUSED/
    // IDLE/ARMED anymore: that path would clear_countdown on exit and
    // re-route through IDLE_NEEDS_ARM, which ignores timer faces and
    // prevents a direct Z-→side-face flow from starting a countdown.
    if (dur == FACE_USB_DURATION) {
        s_active_face = face;
        if (s_state == BF_TIMER_RESETTABLE) {
            pause_countdown();
        }
        return;
    }

    s_active_face = face;

    switch (s_state) {

    case BF_TIMER_BOOT:
        // Initial face observation just decides our resting state.
        if (dur == FACE_DISPLAY_DURATION)      enter(BF_TIMER_ARMED);
        else                                    enter(BF_TIMER_IDLE_NEEDS_ARM);
        return;

    case BF_TIMER_IDLE_NEEDS_ARM:
        if (dur == FACE_DISPLAY_DURATION) enter(BF_TIMER_ARMED);
        // Timer faces in IDLE are ignored — must arm via display first.
        return;

    case BF_TIMER_ARMED:
        if (dur > 0) start_countdown(dur);
        // Display face stays in ARMED.
        return;

    case BF_TIMER_RESETTABLE:
        if (dur == FACE_DISPLAY_DURATION) {
            pause_countdown();
        } else if (dur > 0) {
            start_countdown(dur);   // reset to new duration
        }
        return;

    case BF_TIMER_LOCKED:
        // Per spec rule #1 — final 60 s ignores all face changes.
        return;

    case BF_TIMER_PAUSED:
        if (dur > 0) {
            // Same face → resume saved remaining; different timer face →
            // start fresh with the new duration. (Doc both routes through
            // RESETTABLE; the difference is whether we keep saved time.)
            int prev_face = s_active_face;
            (void)prev_face;
            // Heuristic: if the new face matches the originally-paused
            // duration, resume; otherwise reset. Here we just use the
            // current face's duration — simpler and matches user intent
            // (whatever face is up now is the one that should run).
            if (s_paused_remaining_sec > 0 &&
                dur == s_active_duration_sec) {
                resume_countdown();
            } else {
                start_countdown(dur);
            }
        }
        return;

    case BF_TIMER_LOW_POWER:
        // Anything other than USB face wakes us up.
        if (dur != FACE_USB_DURATION) {
            clear_countdown();
            enter(BF_TIMER_IDLE_NEEDS_ARM);
            // Re-feed the current face so we land in the right resting state.
            handle_face(face);
        }
        return;

    case BF_TIMER_COOLDOWN_NEEDS_FACE_CHG:
        // 2026-05-08: cooldown sonrası herhangi bir timer yüzüne dönmek
        // doğrudan yeni countdown başlatır — display yüzünden geçme
        // zorunluluğu kaldırıldı (kullanıcı isteği). Eski davranışta
        // önce IDLE_NEEDS_ARM → display face → ARMED → timer face zorunluydu.
        if (dur > 0) {
            start_countdown(dur);
        } else if (dur == FACE_DISPLAY_DURATION) {
            // Display face → ARMED (klasik). Sonra herhangi bir timer
            // yüzüne dönerse oradan countdown başlar.
            enter(BF_TIMER_ARMED);
        } else {
            // USB face → IDLE'a in.
            clear_countdown();
            enter(BF_TIMER_IDLE_NEEDS_ARM);
        }
        return;

    case BF_TIMER_EXPIRED:
    case BF_TIMER_API_TRIGGERING:
    case BF_TIMER_COOLDOWN:
        // Transitional / blocking states — ignore face during these.
        return;
    }
}

static void handle_tick(void)
{
    if (s_state != BF_TIMER_RESETTABLE && s_state != BF_TIMER_LOCKED) {
        // Stale tick from before stop — defensive.
        stop_tick_timer();
        return;
    }

    int remaining = compute_remaining_sec();
    publish_tick(remaining);

    if (remaining <= 0) {
        on_expired();
        return;
    }
    // Final-warning vibration: tam WARN_REMAINING_SEC kalınca tek pulse.
    // Tick 1 Hz → her geri sayım için en fazla 1 kez tetiklenir.
    if (remaining == WARN_REMAINING_SEC) {
        bf_vibration_pulse_ms(WARN_VIBRATION_MS);
    }
    if (s_state == BF_TIMER_RESETTABLE && remaining <= LOCK_REMAINING_SEC) {
        enter(BF_TIMER_LOCKED);
    }
}

static void handle_cooldown_done(void)
{
    if (s_state != BF_TIMER_COOLDOWN) return;
    enter(BF_TIMER_COOLDOWN_NEEDS_FACE_CHG);
}

static void handle_cancel(void)
{
    clear_countdown();
    s_paused_remaining_sec = 0;
    enter(BF_TIMER_IDLE_NEEDS_ARM);
}

// ---------------------------------------------------------------------
// Task loop
// ---------------------------------------------------------------------

static void timer_task(void *arg)
{
    (void)arg;
    for (;;) {
        evt_t e;
        if (xQueueReceive(s_evt_q, &e, portMAX_DELAY) != pdTRUE) continue;
        switch (e.type) {
        case EVT_FACE_CHANGED:      handle_face(e.face);    break;
        case EVT_TICK:              handle_tick();          break;
        case EVT_COOLDOWN_DONE:     handle_cooldown_done(); break;
        case EVT_CANCEL:            handle_cancel();        break;
        case EVT_API_TRIGGER_DONE:
            // Whole sequential chain finished. Only enter cooldown if
            // we're still in API_TRIGGERING — a CLI cancel or face-down
            // override could have moved us elsewhere in the meantime.
            if (s_state == BF_TIMER_API_TRIGGERING) enter_cooldown();
            break;
        }
    }
}

// ---------------------------------------------------------------------
// Event subscription
// ---------------------------------------------------------------------

static void on_face_event(const sk_event_t *evt, void *user)
{
    (void)user;
    if (!evt->payload_json || !s_evt_q) return;
    const char *p = strstr(evt->payload_json, "\"face\":");
    if (!p) return;
    int face = atoi(p + 7);
    evt_t out = { .type = EVT_FACE_CHANGED, .face = face };
    if (xQueueSend(s_evt_q, &out, 0) != pdTRUE) {
        ESP_LOGW(TAG, "queue full, face=%d lost", face);
    }
}

// ---------------------------------------------------------------------
// Public getters + CLI
// ---------------------------------------------------------------------

bf_timer_state_t  bf_timer_engine_state(void)        { return s_state; }
const char       *bf_timer_engine_state_str(void)    { return state_str(s_state); }
int               bf_timer_engine_remaining_sec(void){ return compute_remaining_sec(); }

int bf_timer_engine_face_duration(int face)
{
    if (face < 0 || face >= (int)(sizeof(s_face_duration_sec) /
                                  sizeof(s_face_duration_sec[0]))) {
        return -2;
    }
    return s_face_duration_sec[face];
}

static sk_err_t cmd_timer_status(sk_cli_ctx_t *ctx)
{
    char buf[160];
    snprintf(buf, sizeof(buf),
             "{\"state\":\"%s\",\"face\":%d,\"duration_sec\":%d,"
             "\"remaining_sec\":%d,\"paused_remaining_sec\":%d}",
             state_str(s_state),
             s_active_face,
             s_active_duration_sec,
             compute_remaining_sec(),
             s_paused_remaining_sec);
    sk_cli_ok(ctx, buf);
    return SK_OK;
}

static sk_err_t cmd_timer_cancel(sk_cli_ctx_t *ctx)
{
    if (!s_evt_q) {
        sk_cli_err(ctx, SK_ERR_BUSY, "{\"reason\":\"engine_not_ready\"}");
        return SK_OK;
    }
    evt_t e = { .type = EVT_CANCEL };
    xQueueSend(s_evt_q, &e, 0);
    sk_cli_ok(ctx, "{\"cancelled\":true}");
    return SK_OK;
}

static const sk_cli_command_t s_cmd_status = {
    .name    = "timer.status",
    .summary = "State machine snapshot",
    .usage   = "timer status",
    .handler = cmd_timer_status,
};
static const sk_cli_command_t s_cmd_cancel = {
    .name    = "timer.cancel",
    .summary = "Abort the current countdown (state reset)",
    .usage   = "timer cancel",
    .handler = cmd_timer_cancel,
};

// ---------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------

esp_err_t bf_timer_engine_init(void)
{
    if (s_evt_q) return ESP_OK;   // idempotent

    s_evt_q = xQueueCreate(16, sizeof(evt_t));
    if (!s_evt_q) return ESP_ERR_NO_MEM;

    const esp_timer_create_args_t tick_args = {
        .callback = on_tick,            .name = "bf_timer_tick",
    };
    if (esp_timer_create(&tick_args, &s_tick_t) != ESP_OK) {
        vQueueDelete(s_evt_q); s_evt_q = NULL;
        return ESP_FAIL;
    }
    const esp_timer_create_args_t cool_args = {
        .callback = on_cooldown_done,   .name = "bf_timer_cool",
    };
    if (esp_timer_create(&cool_args, &s_cool_t) != ESP_OK) {
        esp_timer_delete(s_tick_t); s_tick_t = NULL;
        vQueueDelete(s_evt_q); s_evt_q = NULL;
        return ESP_FAIL;
    }

    int sub;
    esp_err_t sub_err = sk_event_bus_subscribe("face.changed",
                                                on_face_event, NULL, &sub);
    if (sub_err != ESP_OK) {
        ESP_LOGE(TAG, "FAILED to subscribe face.changed: %s — countdown will be DEAD",
                 esp_err_to_name(sub_err));
    } else {
        ESP_LOGI(TAG, "subscribed face.changed (id=%d)", sub);
    }

    // CLI timer.status / timer.cancel removed — countdown is shown on the
    // OLED in real time, and cancellation is a physical-device action.
    // Engine still owns its state machine; just no CLI surface.
    (void)s_cmd_status; (void)s_cmd_cancel;
    sk_capabilities_register_book("bf_timer_engine", "0.1.0");

    // 4096 → 6144: state-machine task fires multiple ESP_LOG lines per
    // transition; 4096 was uncomfortably close to vfprintf's worst case
    // when peer-connect events stacked on top of face.changed.
    BaseType_t r = xTaskCreate(timer_task, "bf_timer", 6144, NULL, 5, NULL);
    if (r != pdPASS) {
        esp_timer_delete(s_tick_t); esp_timer_delete(s_cool_t);
        vQueueDelete(s_evt_q); s_evt_q = NULL;
        return ESP_FAIL;
    }

    s_state = BF_TIMER_IDLE_NEEDS_ARM;
    publish_state();
    ESP_LOGI(TAG, "ready (faces 5/15/30 min + 60 sn TEST on X-/X+/Y+/Y-)");
    return ESP_OK;
}
