// =====================================================================
// bf_power v0.1 — see header for scope notes. State is mutated only by
// the periodic check task and the event handlers, which all enqueue
// onto a single FreeRTOS queue and run serially.
// =====================================================================

#include "bf_power.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "bf_display.h"
#include "pins.h"
#include "sk_capabilities.h"
#include "sk_cli.h"
#include "sk_errors.h"
#include "sk_event_bus.h"
// sk_wifi.h no longer needed: WiFi stays in WIFI_PS_MAX_MODEM at all
// times (set by sk_wifi after GOT_IP). bf_power doesn't disconnect.

static const char *TAG = "bf_power";

#define SCREEN_OFF_MS    (5  * 60 * 1000)   // 5 min idle → OLED display off cmd
#define LOW_POWER_MS     (10 * 60 * 1000)   // 10 min no countdown → also disconnect wifi
#define CHECK_PERIOD_MS  10000              // re-evaluate every 10 s

// ---------------------------------------------------------------------
// Event queue
// ---------------------------------------------------------------------

typedef enum {
    EVT_TIMER_STATE,         // timer state changed
    EVT_FACE_CHANGED,        // user moved the cube (i = face number)
    EVT_BUTTON,              // button.released
    EVT_TICK,                // periodic check
    EVT_KICK,                // explicit bf_power_kick() call
} evt_type_t;

typedef struct {
    evt_type_t type;
    bool       timer_running;   // EVT_TIMER_STATE only
    int        face;            // EVT_FACE_CHANGED only
} evt_t;

static QueueHandle_t      s_q       = NULL;
static esp_timer_handle_t s_tick_t  = NULL;

static bf_power_state_t s_state             = BF_POWER_ACTIVE_IDLE;
static int64_t          s_last_activity_us  = 0;
static int64_t          s_last_countdown_us = 0;
static bool             s_timer_running     = false;
static int64_t          s_state_entered_us  = 0;

// ESP_PM_NO_LIGHT_SLEEP lock — held during ACTIVE_* states, released
// during SCREEN_OFF / LOW_POWER. Two reasons:
//   1) Light-sleep power-gates the USB phy. With the lock held while a
//      developer is interacting with the device, USB Serial/JTAG stays
//      enumerated and `idf.py monitor` keeps its connection. Once the
//      device falls into SCREEN_OFF (5 min idle) the user almost
//      certainly isn't watching the monitor anymore, so dropping the
//      lock to claim battery savings is safe.
//   2) WiFi association + DHCP run noticeably faster when the CPU isn't
//      doing micro-sleeps between every packet — boot becomes ~1 s
//      faster on weak APs.
// State guard (s_no_sleep_held) keeps acquire/release calls balanced
// across redundant transitions (the apply() path runs on every state
// change, including no-op equal-to-current).
static esp_pm_lock_handle_t s_no_sleep_lock = NULL;
static bool                 s_no_sleep_held = false;

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------

static const char *state_str(bf_power_state_t s)
{
    switch (s) {
    case BF_POWER_ACTIVE_TIMER: return "active_timer";
    case BF_POWER_ACTIVE_IDLE:  return "active_idle";
    case BF_POWER_SCREEN_OFF:   return "screen_off";
    case BF_POWER_LOW_POWER:    return "low_power";
    default:                    return "?";
    }
}

static void publish_state(void)
{
    int64_t since = (esp_timer_get_time() - s_state_entered_us) / 1000;
    char buf[96];
    snprintf(buf, sizeof(buf),
             "{\"state\":\"%s\",\"since_ms\":%lld}",
             state_str(s_state), (long long)since);
    sk_event_bus_publish("power.state", buf);
    ESP_LOGI(TAG, "→ %s (since %lld ms)", state_str(s_state), (long long)since);
}

// ---------------------------------------------------------------------
// Apply the I/O side-effects for a given state.
// ---------------------------------------------------------------------

static void hold_no_sleep(bool hold)
{
    if (!s_no_sleep_lock) return;
    if (hold && !s_no_sleep_held) {
        esp_pm_lock_acquire(s_no_sleep_lock);
        s_no_sleep_held = true;
    } else if (!hold && s_no_sleep_held) {
        esp_pm_lock_release(s_no_sleep_lock);
        s_no_sleep_held = false;
    }
}

static void apply(bf_power_state_t s)
{
    switch (s) {
    case BF_POWER_ACTIVE_TIMER:
    case BF_POWER_ACTIVE_IDLE:
        // OLED on — wakes from sleep cmd if needed.
        bf_display_sleep(false);
        // Block light-sleep so USB Serial/JTAG monitor stays alive and
        // WiFi/BLE peer interactions don't pay the sleep-entry latency.
        hold_no_sleep(true);
        break;

    case BF_POWER_SCREEN_OFF:
        // OLED into ~1-5 µA standby (0xAE). Panel RAM preserved.
        bf_display_sleep(true);
        // bf_power lock'unu SERBEST BIRAK — kullanıcı 5 dk hareketsiz,
        // tickless light_sleep'e izin ver. USB Serial/JTAG flash
        // güvenliği IDF driver'ın kendi `usb_serial_jtag` lock'una
        // delegasyon: CONFIG_USJ_NO_AUTO_LS_ON_CONNECTION=y ile host
        // bağlıyken otomatik tutulur (SOF-driven, ~30 ms latency),
        // pille çalışırken serbest. Sonuç: dev'de port her zaman
        // canlı, pille idle ~10-15 mA → 40+ saat ömür.
        hold_no_sleep(false);
        break;

    case BF_POWER_LOW_POWER:
        // 10 dk countdown yok → derin idle. bf_power lock'u serbest;
        // light_sleep aktif (USB takılı değilse). Tasarruf:
        //   1) face_detector polling 1 Hz'e düşer
        //   2) Tickless idle çoğu zaman light_sleep'te
        //   3) WiFi DTIM modem-sleep aligned
        // USB takılıysa IDF driver kendi NO_LIGHT_SLEEP lock'unu
        // tutar (config flag), light_sleep girmez. Pille hot-plug
        // sırasında aynı driver lock'u SOF kaybolduğunda 3 tick
        // içinde release eder → otomatik power-save.
        bf_display_sleep(true);
        hold_no_sleep(false);
        break;
    }
}

static void enter(bf_power_state_t s)
{
    if (s == s_state) return;
    s_state = s;
    s_state_entered_us = esp_timer_get_time();
    apply(s);
    publish_state();
}

// ---------------------------------------------------------------------
// State decisions
// ---------------------------------------------------------------------

// Sticky face-down flag — set by face.changed when the display face is
// at the bottom (BF_FACE_Z_DOWN = 2). While true, we force SCREEN_OFF
// regardless of timer / activity state. Cleared the moment the cube is
// rotated to any other face. This is the "fast wake" path the user
// asked for: one I2C cmd flips the OLED back on, no wifi reconnect
// needed because we never disconnected.
static bool s_face_down = false;

static void recompute(void)
{
    int64_t now = esp_timer_get_time();

    // Highest-priority override: display face hidden → screen off.
    if (s_face_down) {
        enter(BF_POWER_SCREEN_OFF);
        return;
    }

    // Timer running takes precedence over everything else.
    if (s_timer_running) {
        s_last_countdown_us = now;
        s_last_activity_us  = now;
        enter(BF_POWER_ACTIVE_TIMER);
        return;
    }

    int64_t idle_ms       = (now - s_last_activity_us)  / 1000;
    int64_t no_count_ms   = (now - s_last_countdown_us) / 1000;

    if (no_count_ms >= LOW_POWER_MS) {
        enter(BF_POWER_LOW_POWER);
    } else if (idle_ms >= SCREEN_OFF_MS) {
        enter(BF_POWER_SCREEN_OFF);
    } else {
        enter(BF_POWER_ACTIVE_IDLE);
    }
}

// ---------------------------------------------------------------------
// Event handlers (all run in the bf_power task)
// ---------------------------------------------------------------------

static void handle_event(const evt_t *e)
{
    int64_t now = esp_timer_get_time();
    switch (e->type) {
    case EVT_TIMER_STATE:
        s_timer_running = e->timer_running;
        if (s_timer_running) s_last_countdown_us = now;
        s_last_activity_us = now;
        recompute();
        break;

    case EVT_FACE_CHANGED:
        s_last_activity_us = now;
        // Display face = +Z up = BF_FACE_Z_UP (1).
        // Display face DOWN = -Z up = BF_FACE_Z_DOWN (2). When the
        // physical display surface is on the bottom, the user can't
        // see anything → cut the OLED. Any other face wakes immediately.
        s_face_down = (e->face == 2);
        // Rotating the cube counts as a deliberate "I'm here" signal:
        // also reset the no-countdown timer so we leave LOW_POWER, not
        // just SCREEN_OFF. Without this, recompute() trips the
        // no_count_ms branch first and the screen stays dark even
        // though the user just woke the device.
        if (!s_face_down) s_last_countdown_us = now;
        ESP_LOGD(TAG, "face.changed face=%d face_down=%d",
                 e->face, (int)s_face_down);
        recompute();
        break;

    case EVT_BUTTON:
    case EVT_KICK:
        s_last_activity_us  = now;
        // Same reasoning as EVT_FACE_CHANGED: explicit user input must
        // pull us out of LOW_POWER, not just SCREEN_OFF.
        s_last_countdown_us = now;
        recompute();
        break;

    case EVT_TICK:
        recompute();
        break;
    }
}

// ---------------------------------------------------------------------
// Subscriptions (called from event-bus context — just enqueue)
// ---------------------------------------------------------------------

static void on_timer_state(const sk_event_t *evt, void *user)
{
    (void)user;
    if (!s_q || !evt->payload_json) return;

    // Treat "running" as: state contains "resettable", "locked", or
    // "expired" (countdown active or just ended). Other states are quiet.
    const char *st = evt->payload_json;
    bool running = (strstr(st, "resettable") || strstr(st, "locked") ||
                    strstr(st, "expired")    || strstr(st, "api_triggering"));
    evt_t out = { .type = EVT_TIMER_STATE, .timer_running = running };
    xQueueSend(s_q, &out, 0);
}

static void on_face_changed(const sk_event_t *evt, void *user)
{
    (void)user;
    if (!s_q || !evt->payload_json) return;
    // Quick parse for "face":N
    int face = -1;
    const char *p = strstr(evt->payload_json, "\"face\"");
    if (p && (p = strchr(p, ':')) != NULL) {
        face = (int)strtol(p + 1, NULL, 10);
    }
    ESP_LOGD(TAG, "subscriber: face.changed payload=%s parsed=%d",
             evt->payload_json, face);
    evt_t out = { .type = EVT_FACE_CHANGED, .face = face };
    xQueueSend(s_q, &out, 0);
}

static void on_button(const sk_event_t *evt, void *user)
{
    (void)evt; (void)user;
    if (!s_q) return;
    evt_t out = { .type = EVT_BUTTON };
    xQueueSend(s_q, &out, 0);
}

// Wake the OLED when a SKAPP session opens (BLE GATT connect). The paired-
// device scanner only scans advertisements, so "connected" fires solely on a
// deliberate app connection. Match the quoted token so "disconnected" (which
// contains the substring "connected") doesn't also trip it.
static void on_ble_state(const sk_event_t *evt, void *user)
{
    (void)user;
    if (!s_q || !evt->payload_json) return;
    if (strstr(evt->payload_json, "\"connected\"")) {
        evt_t out = { .type = EVT_KICK };
        xQueueSend(s_q, &out, 0);
    }
}

static void on_tick_cb(void *arg)
{
    (void)arg;
    if (!s_q) return;
    evt_t out = { .type = EVT_TICK };
    xQueueSend(s_q, &out, 0);
}

// ---------------------------------------------------------------------
// Public API + CLI
// ---------------------------------------------------------------------

bf_power_state_t bf_power_state(void)        { return s_state; }
const char      *bf_power_state_str(void)    { return state_str(s_state); }

void bf_power_kick(void)
{
    if (!s_q) return;
    evt_t out = { .type = EVT_KICK };
    xQueueSend(s_q, &out, 0);
}

// ---------------------------------------------------------------------
// CLI: power.status / power.kick
// ---------------------------------------------------------------------

static sk_err_t cmd_power_status(sk_cli_ctx_t *ctx)
{
    int64_t now = esp_timer_get_time();
    int64_t since_ms      = (now - s_state_entered_us)  / 1000;
    int64_t idle_ms       = (now - s_last_activity_us)  / 1000;
    int64_t no_count_ms   = (now - s_last_countdown_us) / 1000;
    char buf[200];
    snprintf(buf, sizeof(buf),
             "{\"state\":\"%s\",\"since_ms\":%lld,\"idle_ms\":%lld,"
             "\"no_count_ms\":%lld,\"face_down\":%s,\"timer_running\":%s,"
             "\"screen_off_ms\":%d,\"low_power_ms\":%d}",
             state_str(s_state),
             (long long)since_ms, (long long)idle_ms, (long long)no_count_ms,
             s_face_down ? "true" : "false",
             s_timer_running ? "true" : "false",
             SCREEN_OFF_MS, LOW_POWER_MS);
    sk_cli_ok(ctx, buf);
    return SK_OK;
}

static sk_err_t cmd_power_kick(sk_cli_ctx_t *ctx)
{
    bf_power_kick();
    sk_cli_ok(ctx, "{\"kicked\":true}");
    return SK_OK;
}

static const sk_cli_command_t s_power_cmds[] = {
    { .name = "power.status",
      .summary = "Show power state, idle timers, and thresholds",
      .usage   = "power status",
      .help_block =
          "Reports current bf_power state (active_timer/active_idle/\n"
          "screen_off/low_power), how long since the last user activity\n"
          "and last countdown tick, and whether the cube is face-down.\n"
          "Useful for diagnosing wake-from-sleep issues.\n",
      .handler = cmd_power_status },

    { .name = "power.kick",
      .summary = "Force the device awake (equivalent to a button press)",
      .usage   = "power kick",
      .help_block =
          "Treats the call as user activity: resets idle and no-countdown\n"
          "timers and re-evaluates state. Used by SKAPP after a CLI\n"
          "session opens so the screen turns back on for the user.\n",
      .handler = cmd_power_kick },
};

// ---------------------------------------------------------------------
// Task loop
// ---------------------------------------------------------------------

static void power_task(void *arg)
{
    (void)arg;
    for (;;) {
        evt_t e;
        if (xQueueReceive(s_q, &e, portMAX_DELAY) != pdTRUE) continue;
        handle_event(&e);
    }
}

// ---------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------

esp_err_t bf_power_init(void)
{
    if (s_q) return ESP_OK;

    // PM lock created BEFORE the first apply() call so the initial
    // ACTIVE_IDLE → hold_no_sleep(true) actually engages. Without this
    // the very first state's lock acquire silently no-ops (lock=NULL),
    // light-sleep activates during boot, USB Serial/JTAG drops, and
    // the user can't `idf.py monitor` until the device idles back to
    // ACTIVE.
    esp_err_t lerr = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0,
                                        "bf_power_active",
                                        &s_no_sleep_lock);
    if (lerr != ESP_OK) {
        ESP_LOGW(TAG, "pm_lock_create failed: %s — light-sleep will "
                      "engage during ACTIVE states (USB monitor unstable)",
                 esp_err_to_name(lerr));
        s_no_sleep_lock = NULL;
    }

    s_q = xQueueCreate(16, sizeof(evt_t));
    if (!s_q) return ESP_ERR_NO_MEM;

    int64_t now = esp_timer_get_time();
    s_last_activity_us  = now;
    s_last_countdown_us = now;
    s_state_entered_us  = now;

    const esp_timer_create_args_t targs = {
        .callback = on_tick_cb, .name = "bf_power_tick",
    };
    if (esp_timer_create(&targs, &s_tick_t) != ESP_OK) {
        vQueueDelete(s_q); s_q = NULL;
        return ESP_FAIL;
    }
    esp_timer_start_periodic(s_tick_t, (uint64_t)CHECK_PERIOD_MS * 1000);

    // Loud failures: a silent subscribe error here pinned the device in
    // SCREEN_OFF (no wake on face change or button) the entire time the
    // event-bus subscriber table was full. Surface it now so it can't
    // happen again unnoticed.
    int sub;
    struct { const char *t; sk_event_handler_t h; } subs[] = {
        { "timer.state",     on_timer_state  },
        { "face.changed",    on_face_changed },
        { "button.released", on_button       },
        { "ble.state",       on_ble_state    },
    };
    for (size_t i = 0; i < sizeof(subs)/sizeof(subs[0]); i++) {
        esp_err_t serr = sk_event_bus_subscribe(subs[i].t, subs[i].h, NULL, &sub);
        if (serr != ESP_OK) {
            ESP_LOGE(TAG,
                "subscribe(%s) FAILED: %s — wake-from-sleep WILL break",
                subs[i].t, esp_err_to_name(serr));
        }
    }

    sk_capabilities_register_book("bf_power", "0.2.0");

    for (size_t i = 0; i < sizeof(s_power_cmds)/sizeof(s_power_cmds[0]); i++) {
        sk_cli_register(&s_power_cmds[i]);
    }

    // 4096 ran lean when peer-connect bursts triggered face/timer/button
    // event subscribers concurrently. 6144 absorbs the spike safely.
    BaseType_t r = xTaskCreate(power_task, "bf_power", 6144, NULL, 4, NULL);
    if (r != pdPASS) {
        esp_timer_stop(s_tick_t); esp_timer_delete(s_tick_t); s_tick_t = NULL;
        vQueueDelete(s_q); s_q = NULL;
        return ESP_FAIL;
    }

    apply(s_state);   // make sure backlight matches initial ACTIVE_IDLE
    publish_state();
    ESP_LOGI(TAG, "ready (screen_off=%d s, low_power=%d s)",
             SCREEN_OFF_MS / 1000, LOW_POWER_MS / 1000);
    return ESP_OK;
}
