#include "bf_vibration.h"

#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"

#include "pins.h"
#include "sk_cli.h"
#include "sk_errors.h"
#include "sk_event_bus.h"

static const char *TAG = "bf_vibration";

#define NVS_NS              "bf_vibration"
#define NVS_KEY_VIBE        "vibe_en"
// Warning gate flags. NVS keys < 16 char. Tilt gate'in NVS anahtarı
// tarihsel sebeplerle `face_warn` adıyla saklanıyor (eski rename öncesi
// kullanıcı toggle değeri kaybolmasın diye).
#define NVS_KEY_TILT_WARN   "face_warn"
#define NVS_KEY_LOWB_WARN   "lowb_warn"

typedef struct {
    uint8_t  count;       // pulse adedi (1..N)
    uint16_t pulse_ms;    // her pulse uzunluğu
    uint16_t gap_ms;      // pulses arası boşluk
} pulse_pattern_t;

static QueueHandle_t s_queue;
static bool          s_vibration_enabled    = true;
// Warning gates default ON — kullanıcı kapatmadıkça uyarı verilir.
// `face.tilted` event'inde (cihaz yamuk duruyor, ambiguous orientation
// 5 sn) titreşim. Aynı flag bf_ui'daki "Cube'u düz koy" overlay'ini
// de gate ediyor (read via bf_tilt_warn_is_enabled). `battery.low`
// event'inde uzun titreşim. Toggle off → subscriber sessiz + overlay
// gizli.
static bool          s_tilt_warn_enabled     = true;
static bool          s_low_batt_warn_enabled = true;

// ---------------------------------------------------------------------
// NVS persistence
// ---------------------------------------------------------------------

static void load_settings(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    uint8_t v;
    if (nvs_get_u8(h, NVS_KEY_VIBE, &v) == ESP_OK) s_vibration_enabled    = (v != 0);
    if (nvs_get_u8(h, NVS_KEY_TILT_WARN, &v) == ESP_OK) s_tilt_warn_enabled     = (v != 0);
    if (nvs_get_u8(h, NVS_KEY_LOWB_WARN, &v) == ESP_OK) s_low_batt_warn_enabled = (v != 0);
    nvs_close(h);
}

static esp_err_t save_one(const char *key, bool on)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_set_u8(h, key, on ? 1 : 0);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

// ---------------------------------------------------------------------
// GPIO drive
// ---------------------------------------------------------------------

// Vibration pin'i N-MOSFET gate'ine doğrudan sürer. Active-high
// (gate HIGH = MOSFET ON = motor enerjili). Master switch OFF → pin
// HIGH'a alınmaz.
static void vibration_drive(bool on)
{
    gpio_set_level(PIN_VIBRATION, (on && s_vibration_enabled) ? 1 : 0);
}

static void worker_task(void *arg)
{
    (void)arg;
    pulse_pattern_t pat;
    for (;;) {
        if (xQueueReceive(s_queue, &pat, portMAX_DELAY) != pdTRUE) continue;
        if (pat.count > 1 || pat.pulse_ms >= 500) {
            // Burst veya uzun pulse: master + actual pin yolunu logla.
            ESP_LOGW(TAG, "pulse: count=%u on=%u off=%u master=%s",
                     pat.count, pat.pulse_ms, pat.gap_ms,
                     s_vibration_enabled ? "ON" : "OFF");
        }
        for (uint8_t i = 0; i < pat.count; i++) {
            vibration_drive(true);
            vTaskDelay(pdMS_TO_TICKS(pat.pulse_ms));
            vibration_drive(false);
            if (i + 1 < pat.count) vTaskDelay(pdMS_TO_TICKS(pat.gap_ms));
        }
    }
}

// ---------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------

bool bf_vibration_is_enabled(void) { return s_vibration_enabled; }

esp_err_t bf_vibration_set_enabled(bool on)
{
    s_vibration_enabled = on;
    // Devam eden bir pulse varsa, master OFF anında pin'i hemen düşür.
    if (!on) gpio_set_level(PIN_VIBRATION, 0);
    return save_one(NVS_KEY_VIBE, on);
}

bool bf_tilt_warn_is_enabled(void)     { return s_tilt_warn_enabled; }
bool bf_low_batt_warn_is_enabled(void) { return s_low_batt_warn_enabled; }

esp_err_t bf_tilt_warn_set_enabled(bool on)
{
    s_tilt_warn_enabled = on;
    return save_one(NVS_KEY_TILT_WARN, on);
}

esp_err_t bf_low_batt_warn_set_enabled(bool on)
{
    s_low_batt_warn_enabled = on;
    return save_one(NVS_KEY_LOWB_WARN, on);
}

static void enqueue(pulse_pattern_t pat)
{
    if (!s_queue) return;   // init çağrılmamış — sessiz drop
    xQueueSend(s_queue, &pat, 0);
}

void bf_vibration_pulse_ms(uint16_t ms)
{
    if (ms == 0) return;
    if (ms > 10000) ms = 10000;   // safety clamp
    enqueue((pulse_pattern_t){
        .count = 1,
        .pulse_ms = ms,
        .gap_ms = 0,
    });
}

void bf_vibration_burst(uint8_t count, uint16_t on_ms, uint16_t off_ms)
{
    if (count == 0 || on_ms == 0) return;
    if (count > 15) count = 15;
    if (on_ms > 5000)  on_ms = 5000;
    if (off_ms > 5000) off_ms = 5000;
    enqueue((pulse_pattern_t){
        .count    = count,
        .pulse_ms = on_ms,
        .gap_ms   = off_ms,
    });
}

// ---------------------------------------------------------------------
// CLI: vibration.on / vibration.off / vibration.status / vibration.test
//      tilt.warn.{on,off,status} / low_batt.warn.{on,off,status}
//      gpio.test <pin>  (donanım pad doğrulama yardımcısı)
// ---------------------------------------------------------------------

static sk_err_t cmd_vibration_on(sk_cli_ctx_t *ctx)
{
    esp_err_t err = bf_vibration_set_enabled(true);
    if (err != ESP_OK) {
        sk_cli_err(ctx, SK_ERR_INTERNAL, "{\"reason\":\"nvs_save\"}");
        return SK_OK;
    }
    sk_event_bus_publish("vibration.enabled", "{\"enabled\":true}");
    sk_cli_ok(ctx, "{\"enabled\":true}");
    return SK_OK;
}

static sk_err_t cmd_vibration_off(sk_cli_ctx_t *ctx)
{
    esp_err_t err = bf_vibration_set_enabled(false);
    if (err != ESP_OK) {
        sk_cli_err(ctx, SK_ERR_INTERNAL, "{\"reason\":\"nvs_save\"}");
        return SK_OK;
    }
    sk_event_bus_publish("vibration.enabled", "{\"enabled\":false}");
    sk_cli_ok(ctx, "{\"enabled\":false}");
    return SK_OK;
}

static sk_err_t cmd_vibration_status(sk_cli_ctx_t *ctx)
{
    char buf[40];
    snprintf(buf, sizeof(buf), "{\"enabled\":%s}",
             s_vibration_enabled ? "true" : "false");
    sk_cli_ok(ctx, buf);
    return SK_OK;
}

// vibration.burst — geri sayım expire pattern'ini doğrudan tetikler
// (4 × 500 ms ON / 250 ms OFF). Master switch ON gerekli; OFF iken
// queue çalışır ama pin sürülmez (uyarı log'lanır). Donanım/yazılım
// teşhisi: timer expire ile aynı kod yolunu manuel çalıştırır.
static sk_err_t cmd_vibration_burst(sk_cli_ctx_t *ctx)
{
    ESP_LOGW(TAG, "vibration.burst: master=%s, firing 4×500ms ON / 250ms OFF",
             s_vibration_enabled ? "ON" : "OFF");
    bf_vibration_burst(4, 500, 250);
    char buf[80];
    snprintf(buf, sizeof(buf),
             "{\"count\":4,\"on_ms\":500,\"off_ms\":250,\"master\":%s}",
             s_vibration_enabled ? "true" : "false");
    sk_cli_ok(ctx, buf);
    return SK_OK;
}

// Warning-gate setters — sadece flag flip + persist; subscriber'lar
// event üzerinden çalışır, doğrudan GPIO yok.
static sk_err_t set_warn(bool is_tilt, bool on, sk_cli_ctx_t *ctx)
{
    esp_err_t err = is_tilt ? bf_tilt_warn_set_enabled(on)
                            : bf_low_batt_warn_set_enabled(on);
    if (err != ESP_OK) {
        sk_cli_err(ctx, SK_ERR_INTERNAL, "{\"reason\":\"nvs_save\"}");
        return SK_OK;
    }
    sk_event_bus_publish(is_tilt ? "tilt.warn.changed"
                                 : "low_batt.warn.changed",
                         on ? "{\"enabled\":true}" : "{\"enabled\":false}");
    char buf[40];
    snprintf(buf, sizeof(buf), "{\"enabled\":%s}", on ? "true" : "false");
    sk_cli_ok(ctx, buf);
    return SK_OK;
}

static sk_err_t status_warn(bool is_tilt, sk_cli_ctx_t *ctx)
{
    bool now = is_tilt ? s_tilt_warn_enabled : s_low_batt_warn_enabled;
    char buf[40];
    snprintf(buf, sizeof(buf), "{\"enabled\":%s}", now ? "true" : "false");
    sk_cli_ok(ctx, buf);
    return SK_OK;
}

static sk_err_t cmd_tilt_warn_on    (sk_cli_ctx_t *ctx) { return set_warn(true,  true,  ctx); }
static sk_err_t cmd_tilt_warn_off   (sk_cli_ctx_t *ctx) { return set_warn(true,  false, ctx); }
static sk_err_t cmd_tilt_warn_status(sk_cli_ctx_t *ctx) { return status_warn(true,  ctx); }
static sk_err_t cmd_lowb_warn_on    (sk_cli_ctx_t *ctx) { return set_warn(false, true,  ctx); }
static sk_err_t cmd_lowb_warn_off   (sk_cli_ctx_t *ctx) { return set_warn(false, false, ctx); }
static sk_err_t cmd_lowb_warn_status(sk_cli_ctx_t *ctx) { return status_warn(false, ctx); }

static const sk_cli_command_t s_cmd_vibration_on = {
    .name    = "vibration.on",
    .summary = "Turn vibration motor on (master switch)",
    .usage   = "vibration on",
    .handler = cmd_vibration_on,
};
static const sk_cli_command_t s_cmd_vibration_off = {
    .name    = "vibration.off",
    .summary = "Turn vibration motor off (master switch)",
    .usage   = "vibration off",
    .handler = cmd_vibration_off,
};
static const sk_cli_command_t s_cmd_vibration_status = {
    .name    = "vibration.status",
    .summary = "Show vibration master switch state",
    .usage   = "vibration status",
    .handler = cmd_vibration_status,
};
static const sk_cli_command_t s_cmd_vibration_burst = {
    .name    = "vibration.burst",
    .summary = "Trigger expire-pattern burst (4 × 500/250 ms) for diagnosis",
    .usage   = "vibration burst",
    .handler = cmd_vibration_burst,
};
static const sk_cli_command_t s_cmd_tilt_warn_on = {
    .name    = "tilt.warn.on",
    .summary = "Enable alert when the cube is tilted (no clear top face)",
    .usage   = "tilt warn on",
    .handler = cmd_tilt_warn_on,
};
static const sk_cli_command_t s_cmd_tilt_warn_off = {
    .name    = "tilt.warn.off",
    .summary = "Disable tilt alert",
    .usage   = "tilt warn off",
    .handler = cmd_tilt_warn_off,
};
static const sk_cli_command_t s_cmd_tilt_warn_status = {
    .name    = "tilt.warn.status",
    .summary = "Show tilt alert state",
    .usage   = "tilt warn status",
    .handler = cmd_tilt_warn_status,
};
static const sk_cli_command_t s_cmd_lowb_warn_on = {
    .name    = "low_batt.warn.on",
    .summary = "Enable alert when battery falls below threshold",
    .usage   = "low_batt warn on",
    .handler = cmd_lowb_warn_on,
};
static const sk_cli_command_t s_cmd_lowb_warn_off = {
    .name    = "low_batt.warn.off",
    .summary = "Disable low-battery alert",
    .usage   = "low_batt warn off",
    .handler = cmd_lowb_warn_off,
};
static const sk_cli_command_t s_cmd_lowb_warn_status = {
    .name    = "low_batt.warn.status",
    .summary = "Show low-battery alert state",
    .usage   = "low_batt warn status",
    .handler = cmd_lowb_warn_status,
};

// ---------------------------------------------------------------------
// Event subscribers — gated alerts
// ---------------------------------------------------------------------

// Tilt alert: bf_face_detector publishes `face.tilted` with `tilted:true`
// when no axis dominates for ≥5 s and `tilted:false` when a dominant
// axis returns. Sadece OFF→ON transition fire eder. Gated by
// tilt.warn.on. bf_ui overlay aynı flag'i paylaşıyor.
static void on_face_tilted(const sk_event_t *evt, void *user)
{
    (void)user;
    if (!s_tilt_warn_enabled) return;
    if (!evt || !evt->payload_json) return;
    if (strstr(evt->payload_json, "\"tilted\":true") == NULL) return;
    enqueue((pulse_pattern_t){ .count = 2, .pulse_ms = 100, .gap_ms = 120 });
}

// Low-battery alert: bf_battery publishes "battery.low" once on the
// OK→LOW transition. Pattern: sustained vibration. Gated by low_batt.warn.on.
static void on_battery_low(const sk_event_t *evt, void *user)
{
    (void)evt; (void)user;
    if (!s_low_batt_warn_enabled) return;
    enqueue((pulse_pattern_t){ .count = 2, .pulse_ms = 250, .gap_ms = 150 });
}

// ---------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------

esp_err_t bf_vibration_init(void)
{
    if (s_queue) return ESP_OK;   // idempotent

    load_settings();

    // GPIO setup — push-pull output, başlangıçta LOW.
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_VIBRATION),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config: %s", esp_err_to_name(err));
        return err;
    }
    // Maks GPIO drive strength (~40 mA) — kullanıcı motoru doğrudan
    // GPIO ile sürdüğü prototip donanımda akım yetersizliğine karşı
    // tampon. CLAUDE.md tasarımı (MOSFET gate) için bu fark yaratmaz.
    gpio_set_drive_capability(PIN_VIBRATION, GPIO_DRIVE_CAP_3);
    vibration_drive(false);   // explicit LOW

    s_queue = xQueueCreate(8, sizeof(pulse_pattern_t));
    if (!s_queue) return ESP_ERR_NO_MEM;

    BaseType_t r = xTaskCreate(worker_task, "bf_vibration", 4096, NULL, 5, NULL);
    if (r != pdPASS) {
        vQueueDelete(s_queue);
        s_queue = NULL;
        ESP_LOGE(TAG, "worker task spawn failed");
        return ESP_FAIL;
    }

    sk_cli_register(&s_cmd_vibration_on);
    sk_cli_register(&s_cmd_vibration_off);
    sk_cli_register(&s_cmd_vibration_status);
    sk_cli_register(&s_cmd_vibration_burst);
    sk_cli_register(&s_cmd_tilt_warn_on);
    sk_cli_register(&s_cmd_tilt_warn_off);
    sk_cli_register(&s_cmd_tilt_warn_status);
    sk_cli_register(&s_cmd_lowb_warn_on);
    sk_cli_register(&s_cmd_lowb_warn_off);
    sk_cli_register(&s_cmd_lowb_warn_status);

    int sub;
    sk_event_bus_subscribe("face.tilted", on_face_tilted,  NULL, &sub);
    sk_event_bus_subscribe("battery.low", on_battery_low,  NULL, &sub);

    ESP_LOGI(TAG, "ready (vibration=GPIO%d %s, tilt_warn=%s, low_batt_warn=%s)",
             PIN_VIBRATION, s_vibration_enabled    ? "ON" : "OFF",
             s_tilt_warn_enabled     ? "ON" : "OFF",
             s_low_batt_warn_enabled ? "ON" : "OFF");
    return ESP_OK;
}
