// =====================================================================
// Blocking Focus — main.c
// =====================================================================
// SKAPP Faz 0 üzerinde Blocking Focus geri sayım küpü. Boot sırası:
//   1) NVS init
//   2) sk_core (identity, CLI, event bus, errors, capabilities)
//   3) USB CLI transport (Serial/JTAG)
//   4) sk_auth (pairing/handshake/HMAC/confirm tokens)
//   5) sk_button (BLE-on / restart / factory-reset gestures)
//   6) WiFi STA + mDNS + BLE GATT + TCP NDJSON
//   7) sk_ota (OTA enabled, manifest URL boş → runtime'da disabled)
//   8) sk_api (outbound HTTP — IFTTT/generic/webhook_post)
//   9) Cihaza özgü bf_* component init'leri (display, sensör, timer, ...)
//
// EDIT bloklarındaki sabitler dışında bu dosyada cihazlar arası ortak
// init mantığı yaşar. Cihaza özgü her şey "Device-specific code" işaretine
// kadar olan bölümün altına yazılır.
// =====================================================================

#include <string.h>

#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

#include "sk_core.h"   // umbrella — pulls every public sk_core API (incl. sk_ota)
#include "sk_api.h"    // outbound HTTP (IFTTT, generic, webhook_post)
#include "sk_log.h"    // structured event log baseline (boot reason etc.)

#include "pins.h"
#include "bf_vibration.h"
#include "bf_battery.h"
#include "bf_secure_store.h"
#include "bf_lis3dsh.h"
#include "bf_face_detector.h"
#include "bf_timer_engine.h"
#include "bf_display.h"
#include "bf_ui.h"
#include "bf_power.h"

// === EDIT [Identity] =================================================
//
// 2-char uppercase ASCII (A-Z) device type code. "BF" = Blocking Focus.
#define SK_DEVICE_TYPE_PREFIX   "BF"

// Single uppercase letter (A-Z) for hardware revision. 'A' = ilk PCB.
#define SK_HW_REV               'A'

// Human-readable product name shown in CLI banner / help header:
//   BF-A06TMFSQT - SmartKraft BlockingFocus v0.3.2 (...)
// Brand "SmartKraft" is fixed in sk_core; this is the per-project part.
#define SK_PRODUCT_NAME         "BlockingFocus"

// Firmware version — semver. Bump on every release.
// NOTE: keep in sync with esp32/BF/version.txt (embedded esp_app_desc version)
// and the published manifest.json `version` field on GitHub Releases.
#define SK_FW_VERSION           "0.3.2"

// Optional build tag (git sha, CI build number). NULL = none.
#define SK_BUILD_INFO           NULL
//
// === EDIT [Control button] ===========================================
//
// Tek donanım butonu sk_button gesture'ları sürer:
//   SHORT_PRESS  → BLE advertising aç (eşleşme penceresi)
//   LONG_PRESS   → device.restart (3 sn basılı tut)
//   MULTI_TAP    → factory reset (5× hızlı tap)
#define SK_BUTTON_GPIO          PIN_USER_BUTTON
#define SK_BUTTON_ACTIVE_LOW    1
//
// === EDIT [Wireless] =================================================
//
// TCP NDJSON port — also announced via mDNS as `_skapp._tcp`.
#define SK_TCP_PORT             8080
//
// === EDIT [Optional features] ========================================
#define SK_API_ENABLE           1   // Outbound HTTP (IFTTT, generic, webhook_post)
#define BF_POWER_ENABLE         1   // face-driven OLED sleep + idle ladder
//
// === EDIT [sk_ota] ===================================================
//
// Manifest-driven OTA. Boş URL → sk_ota_init no-op (runtime'da disabled).
// GitHub Releases: "latest" otomatik en yeni sürüme yönlenir → URL sabit kalır.
#define SK_OTA_ENABLE           1
#define SK_OTA_MANIFEST_URL     "https://github.com/smrtkrft/BlockingFocus/releases/latest/download/manifest.json"
// =====================================================================

// Compile-time guards — typos in EDIT block are caught now, not at first boot.
_Static_assert(sizeof(SK_DEVICE_TYPE_PREFIX) == 3,
               "SK_DEVICE_TYPE_PREFIX must be exactly 2 ASCII characters");
_Static_assert(SK_HW_REV >= 'A' && SK_HW_REV <= 'Z',
               "SK_HW_REV must be an uppercase letter 'A' through 'Z'");
_Static_assert(SK_BUTTON_GPIO >= 0,
               "SK_BUTTON_GPIO must be a valid GPIO number");
_Static_assert(SK_BUTTON_GPIO != PIN_BOOT_RESERVED,
               "SK_BUTTON_GPIO collides with the ESP32-C6 boot pin (GPIO9)");
_Static_assert(SK_TCP_PORT > 0 && SK_TCP_PORT < 65536,
               "SK_TCP_PORT must be a valid TCP port (1-65535)");

static const char *TAG = "main";

// CLI banner status line — fills the parenthesized portion of:
//   BF-A06TMFSQT - SmartKraft BlockingFocus v0.3.2 (wifi: connected, battery 87%, ...)
// Each part guarded so a missing/uninitialised module is silently skipped.
// Add timer state, cube face etc. as those bf_* components land.
static size_t bf_status_line(char *out, size_t cap)
{
    size_t off = 0;

    sk_wifi_status_t w;
    sk_wifi_status(&w);
    off += (size_t)snprintf(out + off, cap - off, "wifi: %s",
                            w.connected ? "connected" : "off");

    if (bf_battery_present() && off < cap) {
        off += (size_t)snprintf(out + off, cap - off,
                                ", battery %d%%", bf_battery_percent());
    }
    return off;
}

void app_main(void)
{
    // Suppress NimBLE host's per-notify INFO chatter
    // ("GATT procedure initiated: notify;" / "att_handle=18") — these
    // dump for every BLE notification and drown the monitor. WARN level
    // keeps real errors. sk_ble: tag still logs the meaningful events
    // (advertising, MTU, connect/disconnect).
    esp_log_level_set("NimBLE", ESP_LOG_WARN);
    // wifi: stack INFO satırları ("Coexist: Wi-Fi connect fail, apply
    // reconnect coex policy" vb.) bağlantı sürecinde her ~10 sn tekrar
    // ediyor. ERROR seviyesi gerçek sorunları yine yakalar.
    esp_log_level_set("wifi",   ESP_LOG_ERROR);
    // wifi_init: boot'ta tcp/udp mailbox boyutları, IRAM OP, vb. 10+
    // satır verbose dump. Üretimde gereksiz.
    esp_log_level_set("wifi_init", ESP_LOG_WARN);
    // BLE_INIT: controller commit hash, MAC, "BLE modem sleep enabled"
    // vb. 6+ satır boot verbose'u. WARN gerçek init hatalarını gösterir.
    esp_log_level_set("BLE_INIT", ESP_LOG_WARN);
    // pp / net80211: WiFi ROM driver verbose; modem-sleep kalıbı
    // sk_wifi'nin GOT_IP'sinde zaten log'lanıyor.
    esp_log_level_set("pp", ESP_LOG_WARN);
    esp_log_level_set("net80211", ESP_LOG_WARN);
    esp_log_level_set("phy", ESP_LOG_WARN);
    esp_log_level_set("phy_init", ESP_LOG_WARN);
    // pm: "Frequency switching config: ..." tek satır boot info — kabul
    // ama yeniden flash'ta gereksiz.
    esp_log_level_set("pm", ESP_LOG_WARN);
    esp_log_level_set("sleep_clock", ESP_LOG_WARN);
    esp_log_level_set("sleep_gpio", ESP_LOG_WARN);
    // i2c: legacy driver migration uyarısı + bf_lis3dsh çift-install
    // hatası temizlendikten sonra bu tag'tan herhangi bir log gelmiyor.
    // ERROR seviyesi gerçek bus hatalarını yakalar.
    esp_log_level_set("i2c", ESP_LOG_ERROR);

    // NVS bootstrap — almost every sk_* library uses NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // sk_core: identity, CLI, event bus, errors, capabilities.
    ESP_ERROR_CHECK(sk_core_init(&(sk_core_cfg_t){
        .device_type_prefix = SK_DEVICE_TYPE_PREFIX,
        .hw_rev             = SK_HW_REV,
        .fw_version         = SK_FW_VERSION,
        .build_info         = SK_BUILD_INFO,
    }));

    // Per-project banner identity: brand line and dynamic status.
    sk_core_set_product(SK_PRODUCT_NAME, NULL);  // version falls back to fw_version
    sk_core_set_status_provider(bf_status_line);

    // Boot reason event — sk_log queue is up after sk_core_init (which
    // calls sk_baseline_init → sk_log_init). POWERON/EXT/SW are clean
    // boots; PANIC/WDT/BROWNOUT mean the previous run died unexpectedly
    // and the user (or SKAPP) should see it in logs.get.
    {
        esp_reset_reason_t rr = esp_reset_reason();
        const char *rr_name = "UNKNOWN";
        bool       unclean  = false;
        switch (rr) {
            case ESP_RST_POWERON:    rr_name = "POWERON";    break;
            case ESP_RST_EXT:        rr_name = "EXT";        break;
            case ESP_RST_SW:         rr_name = "SW";         break;
            case ESP_RST_DEEPSLEEP:  rr_name = "DEEPSLEEP";  break;
            case ESP_RST_PANIC:      rr_name = "PANIC";      unclean = true; break;
            case ESP_RST_INT_WDT:    rr_name = "INT_WDT";    unclean = true; break;
            case ESP_RST_TASK_WDT:   rr_name = "TASK_WDT";   unclean = true; break;
            case ESP_RST_WDT:        rr_name = "WDT";        unclean = true; break;
            case ESP_RST_BROWNOUT:   rr_name = "BROWNOUT";   unclean = true; break;
            case ESP_RST_SDIO:       rr_name = "SDIO";       break;
            // ESP-IDF v5.x ek reset türleri (eski IDF'lerde tanımlı değil,
            // bu yüzden #ifdef ile koruyoruz). USB ve JTAG, flash sonrası
            // CDC reset ve OpenOCD restart için yaygın; CPU_LOCKUP fault
            // hat'i tetiklenirse gelir; PWR_GLITCH brownout'tan farklı bir
            // güç bozulması; EFUSE yapılandırma hatası.
#ifdef ESP_RST_USB
            case ESP_RST_USB:        rr_name = "USB";        break;
#endif
#ifdef ESP_RST_JTAG
            case ESP_RST_JTAG:       rr_name = "JTAG";       break;
#endif
#ifdef ESP_RST_EFUSE
            case ESP_RST_EFUSE:      rr_name = "EFUSE";      unclean = true; break;
#endif
#ifdef ESP_RST_PWR_GLITCH
            case ESP_RST_PWR_GLITCH: rr_name = "PWR_GLITCH"; unclean = true; break;
#endif
#ifdef ESP_RST_CPU_LOCKUP
            case ESP_RST_CPU_LOCKUP: rr_name = "CPU_LOCKUP"; unclean = true; break;
#endif
            default: break;
        }
        if (unclean) {
            SK_LOG_W("boot", "unclean", "reset=%s fw=%s", rr_name, SK_FW_VERSION);
        } else {
            SK_LOG_I("boot", "up", "reset=%s fw=%s", rr_name, SK_FW_VERSION);
        }
    }

    // Topic catalog — registered ONCE here, in the order categories should
    // appear in `help`. Three buckets:
    //   SYSTEM  — device-level baseline shared across all SmartKraft cubes
    //             (network, firmware, telemetry, diagnostics)
    //   SKAPP   — paired-phone management (pairing window, passphrase, bonds)
    //   OUTPUT  — how this cube affects the world (webhooks, vibration)
    // Order inside each bucket follows the order calls below.
    sk_cli_register_topic("wifi",      "Network connection",                                 "SYSTEM");
    sk_cli_register_topic("ble",       "Bluetooth transport",                                "SYSTEM");
    sk_cli_register_topic("ota",       "Firmware updates (check / install / rollback)",      "SYSTEM");
    sk_cli_register_topic("device",    "Identity, restart, factory reset",                   "SYSTEM");
    sk_cli_register_topic("battery",   "Battery voltage and threshold events",               "SYSTEM");
    sk_cli_register_topic("logs",      "Log entries (ring buffer)",                          "SYSTEM");

    sk_cli_register_topic("pairing",   "SKAPP pairing window (open / status / close)",       "SKAPP");
    sk_cli_register_topic("auth",      "SKAPP connection passphrase (set / change / mode)",  "SKAPP");
    sk_cli_register_topic("bond",      "Paired SKAPP installs (list / remove)",              "SKAPP");

    sk_cli_register_topic("api",       "Outbound webhook presets",                           "OUTPUT");
    sk_cli_register_topic("vibration", "Vibration motor master switch",                      "OUTPUT");

    ESP_ERROR_CHECK(sk_transport_usb_init(NULL));

    // === Power management ============================================
    // PM/light-sleep aktif edilmeden önce USB Serial/JTAG enumerate
    // olmalı: aksi halde host PC cihazı göremez ve `idf.py flash`
    // sonrası manuel reset basmadan monitor çalışmaz. light_sleep
    // CPU/APB clock'u idle tick'lerde gate ediyor; USB controller bu
    // sırada host descriptor probe'una cevap veremiyor ve enumerasyon
    // yarışı kaybediyor. Sırayı `sk_transport_usb_init` SONRASINA
    // çekmek bu yarışı ortadan kaldırır; steady-state DFS + light
    // sleep tasarrufu korunur. sk_wifi'nin `WIFI_PS_MAX_MODEM` çağrısı
    // yine bu PM aktivasyonu üzerine binecek (sk_wifi_init ileride
    // çağrılıyor).
    //
    // min/max CPU: 40 MHz (XTAL) idle, 160 MHz (PLL) under load. On
    // ESP32-C6 the APB bus runs from a separate clock tree fixed at
    // 80 MHz regardless of CPU DFS state, so legacy driver/i2c.h
    // transactions are NOT affected by min↔max transitions.
    // min_freq_mhz must be ≥ XTAL frequency (40 MHz on C6).
    // light_sleep_enable=true → FreeRTOS tickless idle otomatik girer.
    esp_pm_config_t pm_cfg = {
        .max_freq_mhz       = 160,
        .min_freq_mhz       = 40,
        .light_sleep_enable = true,
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_cfg));

    // Auth subscriptions registered before button so a press during boot
    // is captured rather than dropped.
    ESP_ERROR_CHECK(sk_auth_init());

    // Optional content-access passphrase. Init right after sk_auth so its
    // factory-reset subscriber is wired before bf_secure_store's. NVS-backed,
    // no hardware deps; safe to fail-open if NVS is unhealthy (init logs a
    // warning, runtime falls back to "no passphrase configured").
    ESP_ERROR_CHECK(sk_passphrase_init());

    // Encrypted KV + 100 KB user-script blob. Registered after sk_auth so
    // its `device.factory-reset.requested` subscriber fires alongside
    // sk_auth's, and before sk_button so a multi-tap mid-boot still hits
    // a fully-armed wipe path. SPIFFS mount + master-key generation happen
    // here on first boot.
    ESP_ERROR_CHECK(bf_secure_store_init());

    ESP_ERROR_CHECK(sk_button_init(&(sk_button_cfg_t){
        .gpio_num        = SK_BUTTON_GPIO,
        .active_low      = SK_BUTTON_ACTIVE_LOW,
        .pullup_internal = true,
    }, NULL, NULL));

    // GPIO wake source — registered AFTER sk_button_init so the pin
    // direction/pull-up configured by sk_button is already in place.
    // GPIO_INTR_LOW_LEVEL matches active-low button. Light-sleep on
    // ESP32-C6's regular GPIOs only supports level-triggered wake
    // (edge wake is RTC-IO / deep-sleep only). The hold here is fine
    // because sk_button's own ISR triggers on the same edge — they
    // coexist on independent peripheral paths.
    //
    // LIS3DSH INT1 GPIO is intentionally NOT enabled as a wake source:
    // bf_lis3dsh doesn't program the sensor's INT1 register yet, so
    // the line floats. Polling at 1 Hz in LOW_POWER (bf_face_detector)
    // yields ≤1 s wake latency — enough for "user just picked up the
    // cube" UX. v0.3 will add sensor-side motion-threshold IRQ.
    ESP_ERROR_CHECK(gpio_wakeup_enable(SK_BUTTON_GPIO, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());

    // Wireless stack.
    ESP_ERROR_CHECK(sk_wifi_init());
    ESP_ERROR_CHECK(sk_mdns_init(SK_TCP_PORT, SK_FW_VERSION));
    ESP_ERROR_CHECK(sk_transport_ble_init(NULL));
    ESP_ERROR_CHECK(sk_transport_tcp_init(&(sk_transport_tcp_cfg_t){
        .port = SK_TCP_PORT,
    }));

#if SK_OTA_ENABLE
    ESP_ERROR_CHECK(sk_ota_init(&(sk_ota_cfg_t){
        .fw_version   = SK_FW_VERSION,
        .manifest_url = SK_OTA_MANIFEST_URL[0] ? SK_OTA_MANIFEST_URL : NULL,
    }));
#endif

#if SK_API_ENABLE
    ESP_ERROR_CHECK(sk_api_init());
#endif

    ESP_LOGI(TAG, "Faz 0 up — id=%s", sk_identity_get());

    // ------------------------------------------------------------------
    // Device-specific (Blocking Focus) — bf_* component init sırası:
    //   1) bf_vibration   → direct GPIO (D7) → MOSFET → motor
    //   2) bf_battery     → ADC + threshold events
    //   3) bf_display     → SSD1306 I2C OLED — installs the I2C bus driver
    //   4) bf_lis3dsh     → I2C accel (shares bus with display)
    //   5) bf_face_detector → 1 sn debounce, face.changed event'i
    //   6) bf_timer_engine  → state machine (60sn lock, arming, API trigger)
    //   7) bf_power       → idle ladder, sleep coordinator
    //
    // Glue: her bf_* component'i sk_event_bus'a publish/subscribe olur,
    // doğrudan birbirini çağırmaz (bf_power → bf_display sleep çağrısı hariç).
    // 2026-05-08: bf_buzzer kaldırıldı; buzzer ile titreşim ayrı sistem.
    // ------------------------------------------------------------------

    ESP_ERROR_CHECK(bf_vibration_init());
    ESP_ERROR_CHECK(bf_battery_init());
    ESP_ERROR_CHECK(bf_display_init());          // first I2C user → installs bus
    ESP_ERROR_CHECK(bf_lis3dsh_init());          // probe NOW, before bf_ui's
                                                 // ui_task starts hammering the
                                                 // bus with framebuffer pushes
                                                 // (otherwise probe TIMEOUTs and
                                                 // push gets cut off mid-page)
    ESP_ERROR_CHECK(bf_face_detector_init());
    ESP_ERROR_CHECK(bf_ui_init());               // splash/brand/idle scenes on top
    ESP_ERROR_CHECK(bf_timer_engine_init());
#if BF_POWER_ENABLE
    ESP_ERROR_CHECK(bf_power_init());            // last: subscribes to all the others
#endif
}
