#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =====================================================================
// bf_power — power-state coordinator (v0.2).
//
// Owns escalation between four power levels driven by activity. Listens
// to timer.state, face.changed, and button events to decide when to
// turn off the OLED and what poll-period subscribers should use.
//
//   ACTIVE_TIMER   timer counting down               OLED on, full polling
//   ACTIVE_IDLE    no timer, recently active         OLED on, full polling
//   SCREEN_OFF     no input for SCREEN_OFF_MS        OLED off (0xAE)
//   LOW_POWER      no countdown for LOW_POWER_MS    OLED off, slow polling
//
// What actually delivers the low-power numbers:
//   - main.c calls esp_pm_configure(light_sleep_enable=true) at boot.
//     CONFIG_PM_ENABLE + CONFIG_FREERTOS_USE_TICKLESS_IDLE in
//     sdkconfig.defaults make FreeRTOS auto-enter esp_light_sleep_start
//     during idle, with no manual sleep calls anywhere.
//   - sk_wifi sets WIFI_PS_MAX_MODEM after GOT_IP. With PM enabled this
//     becomes real DTIM-aligned modem-sleep (~3-7 mA average).
//   - bf_face_detector subscribes to power.state and drops its I2C
//     poll period from 100 ms (10 Hz) → 250 ms in SCREEN_OFF
//     → 1000 ms in LOW_POWER. The longer vTaskDelay is what gives
//     the PM layer big idle windows to actually light-sleep through.
//
// bf_power's own job is therefore narrow: pick the state, drive the
// OLED 0xAE/0xAF cmd, publish power.state. Everything else is a
// subscriber decision.
//
// Wake-on-button: main.c calls gpio_wakeup_enable(BUTTON, LOW_LEVEL) +
// esp_sleep_enable_gpio_wakeup() so the user gets sub-50-ms response
// even from deep light-sleep. Cube rotation wakes via the 1 Hz poll
// loop (worst case 1 sec latency, then face.changed kicks bf_power
// back to ACTIVE_IDLE and polling jumps back to 10 Hz). Sensor-side
// LIS3DSH INT1 motion-wake is reserved for v0.3.
//
// Events on sk_event_bus:
//   power.state    {"state":"active_timer|active_idle|screen_off|low_power",
//                   "since_ms":N}
// =====================================================================

typedef enum {
    BF_POWER_ACTIVE_TIMER = 0,
    BF_POWER_ACTIVE_IDLE,
    BF_POWER_SCREEN_OFF,
    BF_POWER_LOW_POWER,
} bf_power_state_t;

esp_err_t          bf_power_init(void);
bf_power_state_t   bf_power_state(void);
const char        *bf_power_state_str(void);

// Re-arm the activity timer (back to ACTIVE_IDLE if currently dimmed).
// Useful for hooks that want to keep the screen alive on a particular
// event without going through the event bus.
void               bf_power_kick(void);

#ifdef __cplusplus
}
#endif
