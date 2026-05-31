#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =====================================================================
// bf_battery — single-cell LiPo voltage monitor.
//
// Reads PIN_BATTERY_ADC through an external resistor divider (ratio set
// by BATTERY_DIVIDER_RATIO_X100 in pins.h). A background task samples
// every 10 s with 8x oversampling, exposes the latest reading, and
// publishes events when the voltage crosses configured thresholds.
//
// Graceful degradation: if ADC init fails the component logs a warning
// and runs in "no reading" mode — every public getter returns 0 and no
// events are emitted. This keeps boot succeeding on a half-assembled
// breadboard where the divider isn't soldered yet.
//
// Events on sk_event_bus:
//   battery.sample        every successful read    {"mv": 3850, "pct": 67}
//   battery.low           crossed BF_BATT_LOW_MV   {"mv": 3290, "pct": 12}
//   battery.critical      crossed BF_BATT_CRIT_MV  {"mv": 3050, "pct": 4}
//   battery.recovered     back above BF_BATT_OK_MV {"mv": 3520, "pct": 32}
//   battery.charge_state  USB / dV-dt change       {"state":"charging"}
//   battery.lockout       voltage <= LOCKOUT_MV    {"active":true}
//
// CLI: `battery.status`
//
// Accuracy strategy (no extra hardware — same divider, same ADC pin):
//   1. 64x median oversampling instead of mean      (spike rejection)
//   2. Skip samples while buzzer or vibration drive (avoid load sag)
//   3. Piecewise-linear LiPo discharge LUT          (non-linear curve)
//   4. EWMA + hysteresis on the displayed percent   (no jitter)
//   5. Freeze SoC while VBUS=1, drive separate      (CV-phase voltage lies)
//      charge_state from VBUS + dV/dt instead.
// =====================================================================

// LiPo single-cell thresholds. Tunable per device family.
//
// Voltage anchors are aligned with the displayed-SoC curve in bf_battery.c
// so "battery.low" fires at ~15 % displayed, not at a number that drifts
// when the LUT is reshaped. If you change the LUT anchors, recompute these.
#define BF_BATT_FULL_MV        4200   // 100 %
#define BF_BATT_EMPTY_MV       3000   //   0 %  (cutoff to protect cell)
#define BF_BATT_LOW_MV         3400   //  15 %  → battery.low
#define BF_BATT_CRIT_MV        3133   //   5 %  → battery.critical
#define BF_BATT_OK_MV          3550   //  25 %  → battery.recovered (debounce)
#define BF_BATT_PRESENT_MIN_MV 2500   // below this we treat the cell as absent

// Lockout: full-screen "LOW BATTERY" overlay when voltage falls below
// this. Sticky — only released when VBUS goes HIGH (charger plugged in).
// 3100 mV ≈ 3.75 % on the displayed curve, ~30-60 minutes before the cell
// hits 3.0 V cutoff under typical light load.
#define BF_BATT_LOCKOUT_MV     3100

// Charge state inferred from VBUS sense + dV/dt over the last ~minute.
// UI uses this to pick the icon (normal fill / + plus / full check).
typedef enum {
    BF_BATT_CHARGE_DISCHARGING = 0,  // VBUS=0
    BF_BATT_CHARGE_CHARGING,         // VBUS=1, voltage rising or below 4.15 V
    BF_BATT_CHARGE_FULL,             // VBUS=1, voltage stable >=4.15 V
} bf_batt_charge_state_t;

esp_err_t bf_battery_init(void);

// Latest measured voltage at the cell (post-divider scaled), in millivolts.
// Returns 0 before the first sample completes, or if ADC unavailable.
uint16_t  bf_battery_voltage_mv(void);

// State of charge 0..100, derived from a piecewise-linear LiPo discharge
// curve, EWMA-smoothed and hysteretic. Frozen while VBUS=1 (the CV-phase
// voltage cannot be mapped to SoC). Returns 0 if no battery present.
int       bf_battery_percent(void);

// True when the latest reading is plausible (>= BF_BATT_PRESENT_MIN_MV).
bool      bf_battery_present(void);

// Charge state (see enum). UI selects icon variant from this.
bf_batt_charge_state_t bf_battery_charge_state(void);

// True when the lockout latch is engaged: voltage fell below
// BF_BATT_LOCKOUT_MV and the cell has not seen USB power since. Cleared
// only when VBUS transitions LOW → HIGH (user plugs in the charger).
// bf_ui watches `battery.lockout` events and reads this on resume.
bool bf_battery_lockout_active(void);

#ifdef __cplusplus
}
#endif
