#include "bf_battery.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pins.h"
#include "sk_baseline.h"
#include "sk_capabilities.h"
#include "sk_cli.h"
#include "sk_errors.h"
#include "sk_event_bus.h"

static const char *TAG = "bf_battery";

#define SAMPLE_PERIOD_MS    10000   // 10 s — battery moves slowly
#define OVERSAMPLE_COUNT    64      // 64x median rejects ADC spikes / RF kicks

// dV/dt window for charge detection: 6 samples * 10 s = 60 s.
#define DVDT_WINDOW_N       6
#define DVDT_RISING_UV_S    8       // >8 µV/s over the window → rising
                                    // (≈ 0.5 mV/min, well above ADC noise)

// Charge-full criteria: VBUS=1, V>=this, stable for STABLE_S seconds.
#define CHARGE_FULL_MV      4150
#define CHARGE_FULL_STABLE_S 30

// EWMA + hysteresis on the displayed percent. EWMA constant is x/256 to
// keep math integer; 32/256 ≈ 0.125 → ~80 s time constant at 10 s/sample.
#define PCT_EWMA_NUM        32
#define PCT_EWMA_DEN        256
#define PCT_HYSTERESIS      2       // displayed % only updates on >=2 step

// ESP32-C6: GPIO0..6 = ADC1_CHANNEL_0..6.
#define BATT_ADC_UNIT       ADC_UNIT_1
#define BATT_ADC_CHANNEL    ((adc_channel_t)PIN_BATTERY_ADC)
// 12 dB attenuation maps the input range to roughly 0..3.3 V.
#define BATT_ADC_ATTEN      ADC_ATTEN_DB_12

static adc_oneshot_unit_handle_t s_adc          = NULL;
static adc_cali_handle_t         s_cali         = NULL;
static bool                      s_adc_ok       = false;
static uint16_t                  s_voltage_mv   = 0;       // last raw OCV reading
static bool                      s_present      = false;

// Displayed (filtered + hysteretic) SoC. Internal EWMA accumulator carries
// 8 fractional bits so a slow EWMA doesn't quantize away to zero.
static int                       s_pct_ewma_x256 = -1;     // -1 = not seeded
static int                       s_pct_displayed = 0;

// Frozen SoC snapshot — last value computed while VBUS=0, held while
// USB is plugged in (CV-phase voltage cannot be mapped to SoC).
static int                       s_pct_frozen   = -1;

// dV/dt ring buffer of recent OCV samples, oldest-first.
static uint16_t                  s_dvdt_ring[DVDT_WINDOW_N];
static uint8_t                   s_dvdt_count   = 0;
static uint8_t                   s_dvdt_head    = 0;       // next write index

// Charge state + "stable above CHARGE_FULL_MV" counter.
static bf_batt_charge_state_t    s_charge       = BF_BATT_CHARGE_DISCHARGING;
static uint16_t                  s_full_stable_s = 0;

// Lockout latch — once voltage fell below BF_BATT_LOCKOUT_MV the screen
// is held on the LOW BATTERY scene until the user plugs in the charger.
// `s_vbus_prev` lets us detect the LOW→HIGH edge on VBUS that releases
// the latch. Initialised lazily on first sample (see monitor_task) so
// the boot-time edge is observed against the actual rail state, not a
// false "LOW" assumption that would race the first sub-threshold
// reading and latch lockout on USB-powered boots.
static bool                      s_lockout      = false;
static bool                      s_vbus_prev    = false;
static bool                      s_vbus_prev_init = false;

// Debounce counter for lockout engage. A single sub-threshold sample
// (e.g. motor / BLE TX rail sag) used to trip lockout immediately;
// require N consecutive sub-threshold readings before latching.
#define BF_LOCKOUT_DEBOUNCE_SAMPLES 3
static uint8_t                   s_lockout_streak = 0;

// Threshold state machine — one-shot transitions, no per-sample spam.
typedef enum {
    BATT_STATE_OK = 0,
    BATT_STATE_LOW,
    BATT_STATE_CRITICAL,
} batt_state_t;
static batt_state_t s_state = BATT_STATE_OK;

// ---------------------------------------------------------------------
// ADC + calibration setup
// ---------------------------------------------------------------------

static esp_err_t adc_setup(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = BATT_ADC_UNIT,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc);
    if (err != ESP_OK) return err;

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = BATT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(s_adc, BATT_ADC_CHANNEL, &chan_cfg);
    if (err != ESP_OK) return err;

    // Curve fitting calibration — supported on C6 in IDF v5.x.
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = BATT_ADC_UNIT,
        .chan     = BATT_ADC_CHANNEL,
        .atten    = BATT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "calibration unavailable (%s) — using raw scaling",
                 esp_err_to_name(err));
        s_cali = NULL;
        // Not fatal: we'll fall back to raw mV computation below.
    }
    return ESP_OK;
}

// Insertion sort — small N (64), branch-predictable, no recursion.
static void sort_ints(int *a, int n)
{
    for (int i = 1; i < n; i++) {
        int v = a[i], j = i - 1;
        while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
        a[j + 1] = v;
    }
}

// Read OVERSAMPLE_COUNT raw ADC samples, return the median voltage at the
// ADC pin in millivolts. Median (not mean) rejects single-sample spikes
// from RF activity (WiFi TX, BLE adv) without dragging the answer toward
// the outlier the way an average does.
static esp_err_t read_pin_mv(int *out_mv)
{
    int samples[OVERSAMPLE_COUNT];
    for (int i = 0; i < OVERSAMPLE_COUNT; i++) {
        int r = 0;
        esp_err_t err = adc_oneshot_read(s_adc, BATT_ADC_CHANNEL, &r);
        if (err != ESP_OK) return err;
        samples[i] = r;
    }
    sort_ints(samples, OVERSAMPLE_COUNT);
    int raw_median = samples[OVERSAMPLE_COUNT / 2];

    if (s_cali) {
        return adc_cali_raw_to_voltage(s_cali, raw_median, out_mv);
    }
    // Fallback: assume 12-bit ADC, full-scale ≈ 3300 mV.
    *out_mv = (raw_median * 3300) / 4095;
    return ESP_OK;
}

// True if the vibration motor is driving the rail right now. Sampling
// while the motor is on causes 100-300 mV of sag and produces a false
// "battery low" reading. We just skip the sample.
static bool load_active(void)
{
    return gpio_get_level(PIN_VIBRATION) != 0;
}

static bool vbus_present(void)
{
    return gpio_get_level(PIN_VBUS_SENSE) != 0;
}

// ---------------------------------------------------------------------
// State updates + event publishing
// ---------------------------------------------------------------------

// Single-cell LiPo voltage → SoC piecewise linear interpolation.
//
// We deliberately stretch the displayed bar so the long mid-plateau
// (4.00 V → 3.40 V) covers 75 % of the bar — that's where most of the
// real energy lives. The top 200 mV (4.20 → 4.00 V) compresses to
// just 10 % of the bar, which matches the user expectation: a freshly-
// charged pack drops fast from 100 → 90 %, then sits at the same number
// for hours of normal use. The bottom 400 mV (3.40 → 3.00 V) maps to
// 15 % so the warning band has visible resolution before brown-out.
//
// Anchor design (4 points):
//   4200 mV →  100 %     CV-phase / freshly charged
//   4000 mV →   90 %     post-relaxation, "real" full
//   3400 mV →   15 %     onset of the cliff, low-battery icon territory
//   3000 mV →    0 %     cutoff
//
// Table sorted ascending by mV. We interpolate linearly between
// adjacent anchors; below the first / above the last we clamp.
typedef struct { uint16_t mv; uint8_t pct; } batt_anchor_t;

static const batt_anchor_t s_curve[] = {
    { 3000,   0 },
    { 3400,  15 },
    { 4000,  90 },
    { 4200, 100 },
};

static int compute_percent(uint16_t mv)
{
    if (mv == 0)                 return 0;
    if (mv <= s_curve[0].mv)     return s_curve[0].pct;
    int last = (int)(sizeof(s_curve) / sizeof(s_curve[0])) - 1;
    if (mv >= s_curve[last].mv)  return s_curve[last].pct;

    for (int i = 0; i < last; i++) {
        const batt_anchor_t *a = &s_curve[i];
        const batt_anchor_t *b = &s_curve[i + 1];
        if (mv >= a->mv && mv <= b->mv) {
            int span_mv  = b->mv  - a->mv;
            int span_pct = b->pct - a->pct;
            return a->pct + ((int)mv - a->mv) * span_pct / span_mv;
        }
    }
    return 0;   // unreachable, silences compiler
}

static void publish(const char *name, uint16_t mv, int pct)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"mv\":%u,\"pct\":%d}", (unsigned)mv, pct);
    sk_event_bus_publish(name, buf);
}

static void update_threshold_state(uint16_t mv, int pct)
{
    // Only run the threshold state machine when the cell is plausibly
    // present. A floating ADC pin (no battery, USB-only) shouldn't fire
    // "battery.critical" alerts.
    if (mv < BF_BATT_PRESENT_MIN_MV) {
        s_state = BATT_STATE_OK;  // reset so we re-emit on cell insertion
        return;
    }

    batt_state_t next = s_state;
    if      (mv <= BF_BATT_CRIT_MV) next = BATT_STATE_CRITICAL;
    else if (mv <= BF_BATT_LOW_MV)  next = (s_state == BATT_STATE_CRITICAL)
                                            ? BATT_STATE_CRITICAL  // hysteresis
                                            : BATT_STATE_LOW;
    else if (mv >= BF_BATT_OK_MV)   next = BATT_STATE_OK;
    // else: between LOW and OK → keep current state (debounce zone)

    if (next == s_state) return;

    if (next == BATT_STATE_CRITICAL && s_state != BATT_STATE_CRITICAL) {
        publish("battery.critical", mv, pct);
        ESP_LOGW(TAG, "CRITICAL: %u mV (%d%%)", (unsigned)mv, pct);
    } else if (next == BATT_STATE_LOW && s_state == BATT_STATE_OK) {
        publish("battery.low", mv, pct);
        ESP_LOGW(TAG, "low: %u mV (%d%%)", (unsigned)mv, pct);
    } else if (next == BATT_STATE_OK && s_state != BATT_STATE_OK) {
        publish("battery.recovered", mv, pct);
        ESP_LOGI(TAG, "recovered: %u mV (%d%%)", (unsigned)mv, pct);
    }
    s_state = next;
}

// Push a sample into the dV/dt ring and return the slope in µV/s across
// the window. Returns 0 until the ring is fully populated (avoids a
// false "rising" verdict from the seed-zero entries).
static int dvdt_uv_per_s(uint16_t mv)
{
    s_dvdt_ring[s_dvdt_head] = mv;
    s_dvdt_head = (s_dvdt_head + 1) % DVDT_WINDOW_N;
    if (s_dvdt_count < DVDT_WINDOW_N) { s_dvdt_count++; return 0; }

    uint16_t oldest = s_dvdt_ring[s_dvdt_head];   // head now points to oldest
    int dmv  = (int)mv - (int)oldest;
    int dt_s = (DVDT_WINDOW_N - 1) * (SAMPLE_PERIOD_MS / 1000);
    return (dmv * 1000) / dt_s;                   // mV*1000/s = µV/s
}

// Lockout latch — separate from the charge state machine because it
// only releases on a *transition* (LOW→HIGH on VBUS) rather than a level.
// A device that boots already on USB sees s_vbus_prev=false on the very
// first sample, so the first vbus_present()==true also counts as an
// edge — which is what we want: an already-plugged device starts un-locked.
static void update_lockout(uint16_t mv, bool vbus)
{
    bool prev = s_lockout;

    // First sample: seed s_vbus_prev with the actual rail state so the
    // edge detector below uses a meaningful baseline. Without this, a
    // device that boots already on USB-C sees `s_vbus_prev=false` on
    // the first read and the FIRST sample's "vbus && !s_vbus_prev"
    // edge fires inside this very call — fine for normal boot, but
    // when combined with the floating-VBUS prototype scenario the
    // first vbus_present() may incorrectly return false, denying the
    // release path and latching lockout for the rest of the session.
    // Initialising once explicitly removes the race.
    if (!s_vbus_prev_init) {
        s_vbus_prev = vbus;
        s_vbus_prev_init = true;
    }

    // Engage on low voltage, only when:
    //   - not already on the charger (a charging cell can sit at 3.05 V
    //     briefly during pre-charge),
    //   - AND the cell is plausibly present. Without this guard, a
    //     dev board running on USB with NO battery installed reads
    //     ADC noise (a few mV from the floating BAT+ side of the
    //     divider) AND a floating VBUS-sense pin (if the on-PCB
    //     divider isn't soldered yet) — which together satisfy
    //     `!vbus && mv > 0 && mv < LOCKOUT_MV` and falsely engage
    //     the lockout. Anchor the check at BF_BATT_PRESENT_MIN_MV
    //     (2500 mV) so noise alone can't trip it.
    //   - AND the sub-threshold condition has held for N consecutive
    //     samples. Single-sample latch was tripping on motor pulse /
    //     BLE TX rail sag; debounce keeps lockout for genuine drain.
    if (!vbus && mv >= BF_BATT_PRESENT_MIN_MV && mv < BF_BATT_LOCKOUT_MV) {
        if (s_lockout_streak < BF_LOCKOUT_DEBOUNCE_SAMPLES) {
            s_lockout_streak++;
        }
        if (s_lockout_streak >= BF_LOCKOUT_DEBOUNCE_SAMPLES) {
            s_lockout = true;
        }
    } else {
        // Voltage recovered (rail bounced back, motor pulse passed,
        // or VBUS came back); reset the streak so a future sag has to
        // re-accumulate from zero.
        s_lockout_streak = 0;
    }

    // Release whenever USB is present (level, not just the rising edge).
    // A rev1 PCB whose VBUS-sense divider is unpopulated can fail to produce
    // a clean LOW->HIGH edge, which previously left lockout latched forever
    // with no software escape. Level-release clears it as soon as any sample
    // reads charging; lockout only ENGAGES on !vbus, so this cannot fight the
    // engage path.
    if (vbus) {
        s_lockout = false;
        s_lockout_streak = 0;
    }
    s_vbus_prev = vbus;

    if (s_lockout != prev) {
        char buf[40];
        snprintf(buf, sizeof(buf), "{\"active\":%s}",
                 s_lockout ? "true" : "false");
        sk_event_bus_publish("battery.lockout", buf);
        ESP_LOGW(TAG, "lockout %s (%u mV, vbus=%d)",
                 s_lockout ? "engaged" : "released",
                 (unsigned)mv, vbus ? 1 : 0);
    }
}

static void publish_charge(bf_batt_charge_state_t st)
{
    const char *name =
        st == BF_BATT_CHARGE_FULL     ? "full" :
        st == BF_BATT_CHARGE_CHARGING ? "charging" : "discharging";
    char buf[48];
    snprintf(buf, sizeof(buf), "{\"state\":\"%s\"}", name);
    sk_event_bus_publish("battery.charge_state", buf);
}

static void update_charge_state(uint16_t mv, int slope_uv_s)
{
    bf_batt_charge_state_t next = s_charge;

    if (!vbus_present()) {
        next = BF_BATT_CHARGE_DISCHARGING;
        s_full_stable_s = 0;
    } else {
        // Track "stable at/above full voltage" for charge-complete detection.
        if (mv >= CHARGE_FULL_MV) {
            uint32_t inc = SAMPLE_PERIOD_MS / 1000;
            if ((uint32_t)s_full_stable_s + inc > UINT16_MAX) s_full_stable_s = UINT16_MAX;
            else s_full_stable_s += (uint16_t)inc;
        } else {
            s_full_stable_s = 0;
        }

        if (s_full_stable_s >= CHARGE_FULL_STABLE_S) {
            next = BF_BATT_CHARGE_FULL;
        } else if (slope_uv_s >= DVDT_RISING_UV_S) {
            next = BF_BATT_CHARGE_CHARGING;
        } else if (s_charge == BF_BATT_CHARGE_DISCHARGING) {
            // Just plugged in, no slope evidence yet — assume charging
            // (safer default than "full" for a UI icon).
            next = BF_BATT_CHARGE_CHARGING;
        }
        // else: keep current state (between samples we don't flip-flop)
    }

    if (next != s_charge) {
        s_charge = next;
        publish_charge(next);
        ESP_LOGI(TAG, "charge_state -> %s",
                 next == BF_BATT_CHARGE_FULL     ? "full" :
                 next == BF_BATT_CHARGE_CHARGING ? "charging" : "discharging");
    }
}

// Update the displayed percent: EWMA on the new measurement, then commit
// to s_pct_displayed only when the smoothed value has moved by at least
// PCT_HYSTERESIS. While VBUS is plugged in we freeze the displayed value
// at the last pre-USB snapshot (s_pct_frozen).
static void update_displayed_percent(int raw_pct, bool vbus)
{
    if (vbus) {
        // While charging, freeze at the last pre-USB snapshot (CV-phase
        // voltage can't be mapped to SoC). BUT if we booted already on USB
        // there is no snapshot yet (s_pct_frozen == -1) — show the voltage-
        // based estimate instead of a misleading 0% (a full pack at 4.2 V
        // was reading 0%). compute_percent(4206mV) ~= 100, so a topped-off
        // cell now correctly reads full.
        s_pct_displayed = (s_pct_frozen >= 0) ? s_pct_frozen : raw_pct;
        return;
    }

    if (s_pct_ewma_x256 < 0) {
        s_pct_ewma_x256 = raw_pct * 256;
    } else {
        s_pct_ewma_x256 += (raw_pct * 256 - s_pct_ewma_x256) * PCT_EWMA_NUM / PCT_EWMA_DEN;
    }
    int smoothed = s_pct_ewma_x256 / 256;
    if (smoothed < 0)   smoothed = 0;
    if (smoothed > 100) smoothed = 100;

    if (abs(smoothed - s_pct_displayed) >= PCT_HYSTERESIS) {
        s_pct_displayed = smoothed;
    }
    s_pct_frozen = s_pct_displayed;   // keep the freeze snapshot fresh
}

static void monitor_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (load_active()) {
            // Buzzer/vibration is sagging the rail. Skip — keep the
            // previous reading and try again on the next tick.
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
            continue;
        }

        int pin_mv = 0;
        esp_err_t err = read_pin_mv(&pin_mv);
        if (err == ESP_OK && pin_mv >= 0) {
            uint32_t cell_mv = (uint32_t)pin_mv * BATTERY_DIVIDER_RATIO_X100 / 100;
            if (cell_mv > 65535) cell_mv = 65535;
            s_voltage_mv = (uint16_t)cell_mv;
            s_present    = (s_voltage_mv >= BF_BATT_PRESENT_MIN_MV);

            int  slope = dvdt_uv_per_s(s_voltage_mv);
            bool vbus  = vbus_present();
            int  raw_pct = compute_percent(s_voltage_mv);

            update_displayed_percent(raw_pct, vbus);
            update_charge_state(s_voltage_mv, slope);
            update_lockout(s_voltage_mv, vbus);

            publish("battery.sample", s_voltage_mv, s_pct_displayed);
            // Threshold alerts use the smoothed displayed voltage logic
            // via mv (low/critical are voltage-anchored, not %-anchored).
            // Only run them while not on USB — a charging cell shouldn't
            // raise "battery.low" as it ramps through 3.3 V from empty.
            if (!vbus) update_threshold_state(s_voltage_mv, s_pct_displayed);
        } else if (err != ESP_OK) {
            ESP_LOGW(TAG, "read failed: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

// ---------------------------------------------------------------------
// Public getters
// ---------------------------------------------------------------------

uint16_t bf_battery_voltage_mv(void) { return s_adc_ok ? s_voltage_mv : 0; }
bool     bf_battery_present(void)    { return s_adc_ok && s_present; }
int      bf_battery_percent(void)
{
    if (!s_adc_ok || !s_present) return 0;
    return s_pct_displayed;
}
bf_batt_charge_state_t bf_battery_charge_state(void) { return s_charge; }
bool bf_battery_lockout_active(void) { return s_lockout; }

// device.info battery provider (registered with sk_baseline so device.info
// reports real battery state instead of a placeholder). Maps internal state
// to the sk_baseline_set_battery_provider contract.
static bool batt_info_provider(int *mv, int *pct, const char **charge)
{
    if (!bf_battery_present()) return false;
    if (mv)  *mv  = (int)bf_battery_voltage_mv();
    if (pct) *pct = bf_battery_percent();
    if (charge) {
        *charge = s_charge == BF_BATT_CHARGE_FULL     ? "full" :
                  s_charge == BF_BATT_CHARGE_CHARGING ? "charging" : "discharging";
    }
    return true;
}

// ---------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------

static sk_err_t cmd_battery_status(sk_cli_ctx_t *ctx)
{
    const char *charge_s =
        s_charge == BF_BATT_CHARGE_FULL     ? "full" :
        s_charge == BF_BATT_CHARGE_CHARGING ? "charging" : "discharging";

    char buf[220];
    snprintf(buf, sizeof(buf),
             "{\"present\":%s,\"mv\":%u,\"pct\":%d,\"state\":\"%s\","
             "\"charge\":\"%s\",\"vbus\":%s,\"lockout\":%s,\"adc_ok\":%s}",
             bf_battery_present() ? "true" : "false",
             (unsigned)bf_battery_voltage_mv(),
             bf_battery_percent(),
             s_state == BATT_STATE_CRITICAL ? "critical" :
             s_state == BATT_STATE_LOW      ? "low"      : "ok",
             charge_s,
             vbus_present() ? "true" : "false",
             s_lockout ? "true" : "false",
             s_adc_ok ? "true" : "false");
    sk_cli_ok(ctx, buf);
    return SK_OK;
}

static const sk_cli_command_t s_cmd_status = {
    .name    = "battery.status",
    .summary = "Show battery voltage, percent, and threshold state",
    .usage   = "battery status",
    .help_block =
        "Returns the live battery snapshot. Fields:\n"
        "  mv         battery voltage in millivolts (3000-4200 typical)\n"
        "  pct        0-100, EWMA-smoothed, frozen while charging\n"
        "  state      ok | low (<15%) | critical (<5%)\n"
        "  charge     discharging | charging | full\n"
        "  vbus       true if USB plugged in\n"
        "  lockout    true if LOW BATTERY screen is held until charge\n"
        "  adc_ok     false if the ADC channel failed to init\n"
        "\n"
        "Read-only.\n"
        "\n"
        "Example:\n"
        "  battery.status",
    .handler = cmd_battery_status,
};

// ---------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------

esp_err_t bf_battery_init(void)
{
    static bool s_inited = false;
    if (s_inited) return ESP_OK;

    esp_err_t err = adc_setup();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC init failed (%s) — battery monitoring disabled",
                 esp_err_to_name(err));
        // Boot continues; getters will return 0, no events fire.
        s_inited = true;
        return ESP_OK;
    }
    s_adc_ok = true;

    // VBUS sense: dijital input through external 10k+10k divider.
    // Prototype boards may ship without the divider populated (see
    // pins.h note); a floating pin then reads non-deterministic and
    // the device latches into battery-lockout while running on USB-C.
    // Internal pull-up forces the safer fail-mode: "USB present" when
    // the divider is absent. Production boards with the divider
    // soldered keep deterministic behaviour because the divider's
    // output impedance (~5kΩ at the midpoint) easily overrides the
    // weak ~45kΩ internal pull when actual VBUS state changes — the
    // line tracks USB plug/unplug cleanly.
    gpio_config_t vbus_cfg = {
        .pin_bit_mask = 1ULL << PIN_VBUS_SENSE,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t vbus_err = gpio_config(&vbus_cfg);
    if (vbus_err != ESP_OK) {
        ESP_LOGW(TAG, "VBUS sense init failed (%s) — charge_state will stay 'discharging'",
                 esp_err_to_name(vbus_err));
    }

    sk_cli_register(&s_cmd_status);
    sk_baseline_set_battery_provider(batt_info_provider);
    sk_capabilities_register_book("bf_battery", "0.1.0");

    // 3072 was overflowing under ESP_LOG vfprintf load when NimBLE notify
    // events arrived in burst (peer connect during ECDH). 5120 leaves
    // comfortable headroom for the ADC poll loop + log formatting.
    BaseType_t r = xTaskCreate(monitor_task, "bf_battery", 5120, NULL, 4, NULL);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "monitor task spawn failed");
        return ESP_FAIL;
    }

    s_inited = true;
    ESP_LOGI(TAG, "ready (divider=%d.%02dx, period=%d ms)",
             BATTERY_DIVIDER_RATIO_X100 / 100,
             BATTERY_DIVIDER_RATIO_X100 % 100,
             SAMPLE_PERIOD_MS);
    return ESP_OK;
}
