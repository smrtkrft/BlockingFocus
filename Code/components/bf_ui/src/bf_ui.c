// =====================================================================
// bf_ui — see header. Implementation strategy:
//   - One FreeRTOS task ("bf_ui") drives all rendering. It blocks on a
//     queue; events (button, timer.state, ...) push render messages.
//   - Boot sequence is hard-coded as splash → brand → idle.
//   - Idle scene shows the "BLOKING" / "FOCUS" wordmark; a running timer
//     switches the panel to the big MM:SS countdown.
// =====================================================================

#include "bf_ui.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "bf_vibration.h"
#include "bf_display.h"
#include "bf_ui_logo.h"
#include "sk_capabilities.h"
#include "sk_event_bus.h"

static const char *TAG = "bf_ui";

// ---------------------------------------------------------------------
// Layout constants
// ---------------------------------------------------------------------

#define SPLASH_HOLD_MS  2000
#define BRAND_HOLD_MS   2000

// ---------------------------------------------------------------------
// State + queue
// ---------------------------------------------------------------------

typedef enum {
    SCENE_SPLASH,
    SCENE_BRAND,
    SCENE_IDLE,
    SCENE_COUNTDOWN,    // big MM:SS while a timer is counting down
    SCENE_BREAK_MSG,    // "Focus Break" full-screen for 3 s after expired
    SCENE_API_ERROR,    // "OOPS / API send failed" 5 s on api.sent ok=false
    SCENE_BATTERY_LOW,  // full-screen "LOW BATTERY" lockout — only released
                        // when bf_battery clears the latch (USB plug-in).
                        // Overrides every other scene while active.
} ui_scene_t;

#define BREAK_MSG_HOLD_MS   3000
#define API_ERROR_HOLD_MS   5000

typedef enum {
    EVT_TICK,           // forced re-render (e.g. tilt overlay change)
    EVT_TIMER_STATE,    // timer.state event → scene switch (idle/countdown)
    EVT_TIMER_TICK,     // timer.tick → countdown remaining (i = remaining_sec)
    EVT_BUTTON_DOWN,    // user just pressed the button
    EVT_BUTTON_HOLD,    // user has held past a threshold (i = reached_ms)
    EVT_BUTTON_UP,      // user released the button
    EVT_SHOW_BREAK,     // timer.state state="expired" → Focus Break splash
    EVT_SHOW_API_ERR,   // api.sent ok=false → OOPS splash
} evt_type_t;

typedef struct {
    evt_type_t type;
    int        i;       // small payload: remaining_sec (TICK) / reached_ms (HOLD)
} evt_t;

static QueueHandle_t      s_q       = NULL;
static ui_scene_t         s_scene   = SCENE_SPLASH;

// Hold-progress UI state — set by button.down/hold/up events. Drawn
// as a thin bar at the top of the idle scene while the user keeps the
// button pressed. Max threshold is 10 s (matches sk_button's table).
static bool               s_hold_active   = false;
static int                s_hold_reached_ms = 0;
#define HOLD_BAR_MAX_MS   10000

// Countdown state — refreshed by timer.tick events.
static int                s_countdown_sec = 0;
static bool               s_timer_counting = false;   // set by timer.state subscriber

// Status bar flags — top of countdown scene shows BLE / WiFi icons.
// Updated by ble.peer.state and wifi.state subscribers (atomic bool
// writes; readers see "good enough" approximation, no lock needed).
static bool               s_wifi_on = false;
static bool               s_ble_on  = false;

// Real battery readings from bf_battery events. -1 = no sample yet, fall
// back to the CLI test override. s_battery_low is set on `battery.low` /
// `battery.critical` and cleared on `battery.recovered`; while true, the
// status-bar battery icon blinks at 1 Hz (CLAUDE.md hardware rule #8).
static int                s_battery_pct_real = -1;
static bool               s_battery_low      = false;
static int64_t            s_battery_blink_last_us = 0;
#define BATTERY_BLINK_HALF_MS  500    // 1 Hz → 500 ms on / 500 ms off

// Lockout flag — set by `battery.lockout {"active":true}` events. While
// true, render_current() forces SCENE_BATTERY_LOW regardless of whatever
// scene the timer / face / button handlers tried to switch to. Cleared
// only when bf_battery emits `battery.lockout {"active":false}` after a
// USB plug-in edge.
static bool               s_battery_lockout  = false;

// Tilted overlay flag — set/cleared by face.tilted events from
// bf_face_detector. While true, both COUNTDOWN and IDLE scenes draw a
// small warning glyph + Turkish hint without touching the underlying
// content (countdown keeps running, the wordmark stays visible).
static bool               s_tilted = false;

// Reason text shown by SCENE_API_ERROR. Filled by on_api_sent when an
// `api.sent ok=false` event arrives — mapped from the `err` field to a
// short Turkish hint. Empty string falls back to the generic message.
#define API_ERR_REASON_MAX  20
static char               s_api_err_reason[API_ERR_REASON_MAX] = {0};

// Message-scene deadline. When SCENE_BREAK_MSG / SCENE_API_ERROR is
// active, the bf_ui task loop watches now_us() against this and
// transitions back to IDLE / COUNTDOWN once it passes.
static int64_t            s_msg_deadline_us = 0;

// ---------------------------------------------------------------------
// Drawing primitives (built on top of bf_display)
// ---------------------------------------------------------------------

// Blit the SmartKraft logo bitmap (page-major, 1 bpp) at top-left
// (px, py). Bitmap is BF_UI_LOGO_W × BF_UI_LOGO_H. ON pixels are drawn,
// OFF pixels are left untouched (so the caller can layer on top of a
// pre-cleared framebuffer).
static void blit_logo(int16_t px, int16_t py)
{
    int pages = BF_UI_LOGO_H / 8;
    for (int page = 0; page < pages; page++) {
        for (int col = 0; col < BF_UI_LOGO_W; col++) {
            uint8_t byte = BF_UI_LOGO[page * BF_UI_LOGO_W + col];
            if (!byte) continue;
            for (int bit = 0; bit < 8; bit++) {
                if (byte & (1u << bit)) {
                    bf_display_pixel(px + col, py + page * 8 + bit, BF_PIX_ON);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------
// Scene renderers — each clears the framebuffer and presents.
// ---------------------------------------------------------------------

// Forward decl: render_tilted_overlay is defined further down (alongside
// render_status_bar) but called from render_idle/render_countdown above
// it. Plain prototype keeps the file's existing top-down order intact.
static void render_tilted_overlay(void);

static void render_splash(void)
{
    // Boot orientation: splash + brand 0°'de sabit (PCB rev'iyle 180° flip).
    // on_face_changed_rot bu pencerede gate'li; render her tick'te
    // idempotent olarak set eder.
    bf_display_set_rotation(0);
    bf_display_clear(BF_PIX_OFF);

    // Logo centered horizontally, near the top.
    int lx = (BF_DISPLAY_WIDTH - BF_UI_LOGO_W) / 2;
    int ly = 4;
    blit_logo(lx, ly);

    // SmartKraft brand text — scale 2 (bigger), positioned a bit above
    // the bottom edge so it has visual weight without crowding the logo.
    // 5x7 font, 6px advance × scale 2 → 12 px per char. 10 chars = 120 px,
    // last char no trailing gap → 118 px wide. Fits 128 with 5 px margin.
    const char *brand = "SmartKraft";
    int scale = 2;
    int tw = (int)strlen(brand) * 6 * scale - scale;     // 118
    int ty = BF_DISPLAY_HEIGHT - 7 * scale - 6;          // y = 128 - 14 - 6 = 108
    bf_display_text((BF_DISPLAY_WIDTH - tw) / 2, ty, brand, scale, BF_PIX_ON);

    bf_display_present();
}

// "BLOKING" / "FOCUS" two-line wordmark at scale 3, vertically placed to
// match the boot brand screen. Caller is responsible for clear + present.
static void draw_wordmark(void)
{
    // BLOKING (7 chars) at scale 3 → 7 × 18 - 3 = 123 px wide, fits 128.
    int w1 = 7 * 6 * 3 - 3;
    bf_display_text((BF_DISPLAY_WIDTH - w1) / 2, 36, "BLOKING", 3, BF_PIX_ON);
    // FOCUS (5 chars) at scale 3 → 5 × 18 - 3 = 87 px wide.
    int w2 = 5 * 6 * 3 - 3;
    bf_display_text((BF_DISPLAY_WIDTH - w2) / 2, 76, "FOCUS",   3, BF_PIX_ON);
}

static void render_brand(void)
{
    // Brand boyunca splash ile aynı 0° rotation korunur (PCB rev flip).
    bf_display_set_rotation(0);
    bf_display_clear(BF_PIX_OFF);
    draw_wordmark();
    bf_display_present();
}

static int64_t now_us(void) { return esp_timer_get_time(); }

static void render_idle(void)
{
    bf_display_clear(BF_PIX_OFF);

    // Identity wordmark — same "BLOKING" / "FOCUS" treatment as the boot
    // brand screen. Shown while the cube rests on its display face with
    // no timer running (replaces the old assistant-eye scene).
    draw_wordmark();

    // Hold-progress bar at the very top — visible only while the user is
    // holding the button. Width fills proportionally to how far past each
    // threshold (1s, 3s, 5s, 10s) they've gone.
    if (s_hold_active) {
        int filled = (s_hold_reached_ms * BF_DISPLAY_WIDTH) / HOLD_BAR_MAX_MS;
        if (filled > BF_DISPLAY_WIDTH) filled = BF_DISPLAY_WIDTH;
        if (filled < 4 && s_hold_reached_ms == 0) filled = 4;   // initial nub at button.down
        bf_display_fill_rect(0, 0, filled, 3, BF_PIX_ON);
    }

    // WiFi-off marker in the idle scene — countdown gets the full status
    // bar already, but the wordmark scene has no chrome. A tiny "!" badge
    // in the top-right keeps the user aware that an upcoming timer expiry
    // would have nowhere to send its API call.
    if (!s_wifi_on) {
        int x = BF_DISPLAY_WIDTH - 4;
        bf_display_fill_rect(x, 2, 2, 6, BF_PIX_ON);   // stem
        bf_display_fill_rect(x, 9, 2, 2, BF_PIX_ON);   // dot
    }

    render_tilted_overlay();
    bf_display_present();
}

// Status-bar icons. Designs picked from logo/icons_preview.html:
//   WiFi    W4 (cellular bars 14×12) ON / W4-OFF-D (!) OFF
//   BLE     B4 (Bluetooth outline 14×12)
//   Battery 22×9 body + 2×5 nub. CHG-6 plus-sign overlay when charging.
// All drawn with primitives so bf_display_pixel rotation applies for
// free — the status bar rotates with the countdown digits.

// WiFi W4 — 4 vertical "cellular signal" bars increasing in height
// (heights 3, 5, 7, 10 px). Disconnected: an exclamation mark "!" in
// the middle of the icon area (W4-OFF-D).
static void draw_wifi_icon(int x, int y, bool connected)
{
    if (connected) {
        bf_display_fill_rect(x + 1,  y + 9, 2, 3,  BF_PIX_ON);
        bf_display_fill_rect(x + 4,  y + 7, 2, 5,  BF_PIX_ON);
        bf_display_fill_rect(x + 7,  y + 5, 2, 7,  BF_PIX_ON);
        bf_display_fill_rect(x + 10, y + 2, 2, 10, BF_PIX_ON);
    } else {
        // "!" — stem + dot, both 2 px wide, centered.
        bf_display_fill_rect(x + 6, y + 2, 2, 6, BF_PIX_ON);   // stem
        bf_display_fill_rect(x + 6, y + 9, 2, 2, BF_PIX_ON);   // dot
    }
}

// BLE B4 — Bluetooth outline glyph, pixel-perfect (matches the
// icons_preview.html B4 bitmap). Disconnected: not drawn.
static void draw_ble_icon(int x, int y, bool connected)
{
    if (!connected) return;
    static const uint8_t pts[][2] = {
        {6,0}, {7,0},
        {6,1}, {8,1},
        {3,2}, {6,2}, {9,2},
        {4,3}, {6,3}, {9,3},
        {5,4}, {6,4}, {8,4},
        {6,5}, {7,5},
        {5,6}, {6,6}, {8,6},
        {4,7}, {6,7}, {9,7},
        {3,8}, {6,8}, {9,8},
        {6,9}, {8,9},
        {6,10}, {7,10},
    };
    for (size_t i = 0; i < sizeof(pts) / sizeof(pts[0]); i++) {
        bf_display_pixel(x + pts[i][0], y + pts[i][1], BF_PIX_ON);
    }
}

// Battery: 20 × 9 outlined body + 2 × 5 terminal nub. Fill bar shows
// percent of interior filled; bf_battery publishes battery.sample events
// from which s_battery_pct_real is updated, so the icon tracks the
// voltage curve live as the cell discharges (or rises while plugged in).
//
// Charging-state overlay (CHG-6 plus sign) intentionally absent — see
// note in render_status_bar() for why.
static void draw_battery_icon(int x, int y, int percent)
{
    if (percent < 0)   percent = 0;
    if (percent > 100) percent = 100;

    const int w = 20, h = 9;
    bf_display_fill_rect(x,         y,            w, 1, BF_PIX_ON);  // top
    bf_display_fill_rect(x,         y + h - 1,    w, 1, BF_PIX_ON);  // bottom
    bf_display_fill_rect(x,         y,            1, h, BF_PIX_ON);  // left
    bf_display_fill_rect(x + w - 1, y,            1, h, BF_PIX_ON);  // right
    bf_display_fill_rect(x + w,     y + 2,        2, 5, BF_PIX_ON);  // nub

    int inner_w  = w - 4;
    int filled_w = (inner_w * percent + 50) / 100;
    if (filled_w > 0) {
        bf_display_fill_rect(x + 2, y + 2, filled_w, h - 4, BF_PIX_ON);
    }
}

// Triangular "⚠" glyph at (x, y) — base 11 px, height 9 px, with a
// vertical stem-and-dot inside. Drawn from primitives so it follows the
// display rotation. Used by render_tilted_overlay.
static void draw_warn_triangle(int x, int y)
{
    // Outline: triangle ascending from base. Each row narrows by 1 px
    // per side; baseline at y+8, apex at y+0.
    for (int dy = 0; dy <= 8; dy++) {
        bf_display_pixel(x + 5 - dy / 2, y + dy, BF_PIX_ON);   // left edge
        bf_display_pixel(x + 5 + dy / 2, y + dy, BF_PIX_ON);   // right edge
    }
    bf_display_fill_rect(x, y + 8, 11, 1, BF_PIX_ON);          // base
    // Inner exclamation (stem + dot) — leave a 1 px gap from base/apex.
    bf_display_fill_rect(x + 5, y + 3, 1, 3, BF_PIX_ON);       // stem
    bf_display_pixel    (x + 5, y + 7,         BF_PIX_ON);     // dot
}

// Draw the tilted-overlay hint on top of whatever the active scene
// already rendered. Triangle in the top-right corner, short Turkish
// hint along the bottom band. Called after the scene's own draw + just
// before bf_display_present(). Caller MUST NOT have presented yet.
//
// Gated by `bf_tilt_warn_is_enabled()` (the same flag bf_vibration reads
// for its audible alert) so a user who turned the warning OFF in SKAPP
// gets neither the beep nor the on-screen overlay — symmetric behaviour
// across both modalities.
static void render_tilted_overlay(void)
{
    if (!s_tilted) return;
    if (!bf_tilt_warn_is_enabled()) return;
    // Warning glyph in top-right area (skip when status bar already has
    // icons there). 11×9, 4 px from right edge / top.
    draw_warn_triangle(BF_DISPLAY_WIDTH - 11 - 4, 4);
    // Bottom hint — small text, transliterated ASCII (the built-in 5×7
    // font has no Turkish glyphs yet: ç→c, ö→o, ş→s, ü→u, ı→i).
    const char *hint = "Kupu duz koy";
    int w = (int)strlen(hint) * 6 - 1;
    bf_display_text((BF_DISPLAY_WIDTH - w) / 2,
                    BF_DISPLAY_HEIGHT - 10, hint, 1, BF_PIX_ON);
}

static void render_status_bar(void)
{
    // Layout (left → right): WiFi, BLE, gap, Battery — phone-style.
    // Countdown digits are big and central (in render_countdown), so
    // the status bar stays uncluttered with just the three icons.
    int by = 2;
    draw_wifi_icon(2,            by, s_wifi_on);
    draw_ble_icon (2 + 14 + 4,   by, s_ble_on);

    int pct = (s_battery_pct_real >= 0) ? s_battery_pct_real : 0;

    // PCB-REV1-VBUS: 5v algilama pcb tasariminda unutuldu, ilk parti
    // pcblerde cihaz sarj ediliyor mu algilanamiyor, ikinci pcb
    // cizimlerinde eklenecek, o zamana kadar sistemde duzeltme
    // gerekiyor, gecici olarak kaldirildi, daha sonra yeniden
    // planlanacak. Bu yuzden CHG-6 "+" overlay'i ve charging suppresses
    // blink dali kaldirildi; ikon yalnizca s_battery_pct_real'i
    // (battery.sample event'inden gelen V -> % donusumu) yansitir.
    //
    // <%15 blink: skip drawing the icon for half of every 1 s window
    // so it visibly flashes ~1 Hz.
    bool blink_off = s_battery_low &&
                     ((now_us() / (BATTERY_BLINK_HALF_MS * 1000)) & 1);
    if (!blink_off) {
        draw_battery_icon(BF_DISPLAY_WIDTH - 22 - 4, by + 2, pct);
    }
}

static void render_countdown(void)
{
    bf_display_clear(BF_PIX_OFF);
    render_status_bar();

    // Big 7-seg MM:SS centered vertically — primary information during
    // a focus session. The cube becomes a dedicated timer while running;
    // the wordmark identity lives in the IDLE scene between sessions.
    int digit_w = 22;
    int digit_h = 36;
    int y = (BF_DISPLAY_HEIGHT - digit_h) / 2;
    bf_display_countdown_7seg(y, s_countdown_sec, digit_w, digit_h, BF_PIX_ON);

    render_tilted_overlay();
    bf_display_present();
}

// "Focus Break" — two stacked lines at scale 4 (single-line "Focus
// Break" doesn't fit the 128 px width; "Focus" is 5 chars, "Break" is
// also 5 chars, both fit at scale 4 → 5*6*4 - 4 = 116 px wide).
static void render_break_msg(void)
{
    bf_display_clear(BF_PIX_OFF);
    int scale = 4;
    int line_w = 5 * 6 * scale - scale;     // 116
    int line_h = 7 * scale;                 // 28
    int gap = 8;
    int total_h = 2 * line_h + gap;
    int y0 = (BF_DISPLAY_HEIGHT - total_h) / 2;
    bf_display_text((BF_DISPLAY_WIDTH - line_w) / 2, y0,            "Focus", scale, BF_PIX_ON);
    bf_display_text((BF_DISPLAY_WIDTH - line_w) / 2, y0 + line_h + gap, "Break", scale, BF_PIX_ON);
    bf_display_present();
}

// "OOPS" huge on top, dynamic Turkish reason hint below — falls back to
// "API hatasi" if no api.sent event has populated the buffer yet.
static void render_api_error(void)
{
    bf_display_clear(BF_PIX_OFF);
    // OOPS at scale 5 → 4 chars × 6 × 5 - 5 = 115 px wide, 7×5 = 35 tall.
    int top_scale = 5;
    int top_w = 4 * 6 * top_scale - top_scale;
    int top_h = 7 * top_scale;
    int top_y = 22;
    bf_display_text((BF_DISPLAY_WIDTH - top_w) / 2, top_y, "OOPS", top_scale, BF_PIX_ON);

    // Subtitle: parsed reason from api.sent.err, scale 2 for emphasis
    // (the operator wants to see WHY it failed, not generic chrome).
    const char *sub = s_api_err_reason[0] ? s_api_err_reason : "API hatasi";
    int sub_scale = 2;
    int sub_w = (int)strlen(sub) * 6 * sub_scale - sub_scale;
    if (sub_w > BF_DISPLAY_WIDTH - 4) {  // fallback to scale 1 if too long
        sub_scale = 1;
        sub_w = (int)strlen(sub) * 6 * sub_scale - sub_scale;
    }
    int sub_y = top_y + top_h + 12;
    bf_display_text((BF_DISPLAY_WIDTH - sub_w) / 2, sub_y, sub, sub_scale, BF_PIX_ON);
    bf_display_present();
}

// "LOW BATTERY" lockout — single full-screen message that stays up until
// the user plugs the charger in. Two-line layout matches render_break_msg
// so the visual weight of "the device is halted" feels consistent with
// the existing critical screens. No status bar, no chrome.
static void render_battery_low(void)
{
    bf_display_clear(BF_PIX_OFF);
    // "LOW" at scale 4 (68 px wide). "BATTERY" at scale 4 would overflow
    // the 128 px panel, so it drops to scale 3 (124 px).
    int low_scale  = 4;
    int batt_scale = 3;
    int low_w  = 3 * 6 * low_scale  - low_scale;
    int batt_w = 7 * 6 * batt_scale - batt_scale;
    int low_h  = 7 * low_scale;
    int batt_h = 7 * batt_scale;
    int gap = 6;
    int total_h = low_h + gap + batt_h;
    int y0 = (BF_DISPLAY_HEIGHT - total_h) / 2;
    bf_display_text((BF_DISPLAY_WIDTH - low_w)  / 2, y0,
                    "LOW",     low_scale,  BF_PIX_ON);
    bf_display_text((BF_DISPLAY_WIDTH - batt_w) / 2, y0 + low_h + gap,
                    "BATTERY", batt_scale, BF_PIX_ON);
    bf_display_present();
}

static void render_current(void)
{
    // Lockout overrides everything. The internal scene state may have
    // moved on (face changed, timer state ticked) but the user only sees
    // LOW BATTERY until VBUS goes HIGH and bf_battery clears the latch.
    if (s_battery_lockout) {
        render_battery_low();
        return;
    }
    switch (s_scene) {
    case SCENE_SPLASH:      render_splash();     break;
    case SCENE_BRAND:       render_brand();      break;
    case SCENE_IDLE:        render_idle();       break;
    case SCENE_COUNTDOWN:   render_countdown();  break;
    case SCENE_BREAK_MSG:   render_break_msg();  break;
    case SCENE_API_ERROR:   render_api_error();  break;
    case SCENE_BATTERY_LOW: render_battery_low();break;
    }
}

// ---------------------------------------------------------------------
// Event subscribers (run in event-bus context — only enqueue work)
// ---------------------------------------------------------------------

// True only while a countdown is actively running (resettable / locked).
// PAUSED deliberately falls into SCENE_IDLE (the wordmark) rather than a
// frozen MM:SS — an instant visual cue that the timer is halted. Resume →
// state goes back to RESETTABLE/LOCKED → scene flips back to COUNTDOWN
// automatically.
static bool state_is_counting(const char *st)
{
    if (!st) return false;
    return strstr(st, "resettable") || strstr(st, "locked");
}

static void on_timer_state(const sk_event_t *evt, void *user)
{
    (void)user;
    if (!s_q || !evt->payload_json) return;
    const char *p = strstr(evt->payload_json, "\"state\":\"");
    if (!p) return;
    s_timer_counting = state_is_counting(p + 9);

    // Pre-load s_countdown_sec straight from the timer.state payload so
    // the very first SCENE_COUNTDOWN render after a face change already
    // shows the correct MM:SS. Without this, the first render reuses
    // the previous run's stale value until the next 1 Hz timer.tick
    // arrives — visible as "rotation flips, then ~1 s later the digits
    // jump to the new duration". Atomic int write (single 32-bit store
    // on RV32) so no synchronization needed against the bf_ui task.
    const char *r = strstr(evt->payload_json, "\"remaining_sec\":");
    if (r) {
        int remaining = (int)strtol(r + 16, NULL, 10);
        if (remaining > 0) s_countdown_sec = remaining;
    }

    // Detect timer.expired transition → enqueue Focus Break splash.
    // The state machine then continues to API_TRIGGERING / COOLDOWN
    // on its own; bf_ui will fall back to whatever scene matches the
    // latest s_timer_counting when the splash expires.
    if (strstr(p + 9, "expired")) {
        evt_t splash = { .type = EVT_SHOW_BREAK };
        xQueueSend(s_q, &splash, 0);
    }

    evt_t out = { .type = EVT_TIMER_STATE };
    xQueueSend(s_q, &out, 0);
}

static void on_wifi_state(const sk_event_t *evt, void *user)
{
    (void)user;
    if (!evt->payload_json) return;
    s_wifi_on = (strstr(evt->payload_json, "\"state\":\"connected\"") != NULL);
}

static void on_ble_state(const sk_event_t *evt, void *user)
{
    (void)user;
    if (!evt->payload_json) return;
    // BLE icon is shown whenever the radio is doing something visible
    // to the user (advertising or connected). Hidden only when fully
    // off (after pairing-window timeout or peer-idle 10-min disconnect).
    s_ble_on = (strstr(evt->payload_json, "\"state\":\"advertising\"") ||
                strstr(evt->payload_json, "\"state\":\"connected\""));
}

// Map an SK API error code string (from api.sent payload `err` field)
// to a short Turkish hint. When no specific mapping exists, the raw err
// code is shown directly (truncated to fit) so the user can read what
// actually went wrong — without this, every unrecognised code surfaced as
// the generic "API hatasi" with no debug path on the user's end.
static const char *api_err_to_hint(const char *code)
{
    if (!code || !*code)                         return "API hatasi";
    if (strstr(code, "API_OFFLINE"))             return "WiFi yok";
    if (strstr(code, "API_TIMEOUT"))             return "Zaman asimi";
    if (strstr(code, "API_TLS"))                 return "TLS hatasi";
    if (strstr(code, "API_CONNECT"))             return "Baglanti yok";
    if (strstr(code, "API_BAD_STATUS"))          return "Sunucu hatasi";
    if (strstr(code, "API_NOT_FOUND"))           return "Endpoint yok";
    if (strstr(code, "API_NOT_CONFIGURED"))      return "Eksik yapilandirma";
    if (strstr(code, "API_DISABLED"))            return "API kapali";
    if (strstr(code, "API_INVALID_TYPE"))        return "Tip gecersiz";
    if (strstr(code, "INTERNAL"))                return "Dahili hata";
    // Last-resort: surface the raw code so debugging doesn't end at
    // "API hatasi". Strip the standard "ERR_" / "ERR_API_" prefix to
    // make room on the small display. The static buffer fits the
    // bf_ui scene rendering since reason draws at scale 1.
    static char fallback[24];
    const char *trimmed = code;
    if (strncmp(trimmed, "ERR_API_", 8) == 0)    trimmed += 8;
    else if (strncmp(trimmed, "ERR_", 4) == 0)   trimmed += 4;
    snprintf(fallback, sizeof(fallback), "%.*s",
             (int)(sizeof(fallback) - 1), trimmed);
    return fallback;
}

static void on_api_sent(const sk_event_t *evt, void *user)
{
    (void)user;
    if (!s_q || !evt->payload_json) return;
    if (strstr(evt->payload_json, "\"ok\":false") == NULL) return;

    // Extract `err` field — value is a quoted ASCII string like
    // "ERR_API_OFFLINE". We copy out at most 32 chars so api_err_to_hint
    // can run substring matches on it.
    char err_code[32] = {0};
    const char *p = strstr(evt->payload_json, "\"err\":\"");
    if (p) {
        p += 7;
        const char *q = strchr(p, '"');
        size_t n = q ? (size_t)(q - p) : 0;
        if (n >= sizeof(err_code)) n = sizeof(err_code) - 1;
        memcpy(err_code, p, n);
    }
    const char *hint = api_err_to_hint(err_code);
    strncpy(s_api_err_reason, hint, sizeof(s_api_err_reason) - 1);
    s_api_err_reason[sizeof(s_api_err_reason) - 1] = '\0';

    evt_t err = { .type = EVT_SHOW_API_ERR };
    xQueueSend(s_q, &err, 0);
}

static void on_face_tilted(const sk_event_t *evt, void *user)
{
    (void)user;
    if (!s_q || !evt->payload_json) return;
    bool tilted = (strstr(evt->payload_json, "\"tilted\":true") != NULL);
    if (tilted == s_tilted) return;
    s_tilted = tilted;
    // Force a re-render so the overlay appears/disappears immediately.
    evt_t e = { .type = EVT_TICK };
    xQueueSend(s_q, &e, 0);
}

// Parse {"mv":N,"pct":M} payloads from bf_battery and update local state.
// All four event names route here: sample/low/critical/recovered. The
// blink flag is driven only by low/critical (set) and recovered (clear);
// sample just refreshes the percent so the icon shows fresh data on the
// next render.
static void on_battery_event(const sk_event_t *evt, void *user)
{
    (void)user;
    if (!evt->name || !evt->payload_json) return;

    const char *p = strstr(evt->payload_json, "\"pct\"");
    if (p && (p = strchr(p, ':')) != NULL) {
        s_battery_pct_real = (int)strtol(p + 1, NULL, 10);
    }

    if (strcmp(evt->name, "battery.low") == 0 ||
        strcmp(evt->name, "battery.critical") == 0) {
        s_battery_low = true;
    } else if (strcmp(evt->name, "battery.recovered") == 0) {
        s_battery_low = false;
    }
}

// `battery.lockout {"active":true|false}` — flip the lockout flag and
// kick a fresh render so the screen flips immediately, even if the UI
// task is currently idle. Render is safe from event-bus context because
// bf_display's I2C path is mutex-guarded; we don't queue this through
// s_q to avoid a one-frame delay (a user about to see "LOW BATTERY"
// shouldn't watch an interim scene render first).
static void on_battery_lockout(const sk_event_t *evt, void *user)
{
    (void)user;
    if (!evt->payload_json) return;
    bool active = (strstr(evt->payload_json, "\"active\":true") != NULL);
    if (active == s_battery_lockout) return;
    s_battery_lockout = active;
    render_current();
}

static void on_timer_tick(const sk_event_t *evt, void *user)
{
    (void)user;
    if (!s_q || !evt->payload_json) return;
    const char *p = strstr(evt->payload_json, "\"remaining_sec\":");
    if (!p) return;
    int remaining = (int)strtol(p + 16, NULL, 10);
    evt_t out = { .type = EVT_TIMER_TICK, .i = remaining };
    xQueueSend(s_q, &out, 0);
}

// Face → display rotation. The user requested a global 90° CCW shift
// from the previous mapping; values below already include that offset
// (i.e. each is the old value + 270 mod 360). Initial s_rotation in
// bf_display.c is also pre-set to 270° so splash/brand match before
// any face has committed.
//
// Mapping (clockwise rotation around the display normal):
//   1  Z+ up (display visible from above)                 → 270°
//   2  Z- up (display face hidden, sleep)                 →  (n/a)
//   3  X+ up                                              →   0°
//   4  X- up                                              → 180°
//   5  Y+ up                                              → 270°
//   6  Y- up                                              →  90°
//
// If a face still shows content rotated wrong on your physical
// assembly, swap the value here (0 ↔ 180 or 90 ↔ 270 to reverse).
static int rotation_for_face(int face)
{
    // PCB rev'i değişince ekran tüm yüzlerde 180° baş aşağı kaldı;
    // her satıra +180° (mod 360) ekleyerek toptan flip yapıyoruz.
    switch (face) {
    case 1: return 90;    // Z+ up   (was 270)
    case 3: return 180;   // X+ up   (was 0)
    case 4: return 0;     // X- up   (was 180)
    case 5: return 90;    // Y+ up   (was 270)
    case 6: return 270;   // Y- up   (was 90)
    default: return 90;   //         (was 270)
    }
}

static void on_face_changed_rot(const sk_event_t *evt, void *user)
{
    (void)user;
    if (!evt->payload_json) return;
    // Boot splash/brand penceresinde rotation 180°'ye sabitlendi
    // (render_splash + render_brand idempotent set ediyor). Bu pencerede
    // face.changed fire ederse override etme — idle'a geçince per-face
    // tablosu (rotation_for_face) devreye girer.
    if (s_scene == SCENE_SPLASH || s_scene == SCENE_BRAND) {
        return;
    }
    const char *p = strstr(evt->payload_json, "\"face\"");
    if (!p || !(p = strchr(p, ':'))) return;
    int face = (int)strtol(p + 1, NULL, 10);
    // Z+ (display face up) keeps the previous rotation so a paused
    // countdown stays oriented the way the user last saw it. Z- (face
    // hidden) likewise leaves rotation alone — the OLED is asleep, no
    // visible content to rotate.
    if (face == 1 || face == 2) {
        ESP_LOGI(TAG, "face.changed face=%d → rotation kept", face);
        return;
    }
    int rot = rotation_for_face(face);
    ESP_LOGI(TAG, "face.changed face=%d → rotation=%d°", face, rot);
    bf_display_set_rotation(rot);
}

// Button release → clear the on-screen hold-progress bar.
static void on_button_released(const sk_event_t *evt, void *user)
{
    (void)evt; (void)user;
    if (!s_q) return;
    evt_t up = { .type = EVT_BUTTON_UP };
    xQueueSend(s_q, &up, 0);
}

static void on_button_down(const sk_event_t *evt, void *user)
{
    (void)evt; (void)user;
    if (!s_q) return;
    evt_t e = { .type = EVT_BUTTON_DOWN };
    xQueueSend(s_q, &e, 0);
}

static void on_button_hold(const sk_event_t *evt, void *user)
{
    (void)user;
    if (!s_q || !evt->payload_json) return;
    const char *p = strstr(evt->payload_json, "\"reached_ms\"");
    if (!p) return;
    p = strchr(p, ':');
    if (!p) return;
    int reached = (int)strtol(p + 1, NULL, 10);
    evt_t e = { .type = EVT_BUTTON_HOLD, .i = reached };
    xQueueSend(s_q, &e, 0);
}

// ---------------------------------------------------------------------
// Task — owns the scene timeline and event processing.
// ---------------------------------------------------------------------

static void enter_idle_scene(void)
{
    s_scene = SCENE_IDLE;
    render_current();
}

static void ui_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "ui_task entered, starting splash");

    // Splash → wait → Brand → wait → Idle.
    s_scene = SCENE_SPLASH;
    render_current();
    vTaskDelay(pdMS_TO_TICKS(SPLASH_HOLD_MS));

    s_scene = SCENE_BRAND;
    render_current();
    vTaskDelay(pdMS_TO_TICKS(BRAND_HOLD_MS));

    enter_idle_scene();

    // Steady-state loop: event processing + periodic overlay refresh.
    for (;;) {
        evt_t e;
        if (xQueueReceive(s_q, &e, pdMS_TO_TICKS(50)) == pdTRUE) {
            switch (e.type) {
            case EVT_TIMER_STATE:
                // Switch between IDLE (wordmark) and COUNTDOWN (big MM:SS)
                // depending on whether the timer engine is actively
                // counting. Don't override SPLASH/BRAND/message scenes
                // (BREAK_MSG / API_ERROR own the panel until their
                // deadline expires).
                if (s_scene == SCENE_IDLE || s_scene == SCENE_COUNTDOWN) {
                    s_scene = s_timer_counting ? SCENE_COUNTDOWN : SCENE_IDLE;
                }
                render_current();
                break;
            case EVT_TIMER_TICK:
                s_countdown_sec = e.i;
                if (s_scene == SCENE_COUNTDOWN) render_current();
                break;
            case EVT_SHOW_BREAK:
                s_scene = SCENE_BREAK_MSG;
                s_msg_deadline_us = now_us() + (int64_t)BREAK_MSG_HOLD_MS * 1000;
                render_current();
                break;
            case EVT_SHOW_API_ERR:
                // OOPS overrides Focus Break if both fire close together
                // (deliberate: failure feedback wins over success).
                s_scene = SCENE_API_ERROR;
                s_msg_deadline_us = now_us() + (int64_t)API_ERROR_HOLD_MS * 1000;
                render_current();
                break;
            case EVT_BUTTON_DOWN:
                s_hold_active = true;
                s_hold_reached_ms = 0;
                if (s_scene == SCENE_IDLE) render_current();
                break;
            case EVT_BUTTON_HOLD:
                s_hold_reached_ms = e.i;
                if (s_scene == SCENE_IDLE) render_current();
                break;
            case EVT_BUTTON_UP:
                s_hold_active = false;
                s_hold_reached_ms = 0;
                if (s_scene == SCENE_IDLE) render_current();
                break;
            case EVT_TICK:
                // Forced re-render (e.g. tilt overlay appeared/cleared).
                render_current();
                break;
            }
        }

        // Message-scene timeout: BREAK_MSG / API_ERROR are full-screen
        // overlays that auto-revert after their hold time. Once the
        // deadline passes, drop back into whatever scene matches the
        // current timer state (COUNTDOWN if still running, IDLE otherwise
        // — usually IDLE since BREAK_MSG fires on expired).
        int64_t now = now_us();
        if ((s_scene == SCENE_BREAK_MSG || s_scene == SCENE_API_ERROR) &&
            s_msg_deadline_us != 0 && now >= s_msg_deadline_us) {
            s_msg_deadline_us = 0;
            s_scene = s_timer_counting ? SCENE_COUNTDOWN : SCENE_IDLE;
            render_current();
        }

        // Battery low blink driver — refresh status bar every 500 ms so
        // the icon visibly flashes. Only relevant in COUNTDOWN (the only
        // scene that draws the status bar). Cheaper than polling on
        // every 50 ms tick — we re-render exactly when the blink phase
        // flips.
        if (s_scene == SCENE_COUNTDOWN && s_battery_low &&
            (now - s_battery_blink_last_us) >= BATTERY_BLINK_HALF_MS * 1000) {
            s_battery_blink_last_us = now;
            render_current();
        }
    }
}

// ---------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------

esp_err_t bf_ui_init(void)
{
    if (s_q) return ESP_OK;   // idempotent

    s_q = xQueueCreate(8, sizeof(evt_t));
    if (!s_q) return ESP_ERR_NO_MEM;

    sk_capabilities_register_book("bf_ui", "0.1.0");

    int sub;
    sk_event_bus_subscribe("timer.state",     on_timer_state,     NULL, &sub);
    sk_event_bus_subscribe("timer.tick",      on_timer_tick,      NULL, &sub);
    sk_event_bus_subscribe("button.down",     on_button_down,     NULL, &sub);
    sk_event_bus_subscribe("button.hold",     on_button_hold,     NULL, &sub);
    sk_event_bus_subscribe("face.changed",    on_face_changed_rot,NULL, &sub);
    sk_event_bus_subscribe("wifi.state",      on_wifi_state,      NULL, &sub);
    sk_event_bus_subscribe("ble.state",       on_ble_state,       NULL, &sub);
    sk_event_bus_subscribe("api.sent",        on_api_sent,        NULL, &sub);
    sk_event_bus_subscribe("battery.sample",    on_battery_event, NULL, &sub);
    sk_event_bus_subscribe("battery.low",       on_battery_event, NULL, &sub);
    sk_event_bus_subscribe("battery.critical",  on_battery_event, NULL, &sub);
    sk_event_bus_subscribe("battery.recovered", on_battery_event, NULL, &sub);
    sk_event_bus_subscribe("battery.lockout",   on_battery_lockout, NULL, &sub);
    sk_event_bus_subscribe("face.tilted",       on_face_tilted,   NULL, &sub);
    // Button release clears the on-screen hold-progress bar.
    sk_event_bus_subscribe("button.released", on_button_released, NULL, &sub);

    BaseType_t r = xTaskCreate(ui_task, "bf_ui", 6144, NULL, 4, NULL);
    if (r != pdPASS) {
        vQueueDelete(s_q); s_q = NULL;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "ready (splash %d ms, brand %d ms)",
             SPLASH_HOLD_MS, BRAND_HOLD_MS);
    return ESP_OK;
}
