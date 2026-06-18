#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =====================================================================
// bf_timer_engine — countdown state machine for Blocking Focus.
//
// Implements docs/state_machine.md. Subscribes to face.changed (from
// bf_face_detector), runs a 1 Hz countdown tick, fires every configured
// sk_api endpoint when the timer expires, then enters a 10 s cooldown.
//
// Events on sk_event_bus:
//   timer.state          {"state":"...","remaining_sec":N,"face":N}
//                        — emitted on every state transition
//   timer.tick           {"remaining_sec":N}
//                        — emitted every second while counting (UI use)
//   timer.expired        {"face":N,"duration_sec":N}
//                        — emitted at countdown=0, before API trigger
//
// CLI: timer.status / timer.cancel are defined but intentionally NOT
// registered (cancellation is physical-only, by product decision).
//
// Face → duration mapping is project-tunable. Default in
// bf_timer_engine.c maps the timer faces to 5/15/30 min, with the Y-down
// face set to 15 s as a short test value; the display face arms the next
// timer; the USB face is handled as idle (BF_TIMER_LOW_POWER is currently
// unreachable, see the NOTE in bf_timer_engine.c).
// =====================================================================

typedef enum {
    BF_TIMER_BOOT = 0,
    BF_TIMER_IDLE_NEEDS_ARM,
    BF_TIMER_ARMED,
    BF_TIMER_RESETTABLE,            // counting, remaining > 60 s
    BF_TIMER_LOCKED,                // counting, remaining <= 60 s — face changes ignored
    BF_TIMER_PAUSED,                // display face up while > 60 s remaining
    BF_TIMER_EXPIRED,
    BF_TIMER_API_TRIGGERING,
    BF_TIMER_COOLDOWN,              // 10 s wait after API
    BF_TIMER_COOLDOWN_NEEDS_FACE_CHG,
    BF_TIMER_LOW_POWER,             // USB face up
} bf_timer_state_t;

esp_err_t          bf_timer_engine_init(void);
bf_timer_state_t   bf_timer_engine_state(void);
const char        *bf_timer_engine_state_str(void);

// Remaining countdown seconds. 0 if no timer is active.
int                bf_timer_engine_remaining_sec(void);

// Configured duration for a face. Returns:
//    > 0  : seconds for a timer face
//    = 0  : display face (no timer, just arms)
//   = -1  : USB face (low-power)
//   = -2  : invalid face number
int                bf_timer_engine_face_duration(int face);

#ifdef __cplusplus
}
#endif
