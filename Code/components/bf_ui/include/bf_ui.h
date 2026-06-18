#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =====================================================================
// bf_ui — high-level scene coordinator on top of bf_display.
//
// Owns the visual identity of the device:
//   - boot splash (logo + "SmartKraft")
//   - brand title ("BLOKING" / "FOCUS")
//   - idle screen: the "BLOKING" / "FOCUS" wordmark shown while the cube
//     rests on its display face with no timer running
//   - countdown rendering (big MM:SS, driven by timer.tick events)
//   - critical overlays: Focus Break splash, API error, low-battery lockout
//
// Subscribes to:
//   timer.state   → switch between idle and countdown scenes
//   timer.tick    → countdown remaining seconds
//   face.changed  → display rotation per cube face
//   wifi/ble/battery/api.sent → status-bar + overlay state
//
// Public API is intentionally narrow — bf_ui is the only thing that
// should drive the panel; other components emit events.
// =====================================================================

esp_err_t bf_ui_init(void);

#ifdef __cplusplus
}
#endif
