#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =====================================================================
// bf_display — 1.5" I2C OLED driver (SH1107 128×128 monochrome).
//
// Framebuffer-based: every draw call writes into a 2 KB RAM buffer
// (1 bit per pixel, 8 vertical pixels packed per byte — page-major
// like SSD1306 but with 16 pages). The panel only updates when
// bf_display_present() is called, or automatically by the timer.tick /
// timer.state subscribers.
//
// Subscribes to:
//   timer.tick     → re-render countdown (MM:SS) every second
//   timer.state    → re-render top status line + clear on state change
//   power.state    → enter/leave display sleep
//
// Graceful: if the OLED doesn't ACK on the I2C bus at init, the
// component logs "absent" and every draw call is a no-op. Firmware boots
// fine on a half-assembled breadboard.
//
// =====================================================================

#define BF_DISPLAY_WIDTH       128
#define BF_DISPLAY_HEIGHT      128
#define BF_DISPLAY_FB_BYTES    (BF_DISPLAY_WIDTH * BF_DISPLAY_HEIGHT / 8)  // 2048 (1 bpp)

// Monochrome pixel: 0 = off, anything else = on.
#define BF_PIX_OFF             0
#define BF_PIX_ON              1

esp_err_t bf_display_init(void);
bool      bf_display_present_check(void);   // "is the panel present?"

// Toggle panel sleep (SSD1306 0xAE/0xAF). bf_power calls this for
// SCREEN_OFF / LOW_POWER transitions — instead of cutting VCC.
esp_err_t bf_display_sleep(bool sleep_on);

// Set the display orientation in 90° increments. 0/180 are reconfigured
// in hardware via SH1107 SEG_REMAP + COM_SCAN (instant, free). 90/270
// would require software framebuffer rotation per push, not yet
// implemented — calling with those values logs a warning and falls back
// to the nearest hardware rotation (0 or 180).
esp_err_t bf_display_set_rotation(int degrees);

// Push the in-RAM framebuffer to the OLED via one I2C burst per page.
esp_err_t bf_display_present(void);

// Drawing primitives — all operate on the framebuffer; call present()
// to commit to the panel.
void      bf_display_clear(uint8_t pixel_value);
void      bf_display_pixel(int16_t x, int16_t y, uint8_t pixel_value);
void      bf_display_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h,
                                uint8_t pixel_value);

// 8×16 ASCII text. Renders chars 0x20..0x7E from the built-in bitmap
// font; unprintable chars become spaces. `scale` 1..4 multiplies both
// dimensions (scale=2 → 16×32 per char, scale=3 → 24×48).
void      bf_display_text(int16_t x, int16_t y, const char *str,
                          uint8_t scale, uint8_t pixel_value);

// Convenience: draw a "MM:SS" countdown centered horizontally at
// vertical position `y`. Picks a scale that fits without overflow.
void      bf_display_countdown(int16_t y, int total_seconds);

// 7-segment digit, drawn from primitives, scalable via (w,h). Useful
// for big readable countdown displays. `digit` must be 0..9.
void      bf_display_draw_digit_7seg(int16_t x, int16_t y, int digit,
                                      int16_t w, int16_t h, uint8_t pixel_value);

// Two-dot colon scaled to match 7-seg digits at the same height.
void      bf_display_draw_colon_7seg(int16_t x, int16_t y,
                                      int16_t w, int16_t h, uint8_t pixel_value);

// High-level: render "MM:SS" using 7-seg digits + colon, centered
// horizontally at vertical position `y`. `total_seconds` clamped to
// 0..(99·60+59). `w` is per-digit width, `h` is per-digit height.
void      bf_display_countdown_7seg(int16_t y, int total_seconds,
                                     int16_t digit_w, int16_t digit_h,
                                     uint8_t pixel_value);

#ifdef __cplusplus
}
#endif
