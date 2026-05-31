// =====================================================================
// bf_display — SH1107 128×128 monochrome OLED over I2C (rev 2).
//
// 2 KB framebuffer in RAM (page-major: 16 pages × 128 cols, each byte
// holds 8 vertical pixels with LSB at top — same convention as SSD1306).
//
// bf_display_present() pushes the whole frame page-by-page (16 pages ×
// 128 data bytes + 1 control byte per page).
//
// I2C bus init is attempted here; if a sibling component (bf_lis3dsh)
// already installed the driver, the install error is silently ignored —
// the bus is shared.
// =====================================================================

#include "bf_display.h"

#include <stdio.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pins.h"
#include "sk_capabilities.h"

static const char *TAG = "bf_display";

// ---------------------------------------------------------------------
// I2C / SSD1327 constants
// ---------------------------------------------------------------------

#define I2C_PORT          I2C_NUM_0
#define I2C_FREQ_HZ       400000
// 200 ms — full framebuffer push (16 pages × ~129 bytes) takes ~80 ms at
// 400 kHz I2C; 50 ms was too tight, transactions got cut off mid-page.
#define I2C_TIMEOUT_TICKS pdMS_TO_TICKS(200)
#define OLED_ADDR         I2C_ADDR_OLED

// SH1107 control byte prefixes (Co=0; D/C# in bit 6).
#define CTRL_CMD          0x00
#define CTRL_DATA         0x40

// SH1107 commands (subset we use).
#define SH_DISP_OFF       0xAE
#define SH_DISP_ON        0xAF
#define SH_DISP_START     0xDC   // followed by 0x00..0x7F (display start line)
#define SH_CONTRAST       0x81   // followed by 0x00..0xFF
#define SH_MEM_MODE       0x20   // 0x20 = page mode, 0x21 = vertical
#define SH_SEG_REMAP      0xA0   // 0xA0 = normal, 0xA1 = remap
#define SH_DISP_NORMAL    0xA4   // entire display normal (use RAM)
#define SH_NORMAL_DISP    0xA6   // 0xA6 = normal, 0xA7 = inverse
#define SH_MUX_RATIO      0xA8   // followed by N-1 (0x7F = 128)
#define SH_CHARGE_PUMP    0xAD   // followed by 0x8A (enable internal)
#define SH_COM_SCAN_NORM  0xC0   // 0xC0 = COM0→COM[N-1], 0xC8 = reverse
#define SH_DISP_OFFSET    0xD3   // followed by 0x00..0x7F
#define SH_CLOCK_DIV      0xD5   // followed by 0xXY (Y=div, X=osc freq)
#define SH_PRECHARGE      0xD9
#define SH_VCOMH          0xDB
#define SH_PAGE_ADDR      0xB0   // OR'd with page (0..15)
#define SH_COL_LOW        0x00   // OR'd with col low nibble
#define SH_COL_HIGH       0x10   // OR'd with col high nibble

// ---------------------------------------------------------------------
// Framebuffer (2 KB — 16 pages × 128 columns; 1 bpp, LSB at top)
// ---------------------------------------------------------------------

static uint8_t s_fb[BF_DISPLAY_FB_BYTES];
static bool    s_present  = false;
static bool    s_inited   = false;

// Active rotation in degrees (0/90/180/270). Applied in bf_display_pixel
// — see the comment above bf_display_set_rotation for the convention.
// Default 270° (= 90° CCW from the panel's native orientation) so that
// the splash/brand scenes already match the cube's intended viewing
// angle before any face.changed event has been processed.
static int     s_rotation = 270;

// ---------------------------------------------------------------------
// 5×7 ASCII font (chars 0x20..0x7F). Each char = 5 column bytes,
// LSB at top. Classic Adafruit GFX 5×7 font (public domain).
// ---------------------------------------------------------------------

#define FONT_W 5
#define FONT_H 7
#define FONT_FIRST_CHAR 0x20
#define FONT_LAST_CHAR  0x7F

static const uint8_t s_font5x7[(FONT_LAST_CHAR - FONT_FIRST_CHAR + 1)][FONT_W] = {
    {0x00,0x00,0x00,0x00,0x00}, // ' '
    {0x00,0x00,0x5F,0x00,0x00}, // '!'
    {0x00,0x07,0x00,0x07,0x00}, // '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // '$'
    {0x23,0x13,0x08,0x64,0x62}, // '%'
    {0x36,0x49,0x55,0x22,0x50}, // '&'
    {0x00,0x05,0x03,0x00,0x00}, // '\''
    {0x00,0x1C,0x22,0x41,0x00}, // '('
    {0x00,0x41,0x22,0x1C,0x00}, // ')'
    {0x14,0x08,0x3E,0x08,0x14}, // '*'
    {0x08,0x08,0x3E,0x08,0x08}, // '+'
    {0x00,0x50,0x30,0x00,0x00}, // ','
    {0x08,0x08,0x08,0x08,0x08}, // '-'
    {0x00,0x60,0x60,0x00,0x00}, // '.'
    {0x20,0x10,0x08,0x04,0x02}, // '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // '0'
    {0x00,0x42,0x7F,0x40,0x00}, // '1'
    {0x42,0x61,0x51,0x49,0x46}, // '2'
    {0x21,0x41,0x45,0x4B,0x31}, // '3'
    {0x18,0x14,0x12,0x7F,0x10}, // '4'
    {0x27,0x45,0x45,0x45,0x39}, // '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // '6'
    {0x01,0x71,0x09,0x05,0x03}, // '7'
    {0x36,0x49,0x49,0x49,0x36}, // '8'
    {0x06,0x49,0x49,0x29,0x1E}, // '9'
    {0x00,0x36,0x36,0x00,0x00}, // ':'
    {0x00,0x56,0x36,0x00,0x00}, // ';'
    {0x08,0x14,0x22,0x41,0x00}, // '<'
    {0x14,0x14,0x14,0x14,0x14}, // '='
    {0x00,0x41,0x22,0x14,0x08}, // '>'
    {0x02,0x01,0x51,0x09,0x06}, // '?'
    {0x32,0x49,0x79,0x41,0x3E}, // '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 'F'
    {0x3E,0x41,0x49,0x49,0x7A}, // 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 'L'
    {0x7F,0x02,0x0C,0x02,0x7F}, // 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 'R'
    {0x46,0x49,0x49,0x49,0x31}, // 'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 'V'
    {0x3F,0x40,0x38,0x40,0x3F}, // 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 'X'
    {0x07,0x08,0x70,0x08,0x07}, // 'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 'Z'
    {0x00,0x7F,0x41,0x41,0x00}, // '['
    {0x02,0x04,0x08,0x10,0x20}, // '\\'
    {0x00,0x41,0x41,0x7F,0x00}, // ']'
    {0x04,0x02,0x01,0x02,0x04}, // '^'
    {0x40,0x40,0x40,0x40,0x40}, // '_'
    {0x00,0x01,0x02,0x04,0x00}, // '`'
    {0x20,0x54,0x54,0x54,0x78}, // 'a'
    {0x7F,0x48,0x44,0x44,0x38}, // 'b'
    {0x38,0x44,0x44,0x44,0x20}, // 'c'
    {0x38,0x44,0x44,0x48,0x7F}, // 'd'
    {0x38,0x54,0x54,0x54,0x18}, // 'e'
    {0x08,0x7E,0x09,0x01,0x02}, // 'f'
    {0x0C,0x52,0x52,0x52,0x3E}, // 'g'
    {0x7F,0x08,0x04,0x04,0x78}, // 'h'
    {0x00,0x44,0x7D,0x40,0x00}, // 'i'
    {0x20,0x40,0x44,0x3D,0x00}, // 'j'
    {0x7F,0x10,0x28,0x44,0x00}, // 'k'
    {0x00,0x41,0x7F,0x40,0x00}, // 'l'
    {0x7C,0x04,0x18,0x04,0x78}, // 'm'
    {0x7C,0x08,0x04,0x04,0x78}, // 'n'
    {0x38,0x44,0x44,0x44,0x38}, // 'o'
    {0x7C,0x14,0x14,0x14,0x08}, // 'p'
    {0x08,0x14,0x14,0x18,0x7C}, // 'q'
    {0x7C,0x08,0x04,0x04,0x08}, // 'r'
    {0x48,0x54,0x54,0x54,0x20}, // 's'
    {0x04,0x3F,0x44,0x40,0x20}, // 't'
    {0x3C,0x40,0x40,0x20,0x7C}, // 'u'
    {0x1C,0x20,0x40,0x20,0x1C}, // 'v'
    {0x3C,0x40,0x30,0x40,0x3C}, // 'w'
    {0x44,0x28,0x10,0x28,0x44}, // 'x'
    {0x0C,0x50,0x50,0x50,0x3C}, // 'y'
    {0x44,0x64,0x54,0x4C,0x44}, // 'z'
    {0x00,0x08,0x36,0x41,0x00}, // '{'
    {0x00,0x00,0x7F,0x00,0x00}, // '|'
    {0x00,0x41,0x36,0x08,0x00}, // '}'
    {0x08,0x08,0x2A,0x1C,0x08}, // '~'
    {0x00,0x00,0x00,0x00,0x00}, // 0x7F (DEL — blank)
};

// ---------------------------------------------------------------------
// I2C helpers
// ---------------------------------------------------------------------

static void i2c_bus_init_once(void)
{
    static bool s_bus_inited = false;
    if (s_bus_inited) return;
    s_bus_inited = true;

    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_I2C_SDA,
        .scl_io_num       = PIN_I2C_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    i2c_param_config(I2C_PORT, &cfg);
    // Sibling component may have installed already — that's fine.
    (void)i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

static esp_err_t oled_send(uint8_t ctrl, const uint8_t *data, size_t len)
{
    uint8_t buf[1 + 32];
    if (len + 1 > sizeof(buf)) return ESP_ERR_INVALID_SIZE;
    buf[0] = ctrl;
    memcpy(buf + 1, data, len);
    return i2c_master_write_to_device(I2C_PORT, OLED_ADDR, buf, len + 1,
                                       I2C_TIMEOUT_TICKS);
}

static esp_err_t oled_cmd1(uint8_t c) { return oled_send(CTRL_CMD, &c, 1); }

static esp_err_t oled_cmd2(uint8_t c1, uint8_t c2)
{
    uint8_t b[2] = { c1, c2 };
    return oled_send(CTRL_CMD, b, 2);
}

// Push the whole framebuffer. SH1107 page-mode addressing: for each
// page (0..15), set page address + column-low + column-high, then
// stream 128 data bytes for that page.
static esp_err_t oled_push_fb(void)
{
    esp_err_t err;
    for (int page = 0; page < 16; page++) {
        // Three commands: page addr, low column nibble, high column nibble.
        uint8_t setpage[3] = {
            (uint8_t)(SH_PAGE_ADDR | (page & 0x0F)),
            (uint8_t)(SH_COL_LOW   | 0x00),
            (uint8_t)(SH_COL_HIGH  | 0x00),
        };
        err = oled_send(CTRL_CMD, setpage, 3);
        if (err != ESP_OK) return err;

        uint8_t buf[1 + 128];
        buf[0] = CTRL_DATA;
        memcpy(buf + 1, &s_fb[page * BF_DISPLAY_WIDTH], BF_DISPLAY_WIDTH);
        err = i2c_master_write_to_device(I2C_PORT, OLED_ADDR, buf, sizeof(buf),
                                          I2C_TIMEOUT_TICKS);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

// ---------------------------------------------------------------------
// Init sequence — SH1107 128×128. Values are the well-tested defaults
// for generic GME1.5"/GME12812 modules; if your panel comes out flipped
// or shifted, tweak SEG_REMAP / COM_SCAN_NORM / DISP_OFFSET below.
// ---------------------------------------------------------------------

static esp_err_t oled_init_sequence(void)
{
    if (oled_cmd1(SH_DISP_OFF)              != ESP_OK) return ESP_FAIL;
    if (oled_cmd2(SH_DISP_START,    0x00)   != ESP_OK) return ESP_FAIL;  // start at line 0
    if (oled_cmd2(SH_CONTRAST,      0x80)   != ESP_OK) return ESP_FAIL;  // mid contrast
    if (oled_cmd2(SH_MEM_MODE,      0x00)   != ESP_OK) return ESP_FAIL;  // page mode
    if (oled_cmd1(SH_SEG_REMAP)             != ESP_OK) return ESP_FAIL;  // 0xA0 normal
    if (oled_cmd1(SH_COM_SCAN_NORM)         != ESP_OK) return ESP_FAIL;  // 0xC0 normal
    if (oled_cmd2(SH_MUX_RATIO,     0x7F)   != ESP_OK) return ESP_FAIL;  // MUX = 128
    // Display offset 0x00 is correct for the GME12812 SH1107 modules in
    // hand. Other variants may need 0x60 — symptom of a wrong value is
    // either vertical clipping (too small) or horizontal wrap-around
    // (too large), and we've now empirically confirmed 0x00 here.
    if (oled_cmd2(SH_DISP_OFFSET,   0x00)   != ESP_OK) return ESP_FAIL;
    if (oled_cmd2(SH_CLOCK_DIV,     0x51)   != ESP_OK) return ESP_FAIL;
    if (oled_cmd2(SH_PRECHARGE,     0x22)   != ESP_OK) return ESP_FAIL;
    if (oled_cmd2(SH_VCOMH,         0x35)   != ESP_OK) return ESP_FAIL;
    if (oled_cmd2(SH_CHARGE_PUMP,   0x8A)   != ESP_OK) return ESP_FAIL;  // internal pump ON
    if (oled_cmd1(SH_DISP_NORMAL)           != ESP_OK) return ESP_FAIL;  // use RAM
    if (oled_cmd1(SH_NORMAL_DISP)           != ESP_OK) return ESP_FAIL;  // not inverted
    return oled_cmd1(SH_DISP_ON);
}

// ---------------------------------------------------------------------
// Framebuffer drawing primitives
// ---------------------------------------------------------------------

void bf_display_clear(uint8_t pixel_value)
{
    memset(s_fb, pixel_value ? 0xFF : 0x00, sizeof(s_fb));
}

void bf_display_pixel(int16_t x, int16_t y, uint8_t pixel_value)
{
    if (x < 0 || y < 0 || x >= BF_DISPLAY_WIDTH || y >= BF_DISPLAY_HEIGHT) return;

    // Apply current rotation. Panel is square (128x128) so axes don't
    // need to swap dimensions, only re-map within the same range.
    int16_t tx, ty;
    switch (s_rotation) {
    case 90:
        tx = (int16_t)(BF_DISPLAY_HEIGHT - 1 - y);
        ty = x;
        break;
    case 180:
        tx = (int16_t)(BF_DISPLAY_WIDTH  - 1 - x);
        ty = (int16_t)(BF_DISPLAY_HEIGHT - 1 - y);
        break;
    case 270:
        tx = y;
        ty = (int16_t)(BF_DISPLAY_WIDTH - 1 - x);
        break;
    case 0:
    default:
        tx = x;
        ty = y;
        break;
    }

    int     page = ty >> 3;
    uint8_t bit  = (uint8_t)(1u << (ty & 7));
    uint8_t *cell = &s_fb[page * BF_DISPLAY_WIDTH + tx];
    if (pixel_value) *cell |=  bit;
    else             *cell &= (uint8_t)~bit;
}

void bf_display_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h,
                          uint8_t pixel_value)
{
    if (w <= 0 || h <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > BF_DISPLAY_WIDTH)  w = BF_DISPLAY_WIDTH  - x;
    if (y + h > BF_DISPLAY_HEIGHT) h = BF_DISPLAY_HEIGHT - y;
    if (w <= 0 || h <= 0) return;

    for (int16_t yy = y; yy < y + h; yy++) {
        for (int16_t xx = x; xx < x + w; xx++) {
            bf_display_pixel(xx, yy, pixel_value);
        }
    }
}

// Render one ASCII char at (x,y) at integer scale.
static void draw_char(int16_t x, int16_t y, char c, uint8_t scale, uint8_t pix)
{
    if (scale < 1) scale = 1;
    if (c < FONT_FIRST_CHAR || c > FONT_LAST_CHAR) c = ' ';
    const uint8_t *glyph = s_font5x7[c - FONT_FIRST_CHAR];

    for (int col = 0; col < FONT_W; col++) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < FONT_H; row++) {
            if (bits & (1 << row)) {
                bf_display_fill_rect(x + col * scale, y + row * scale,
                                     scale, scale, pix);
            }
        }
    }
}

void bf_display_text(int16_t x, int16_t y, const char *str,
                     uint8_t scale, uint8_t pixel_value)
{
    if (!str) return;
    if (scale < 1) scale = 1;
    int16_t cx = x;
    int16_t advance = (FONT_W + 1) * scale;
    while (*str) {
        draw_char(cx, y, *str++, scale, pixel_value);
        cx += advance;
    }
}

// ---------------------------------------------------------------------
// 7-segment digit renderer — scalable from primitives
// ---------------------------------------------------------------------

// Standard 7-segment patterns: bits 0..6 = a,b,c,d,e,f,g.
//   a = top, b = top-right, c = bottom-right, d = bottom,
//   e = bottom-left, f = top-left, g = middle.
static const uint8_t s_seg_pattern[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66,    // 0,1,2,3,4
    0x6D, 0x7D, 0x07, 0x7F, 0x6F,    // 5,6,7,8,9
};

// Internal: a "fat" horizontal segment with chamfered ends, sized to
// fit width=`w` thickness=`t`.
static void seg_horiz(int16_t x, int16_t y, int16_t w, int16_t t, uint8_t v)
{
    if (w < 2 || t < 1) return;
    int half = t / 2;
    // Diamond shape: each row is wider in the middle, narrower at top/bottom.
    for (int r = 0; r < t; r++) {
        int inset = (r <= half) ? (half - r) : (r - half);
        bf_display_fill_rect(x + inset, y + r, w - 2 * inset, 1, v);
    }
}

// Internal: a "fat" vertical segment with chamfered ends.
static void seg_vert(int16_t x, int16_t y, int16_t h, int16_t t, uint8_t v)
{
    if (h < 2 || t < 1) return;
    int half = t / 2;
    for (int c = 0; c < t; c++) {
        int inset = (c <= half) ? (half - c) : (c - half);
        bf_display_fill_rect(x + c, y + inset, 1, h - 2 * inset, v);
    }
}

void bf_display_draw_digit_7seg(int16_t x, int16_t y, int digit,
                                int16_t w, int16_t h, uint8_t v)
{
    if (digit < 0 || digit > 9) return;
    if (w < 6 || h < 8) return;

    int t = w / 6;                 // segment thickness — ~16% of width
    if (t < 2) t = 2;
    int sw = w - 2 * t;            // length of horizontal segments
    int sh = (h - 3 * t) / 2;      // length of vertical segments
    if (sw < 1) sw = 1;
    if (sh < 1) sh = 1;

    uint8_t p = s_seg_pattern[digit];

    // a (top)
    if (p & 0x01) seg_horiz(x + t, y, sw, t, v);
    // b (top-right)
    if (p & 0x02) seg_vert(x + w - t, y + t, sh, t, v);
    // c (bottom-right)
    if (p & 0x04) seg_vert(x + w - t, y + 2 * t + sh, sh, t, v);
    // d (bottom)
    if (p & 0x08) seg_horiz(x + t, y + h - t, sw, t, v);
    // e (bottom-left)
    if (p & 0x10) seg_vert(x, y + 2 * t + sh, sh, t, v);
    // f (top-left)
    if (p & 0x20) seg_vert(x, y + t, sh, t, v);
    // g (middle)
    if (p & 0x40) seg_horiz(x + t, y + t + sh, sw, t, v);
}

void bf_display_draw_colon_7seg(int16_t x, int16_t y,
                                int16_t w, int16_t h, uint8_t v)
{
    int t = w / 6;
    if (t < 2) t = 2;
    int dot = t * 2;
    if (dot < 4) dot = 4;
    // Two dots at 1/3 and 2/3 of the digit height.
    bf_display_fill_rect(x + (w - dot) / 2, y + h / 3 - dot / 2,
                         dot, dot, v);
    bf_display_fill_rect(x + (w - dot) / 2, y + (2 * h) / 3 - dot / 2,
                         dot, dot, v);
}

void bf_display_countdown_7seg(int16_t y, int total_seconds,
                               int16_t digit_w, int16_t digit_h,
                               uint8_t v)
{
    if (total_seconds < 0) total_seconds = 0;
    if (total_seconds > 99 * 60 + 59) total_seconds = 99 * 60 + 59;
    int mm = total_seconds / 60;
    int ss = total_seconds % 60;

    int colon_w = digit_w / 2;
    int gap = digit_w / 8;
    if (gap < 1) gap = 1;
    // total = 4 digits + 1 colon + 4 gaps
    int total_w = 4 * digit_w + colon_w + 4 * gap;
    int x = (BF_DISPLAY_WIDTH - total_w) / 2;

    // Erase the band first (digit_h tall + a couple px above/below).
    bf_display_fill_rect(0, y - 1, BF_DISPLAY_WIDTH, digit_h + 2, BF_PIX_OFF);

    int cx = x;
    bf_display_draw_digit_7seg(cx, y, mm / 10, digit_w, digit_h, v);
    cx += digit_w + gap;
    bf_display_draw_digit_7seg(cx, y, mm % 10, digit_w, digit_h, v);
    cx += digit_w + gap;
    bf_display_draw_colon_7seg(cx, y, colon_w, digit_h, v);
    cx += colon_w + gap;
    bf_display_draw_digit_7seg(cx, y, ss / 10, digit_w, digit_h, v);
    cx += digit_w + gap;
    bf_display_draw_digit_7seg(cx, y, ss % 10, digit_w, digit_h, v);
}

void bf_display_countdown(int16_t y, int total_seconds)
{
    if (total_seconds < 0) total_seconds = 0;
    if (total_seconds > 99 * 60 + 59) total_seconds = 99 * 60 + 59;
    int mm = total_seconds / 60;
    int ss = total_seconds % 60;
    char buf[8];
    snprintf(buf, sizeof(buf), "%02d:%02d", mm, ss);

    // 5 chars × (5+1) × scale ≤ 128. scale=4 → 120 px wide, fits with margin.
    uint8_t scale = 4;
    int16_t advance = (FONT_W + 1) * scale;
    int16_t total_w = advance * 5 - scale;
    int16_t x = (BF_DISPLAY_WIDTH - total_w) / 2;

    bf_display_fill_rect(0, y - 1, BF_DISPLAY_WIDTH,
                         FONT_H * scale + 2, BF_PIX_OFF);
    bf_display_text(x, y, buf, scale, BF_PIX_ON);
}

// ---------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------

bool bf_display_present_check(void) { return s_present; }

esp_err_t bf_display_present(void)
{
    if (!s_present) return ESP_OK;
    return oled_push_fb();
}

esp_err_t bf_display_sleep(bool sleep_on)
{
    if (!s_present) return ESP_OK;
    return oled_cmd1(sleep_on ? SH_DISP_OFF : SH_DISP_ON);
}

// All 4 rotations are done in software — every bf_display_pixel call
// transforms (x, y) before writing to the framebuffer. SH1107 stays in
// the default 0° SEG/COM config. This keeps the math in one place and
// means the panel sees a consistent layout regardless of orientation.
//
// Rotation convention (clockwise around the panel's own face):
//   0   → identity
//   90  → (x, y) becomes (H-1-y, x) — top of content moves to right
//   180 → (x, y) becomes (W-1-x, H-1-y)
//   270 → (x, y) becomes (y, W-1-x) — top of content moves to left
esp_err_t bf_display_set_rotation(int degrees)
{
    int d = ((degrees % 360) + 360) % 360;
    if (d != 0 && d != 90 && d != 180 && d != 270) {
        ESP_LOGW(TAG, "rotation %d not aligned to 90° step — clamping to 0°", degrees);
        d = 0;
    }
    int prev = s_rotation;
    s_rotation = d;
    ESP_LOGD(TAG, "rotation %d° → %d°", prev, d);
    return ESP_OK;
}

// ---------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------
//
// Note: bf_display is a pure driver — it owns the panel and the
// framebuffer, but does NOT subscribe to events. All scene/state-driven
// rendering lives in bf_ui. Keeping this separation is what lets the
// driver be reused unchanged on other SmartKraft devices with their
// own UI components.

esp_err_t bf_display_init(void)
{
    if (s_inited) return ESP_OK;
    s_inited = true;

    i2c_bus_init_once();

    // Probe by trying to send display-off. SH1107 ACKs even before init.
    esp_err_t err = oled_cmd1(SH_DISP_OFF);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "OLED not detected at 0x%02X (%s) — display disabled, "
                      "firmware will run without it",
                 OLED_ADDR, esp_err_to_name(err));
        return ESP_OK;
    }

    err = oled_init_sequence();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init sequence failed: %s", esp_err_to_name(err));
        return ESP_OK;
    }

    s_present = true;
    bf_display_clear(BF_PIX_OFF);
    bf_display_present();

    sk_capabilities_register_book("bf_display", "0.2.3");   // SH1107, no UI hooks
    ESP_LOGI(TAG, "ready (SH1107 128x128 mono @ 0x%02X, fb=%d B)",
             OLED_ADDR, BF_DISPLAY_FB_BYTES);
    return ESP_OK;
}
