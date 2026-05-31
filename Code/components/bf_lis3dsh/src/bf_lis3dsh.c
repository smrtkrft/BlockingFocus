// =====================================================================
// bf_lis3dsh — accelerometer driver (multi-chip aware).
//
// Production target (2026-05-08): ST LIS3DH at I2C 0x18 (SDO=GND).
//
// Legacy: ST LIS3DSH at I2C 0x1E (probed if LIS3DH absent — original
// hardware revision and any breadboards still using the old chip).
//
// Prototype fallback: InvenSense MPU-6050 / MPU-9250 at 0x68. The user
// often has one of these on hand before the real accelerometer ships,
// and using the same software API lets us validate the cube focus loop
// (face_detector → timer_engine) end-to-end without firmware changes
// when the production sensor does arrive.
//
// Init flow:
//   1) Probe LIS3DH WHO_AM_I (0x0F at 0x18, expect 0x33).
//   2) Else probe LIS3DSH WHO_AM_I (0x0F at 0x1E, expect 0x3F).
//   3) Else probe MPU at 0x68 (WHO_AM_I 0x75, expect 0x68 or 0x71).
//   4) Configure whichever chip is found, store its kind.
//   5) bf_lis3dsh_read() branches on chip kind to use the right register
//      layout and byte order. All three chips are configured at ±2 g,
//      giving the same 16384 LSB/g sensitivity at 16-bit alignment, so
//      bf_lis3dsh_read_g uses one constant.
//
// Note on LIS3DH 12-bit data: the chip returns 12-bit accel left-justified
// in 16-bit MSB:LSB pairs (high-resolution mode). After reading we leave
// the value untouched — the same int16_t carries it; the LSB-per-g still
// works out to 16384 because of the left-justification. Identical math
// to LIS3DSH end-to-end.
// =====================================================================

#include "bf_lis3dsh.h"

#include <stdio.h>

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pins.h"
#include "sk_capabilities.h"
#include "sk_cli.h"
#include "sk_errors.h"

static const char *TAG = "bf_lis3dsh";

#define I2C_PORT          I2C_NUM_0
#define I2C_FREQ_HZ       400000
#define I2C_TIMEOUT_TICKS pdMS_TO_TICKS(50)

// ---------------------------------------------------------------------
// Chip-specific constants
// ---------------------------------------------------------------------

// LIS3DH (production, 2026-05-08+)
#define LIS3DH_ADDR           I2C_ADDR_LIS3DH      // 0x18 (SDO=GND)
#define LIS3DH_WHO_EXPECTED   0x33
#define LIS3DH_CTRL_REG1      0x20   // ODR | LPen | Zen Yen Xen
#define LIS3DH_CTRL_REG4      0x23   // BDU | BLE | FS1 FS0 | HR | ST | SIM

// LIS3DSH (legacy)
#define LIS_ADDR              I2C_ADDR_LIS3DSH    // 0x1E
#define LIS_WHO               0x0F
#define LIS_WHO_EXPECTED      0x3F
#define LIS_CTRL_REG4         0x20
#define LIS_CTRL_REG5         0x24
#define LIS_OUT_X_L           0x28
#define LIS_AUTO_INC          0x80                // OR with subaddr for multi-byte read

#define MPU_ADDR              0x68                // MPU-6050 / 9250 with AD0=GND
#define MPU_WHO               0x75
#define MPU_WHO_6050          0x68
#define MPU_WHO_9250          0x71
#define MPU_PWR_MGMT_1        0x6B
#define MPU_ACCEL_CONFIG      0x1C
#define MPU_ACCEL_XOUT_H      0x3B

// Both chips at ±2 g full-scale → 16384 LSB/g.
#define LSB_PER_G             16384.0f

// ---------------------------------------------------------------------
// State
// ---------------------------------------------------------------------

typedef enum {
    IMU_NONE = 0,
    IMU_LIS3DH,
    IMU_LIS3DSH,
    IMU_MPU6050,
    IMU_MPU9250,
} imu_kind_t;

static imu_kind_t s_kind    = IMU_NONE;
static uint8_t    s_addr    = 0;       // I2C address of the active chip
static bool       s_present = false;

// ---------------------------------------------------------------------
// Bus + I2C helpers
// ---------------------------------------------------------------------

// I2C bus driver bf_display tarafından zaten install edilmiş (init
// sırasında bf_display önce çağrılıyor — main.c). Burada tekrar
// install denemek IDF'in `i2c driver install error` log'u atmasına
// sebep oluyor (zararsız ama gürültü). Bus konfigürasyonuna gerek
// yok; biz sadece okuma/yazma helper'ları ile bus'ı kullanıyoruz.
static void i2c_bus_init_once(void)
{
    /* no-op — bus owned by bf_display */
}

static esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t pkt[2] = { reg, val };
    return i2c_master_write_to_device(I2C_PORT, addr, pkt, 2,
                                       I2C_TIMEOUT_TICKS);
}

static esp_err_t i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *out, size_t n)
{
    return i2c_master_write_read_device(I2C_PORT, addr, &reg, 1,
                                         out, n, I2C_TIMEOUT_TICKS);
}

// ---------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------

bool bf_lis3dsh_present(void) { return s_present; }

esp_err_t bf_lis3dsh_read(int16_t *x, int16_t *y, int16_t *z)
{
    if (!s_present) return ESP_ERR_NOT_FOUND;
    if (!x || !y || !z) return ESP_ERR_INVALID_ARG;

    uint8_t buf[6];

    if (s_kind == IMU_LIS3DH || s_kind == IMU_LIS3DSH) {
        // Same OUT_X_L register (0x28) and same auto-increment bit (MSB
        // of subaddr) on both ST chips. Both little-endian.
        esp_err_t err = i2c_read_regs(s_addr, LIS_OUT_X_L | LIS_AUTO_INC,
                                       buf, sizeof(buf));
        if (err != ESP_OK) return err;
        *x = (int16_t)(buf[0] | (buf[1] << 8));
        *y = (int16_t)(buf[2] | (buf[3] << 8));
        *z = (int16_t)(buf[4] | (buf[5] << 8));
    } else {
        // MPU-6050 / MPU-9250 — same accel register layout (datasheet
        // section 4 of MPU-6050 RM, identical bytes on MPU-9250).
        esp_err_t err = i2c_read_regs(s_addr, MPU_ACCEL_XOUT_H,
                                       buf, sizeof(buf));
        if (err != ESP_OK) return err;
        // Big-endian on MPU.
        *x = (int16_t)((buf[0] << 8) | buf[1]);
        *y = (int16_t)((buf[2] << 8) | buf[3]);
        *z = (int16_t)((buf[4] << 8) | buf[5]);
    }
    return ESP_OK;
}

esp_err_t bf_lis3dsh_read_g(float *x, float *y, float *z)
{
    int16_t rx, ry, rz;
    esp_err_t err = bf_lis3dsh_read(&rx, &ry, &rz);
    if (err != ESP_OK) return err;
    if (x) *x = (float)rx / LSB_PER_G;
    if (y) *y = (float)ry / LSB_PER_G;
    if (z) *z = (float)rz / LSB_PER_G;
    return ESP_OK;
}

// ---------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------

static const char *kind_str(imu_kind_t k)
{
    switch (k) {
    case IMU_LIS3DH:  return "LIS3DH";
    case IMU_LIS3DSH: return "LIS3DSH";
    case IMU_MPU6050: return "MPU-6050";
    case IMU_MPU9250: return "MPU-9250";
    default:          return "none";
    }
}

static sk_err_t cmd_accel_read(sk_cli_ctx_t *ctx)
{
    if (!s_present) {
        sk_cli_err(ctx, SK_ERR_NOT_FOUND, "{\"reason\":\"sensor_absent\"}");
        return SK_OK;
    }
    int16_t x, y, z;
    esp_err_t err = bf_lis3dsh_read(&x, &y, &z);
    if (err != ESP_OK) {
        sk_cli_err(ctx, SK_ERR_INTERNAL, NULL);
        return SK_OK;
    }
    char buf[192];
    snprintf(buf, sizeof(buf),
             "{\"chip\":\"%s\",\"addr\":\"0x%02X\","
             "\"raw\":{\"x\":%d,\"y\":%d,\"z\":%d},"
             "\"g\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f}}",
             kind_str(s_kind), s_addr,
             x, y, z,
             (double)x / LSB_PER_G,
             (double)y / LSB_PER_G,
             (double)z / LSB_PER_G);
    sk_cli_ok(ctx, buf);
    return SK_OK;
}

static const sk_cli_command_t s_cmd_accel = {
    .name    = "accel.read",
    .summary = "One-shot accelerometer read (LIS3DH/LIS3DSH/MPU)",
    .usage   = "accel read",
    .handler = cmd_accel_read,
};

// ---------------------------------------------------------------------
// Probes
// ---------------------------------------------------------------------

static bool probe_lis3dh(void)
{
    uint8_t who = 0;
    if (i2c_read_regs(LIS3DH_ADDR, LIS_WHO, &who, 1) != ESP_OK) return false;
    if (who != LIS3DH_WHO_EXPECTED) return false;

    // CTRL_REG1 (0x20): ODR=0100 (50 Hz, normal mode), LPen=0, all 3 axes on.
    if (i2c_write_reg(LIS3DH_ADDR, LIS3DH_CTRL_REG1, 0x47) != ESP_OK) return false;
    // CTRL_REG4 (0x23): BDU=0, BLE=0, FS=00 (±2 g), HR=0 (normal), ST=00, SIM=0.
    // 0x00 → defaults; explicit write to be deterministic post-reset.
    if (i2c_write_reg(LIS3DH_ADDR, LIS3DH_CTRL_REG4, 0x00) != ESP_OK) return false;

    s_kind = IMU_LIS3DH;
    s_addr = LIS3DH_ADDR;
    ESP_LOGI(TAG, "LIS3DH ready @ 0x%02X (WHO=0x%02X, ODR=50 Hz, FS=±2 g)",
             LIS3DH_ADDR, who);
    return true;
}

static bool probe_lis3dsh(void)
{
    uint8_t who = 0;
    if (i2c_read_regs(LIS_ADDR, LIS_WHO, &who, 1) != ESP_OK) return false;
    if (who != LIS_WHO_EXPECTED) return false;

    // CTRL_REG4: ODR=50 Hz, all 3 axes on.
    if (i2c_write_reg(LIS_ADDR, LIS_CTRL_REG4, 0x47) != ESP_OK) return false;
    // CTRL_REG5: ±2 g (default), default anti-alias.
    if (i2c_write_reg(LIS_ADDR, LIS_CTRL_REG5, 0x00) != ESP_OK) return false;

    s_kind = IMU_LIS3DSH;
    s_addr = LIS_ADDR;
    ESP_LOGI(TAG, "LIS3DSH ready @ 0x%02X (WHO=0x%02X, ODR=50 Hz, FS=±2 g)",
             LIS_ADDR, who);
    return true;
}

static bool probe_mpu(void)
{
    uint8_t who = 0;
    if (i2c_read_regs(MPU_ADDR, MPU_WHO, &who, 1) != ESP_OK) return false;
    if (who != MPU_WHO_6050 && who != MPU_WHO_9250) {
        ESP_LOGW(TAG, "device at 0x%02X returned WHO=0x%02X — unknown, skipping",
                 MPU_ADDR, who);
        return false;
    }

    // Wake from sleep mode (default after reset).
    if (i2c_write_reg(MPU_ADDR, MPU_PWR_MGMT_1, 0x00) != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(10));   // settle internal regulator
    // ACCEL_CONFIG = 0x00 → ±2 g full scale, no high-pass filter.
    if (i2c_write_reg(MPU_ADDR, MPU_ACCEL_CONFIG, 0x00) != ESP_OK) return false;

    s_kind = (who == MPU_WHO_9250) ? IMU_MPU9250 : IMU_MPU6050;
    s_addr = MPU_ADDR;
    ESP_LOGI(TAG, "%s ready @ 0x%02X (WHO=0x%02X, FS=±2 g) — prototype IMU",
             kind_str(s_kind), MPU_ADDR, who);
    return true;
}

// ---------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------

esp_err_t bf_lis3dsh_init(void)
{
    static bool s_inited = false;
    if (s_inited) return ESP_OK;
    s_inited = true;

    i2c_bus_init_once();

    if (probe_lis3dh()) {
        s_present = true;
    } else if (probe_lis3dsh()) {
        s_present = true;
    } else if (probe_mpu()) {
        s_present = true;
    } else {
        ESP_LOGW(TAG, "no IMU detected on bus — face detection disabled");
        return ESP_OK;
    }

    // CLI accel.read removed — raw IMU values are dev-only; the consumer
    // (bf_face_detector) reads through the public API directly.
    (void)s_cmd_accel;
    sk_capabilities_register_book("bf_lis3dsh", "0.3.0");   // + LIS3DH
    return ESP_OK;
}
