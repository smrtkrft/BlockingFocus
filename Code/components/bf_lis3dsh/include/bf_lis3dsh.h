#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =====================================================================
// bf_lis3dsh — 3-axis accelerometer driver (multi-chip).
//
// Probe order: ST LIS3DH @ 0x18 → ST LIS3DSH @ 0x1E → MPU-6050/9250 @ 0x68.
// Component name kept as "lis3dsh" for git/include continuity; LIS3DH
// is the production part as of 2026-05-08.
//
// Pure hardware abstraction: probe + register configuration + raw reads.
// Higher-level policy (cube face detection, motion wake) lives in
// bf_face_detector and bf_power respectively.
//
// Probe failure is non-fatal: the component logs a warning, marks itself
// absent, and read calls return ESP_ERR_NOT_FOUND so downstream features
// degrade gracefully.
//
// Default config: ±2 g, 50 Hz ODR, all axes enabled. Same 16384 LSB/g
// scaling on all chips → bf_lis3dsh_read_g uses one constant.
// =====================================================================

esp_err_t bf_lis3dsh_init(void);

// True if any supported accelerometer was found at boot.
bool      bf_lis3dsh_present(void);

// Raw 16-bit signed axis values. ±32768 maps to ±2 g at default scale.
esp_err_t bf_lis3dsh_read(int16_t *x, int16_t *y, int16_t *z);

// Convenience wrapper — returns axis values in g (one g ≈ 9.81 m/s²).
esp_err_t bf_lis3dsh_read_g(float *x, float *y, float *z);

#ifdef __cplusplus
}
#endif
