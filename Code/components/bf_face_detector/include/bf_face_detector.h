#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =====================================================================
// bf_face_detector — cube-face detection from accelerometer.
//
// Polls bf_lis3dsh at 10 Hz, picks the dominant axis as the upright
// orientation, debounces by 1 s (CLAUDE.md hardware rule #3), then
// publishes face.changed when the committed face flips.
//
// Face numbering (raw, axis-driven — independent of cube assembly):
//   1 = +Z up   (LIS3DSH datasheet "Z up")     — convention: display face
//   2 = -Z up
//   3 = +X up
//   4 = -X up
//   5 = +Y up
//   6 = -Y up
// Project layer (bf_timer_engine) maps these numbers to durations
// (5/15/30/60 min) according to physical PCB orientation in the cube.
//
// Events on sk_event_bus:
//   face.changed  {"face": 3, "previous": 1}
//   face.tilted   {"tilted": true|false}
//                 — fires when the cube has been resting on a corner /
//                 edge for ≥ 5 s (tilted=true), and again once a flat
//                 axis returns (tilted=false). Committed face is left
//                 alone — an active countdown keeps running. UI uses
//                 this to show a "place the cube flat" overlay hint.
//
// CLI: `face.status`
//
// Graceful: if bf_lis3dsh is absent, the polling task still spawns but
// every read fails — face_current() stays 0, no events emitted.
// =====================================================================

#define BF_FACE_NONE       0
#define BF_FACE_Z_UP       1
#define BF_FACE_Z_DOWN     2
#define BF_FACE_X_UP       3
#define BF_FACE_X_DOWN     4
#define BF_FACE_Y_UP       5
#define BF_FACE_Y_DOWN     6

esp_err_t bf_face_detector_init(void);

// Currently committed face (post-debounce). 0 until first stable
// orientation is observed.
int       bf_face_detector_current(void);

#ifdef __cplusplus
}
#endif
