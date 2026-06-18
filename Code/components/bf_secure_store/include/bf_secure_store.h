#pragma once

// bf_secure_store — encrypted, SKAPP-only key/value + 100 KB user-script blob.
//
// Two storage areas, both encrypted at rest with AES-256-GCM:
//
//   1) Key/value namespace ("secure.*" CLI commands):
//      Small (<= ~1.5 KB total) named string entries. Used for WiFi
//      credentials, device-config flags, SKAPP-side bookkeeping. Keys are
//      arbitrary UTF-8 strings (max 31 bytes). Values are arbitrary UTF-8
//      strings (max 256 bytes each).
//
//   2) User scratch blob ("userdata.*" CLI commands):
//      Up to 100 KB of opaque user payload. Intended to back script /
//      preset / API recipe storage that the user manages from the SKAPP.
//      Reads/writes are byte-addressable (offset + length) so the SKAPP can
//      stream chunks larger than a single CLI envelope.
//
// Threat model:
//   - USB CLI is treated as **untrusted**. None of these areas are
//     readable or writeable from USB. The CLI commands are registered
//     with requires_auth=true; USB always dispatches the unauthenticated
//     path.
//   - SKAPP-over-BLE/TCP, after the HMAC-envelope handshake, is the only
//     channel that may read/write secure entries.
//   - Flash dump remains a vulnerability vector (master key currently lives
//     in plain NVS — see implementation notes). Production hardening will
//     move the master key to the ESP32-C6 HMAC peripheral keyed by a
//     read-protected eFuse block. The on-disk format here already embeds
//     all the metadata required for that swap.
//
// Factory reset semantics:
//   - on `device.factory-reset.requested` event: master key, all KV
//     entries, and the user-scratch blob are erased. A fresh (empty)
//     master key is generated on the next call.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BF_SECURE_KV_MAX_KEY_LEN     31    // bytes (UTF-8, no null)
#define BF_SECURE_KV_MAX_VALUE_LEN   256   // bytes (UTF-8, no null)
#define BF_SECURE_KV_MAX_ENTRIES     32
#define BF_SECURE_USERDATA_CAP       102400  // exactly 100 KB plaintext

// Initialize: mount SPIFFS, load/generate master key, register CLI commands.
// Subscribes to "device.factory-reset.requested". Idempotent.
esp_err_t bf_secure_store_init(void);

// Get / set / erase a key/value entry. `value_out` receives a NUL-terminated
// UTF-8 string; `cap` includes space for the terminator. `out_len` (optional)
// reports the byte length excluding the terminator.
esp_err_t bf_secure_store_kv_get(const char *key,
                                 char *value_out, size_t cap, size_t *out_len);
esp_err_t bf_secure_store_kv_set(const char *key, const char *value);
esp_err_t bf_secure_store_kv_erase(const char *key);

// List all known keys. `keys_out` is filled with NUL-terminated strings up to
// `max`. `n_out` always reports the actual count even if it exceeds max
// (allowing the caller to detect truncation).
esp_err_t bf_secure_store_kv_list(char keys_out[][BF_SECURE_KV_MAX_KEY_LEN + 1],
                                  size_t max, size_t *n_out);

// User-scratch byte-addressable read/write. `offset+len` may not exceed the
// current logical size for read, or `BF_SECURE_USERDATA_CAP` for write.
// Writes past the current logical size grow the blob; gaps are zero-filled.
size_t    bf_secure_store_userdata_size(void);
esp_err_t bf_secure_store_userdata_read(size_t offset, void *out, size_t len, size_t *actual);
esp_err_t bf_secure_store_userdata_write(size_t offset, const void *in, size_t len);
esp_err_t bf_secure_store_userdata_truncate(size_t new_size);
esp_err_t bf_secure_store_userdata_clear(void);

#ifdef __cplusplus
}
#endif
