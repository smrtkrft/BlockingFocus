// bf_secure_store — see header for the rationale, threat model and on-disk
// layout overview. This file is the only place that touches the master key
// or the AES-GCM primitives.

#include "bf_secure_store.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "cJSON.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "mbedtls/base64.h"
#include "mbedtls/gcm.h"
#include "nvs.h"

#include "sk_auth.h"
#include "sk_cli.h"
#include "sk_errors.h"
#include "sk_event_bus.h"

static const char *TAG = "bf_sec";

// ---- Constants -------------------------------------------------------------

#define KEY_LEN          32           // AES-256
#define NONCE_LEN        12           // GCM standard
#define TAG_LEN          16           // GCM tag

#define NVS_NS_KV        "bf_sec"
#define NVS_NS_INT       "bf_sec_int"
#define NVS_KEY_KV_BLOB  "kv"
#define NVS_KEY_MASTER   "mk"

#define USERDATA_PARTITION "userdata"
#define USERDATA_MOUNTPT   "/userdata"
#define USERDATA_PATH      "/userdata/blob.enc"
#define USERDATA_MAGIC     0x554B5332u  // 'SKU2' little-endian on the wire
#define USERDATA_VERSION   1u

#define KV_BLOB_MAGIC      0x564B5332u  // 'SKV2' little-endian on the wire
#define KV_BLOB_VERSION    1u
// Plaintext budget for the entire KV map (sum of all entries' encoded size).
// NVS blob ceiling is well under 4 KB by default; we keep a hard cap so
// the per-set quota check is deterministic.
#define KV_PLAIN_CAP       1536

// ---- State -----------------------------------------------------------------

static SemaphoreHandle_t s_mtx       = NULL;
static bool              s_ready     = false;
static bool              s_userdata_mounted = false;
static uint8_t           s_master_key[KEY_LEN];

// In-memory cache of the decrypted KV map. Loaded lazily on first access,
// rewritten to NVS on every set/erase. Entries are kept sorted by key for
// deterministic listing.
typedef struct {
    char    name[BF_SECURE_KV_MAX_KEY_LEN + 1];
    char    value[BF_SECURE_KV_MAX_VALUE_LEN + 1];
    size_t  value_len;   // not counting NUL
} kv_entry_t;

static kv_entry_t  s_kv[BF_SECURE_KV_MAX_ENTRIES];
static size_t      s_kv_count   = 0;
static bool        s_kv_loaded  = false;

// ---- Forward decls ---------------------------------------------------------

static esp_err_t kv_load_locked(void);
static esp_err_t kv_save_locked(void);
static int       kv_find_locked(const char *key);
static void      register_cli(void);
static void      on_factory_reset(const sk_event_t *evt, void *user);

// ---- AES-GCM wrapper -------------------------------------------------------

// Encrypt: output layout = nonce(12) || ciphertext(plain_len) || tag(16).
// `out` must have capacity >= plain_len + NONCE_LEN + TAG_LEN.
static esp_err_t aead_encrypt(const uint8_t *plain, size_t plain_len,
                              uint8_t *out, size_t *out_len)
{
    uint8_t nonce[NONCE_LEN];
    esp_fill_random(nonce, NONCE_LEN);

    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);
    int rc = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, s_master_key, KEY_LEN * 8);
    if (rc != 0) { mbedtls_gcm_free(&gcm); return ESP_FAIL; }

    uint8_t tag[TAG_LEN];
    rc = mbedtls_gcm_crypt_and_tag(&gcm, MBEDTLS_GCM_ENCRYPT,
                                   plain_len,
                                   nonce, NONCE_LEN,
                                   NULL, 0,           // no AAD
                                   plain,
                                   out + NONCE_LEN,
                                   TAG_LEN, tag);
    mbedtls_gcm_free(&gcm);
    if (rc != 0) return ESP_FAIL;

    memcpy(out,                          nonce, NONCE_LEN);
    memcpy(out + NONCE_LEN + plain_len,  tag,   TAG_LEN);
    *out_len = NONCE_LEN + plain_len + TAG_LEN;
    return ESP_OK;
}

// Decrypt: input layout = nonce || ciphertext || tag. plaintext written to
// `out`; caller must size `out` >= cipher_len - NONCE_LEN - TAG_LEN.
static esp_err_t aead_decrypt(const uint8_t *blob, size_t blob_len,
                              uint8_t *out, size_t *out_len)
{
    if (blob_len < (size_t)(NONCE_LEN + TAG_LEN)) return ESP_ERR_INVALID_SIZE;
    size_t cipher_len = blob_len - NONCE_LEN - TAG_LEN;

    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);
    int rc = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, s_master_key, KEY_LEN * 8);
    if (rc != 0) { mbedtls_gcm_free(&gcm); return ESP_FAIL; }

    rc = mbedtls_gcm_auth_decrypt(&gcm,
                                  cipher_len,
                                  blob, NONCE_LEN,
                                  NULL, 0,            // no AAD
                                  blob + NONCE_LEN + cipher_len, TAG_LEN,
                                  blob + NONCE_LEN,   // ciphertext
                                  out);
    mbedtls_gcm_free(&gcm);
    if (rc != 0) {
        ESP_LOGW(TAG, "auth_decrypt rc=%d (tampered or wrong key)", rc);
        return ESP_FAIL;
    }
    *out_len = cipher_len;
    return ESP_OK;
}

// ---- Master key ------------------------------------------------------------

static esp_err_t master_key_load_or_generate(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS_INT, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    size_t sz = sizeof(s_master_key);
    err = nvs_get_blob(h, NVS_KEY_MASTER, s_master_key, &sz);
    if (err == ESP_OK && sz == KEY_LEN) {
        nvs_close(h);
        ESP_LOGI(TAG, "master key loaded");
        return ESP_OK;
    }

    // Generate fresh key.
    esp_fill_random(s_master_key, KEY_LEN);
    err = nvs_set_blob(h, NVS_KEY_MASTER, s_master_key, KEY_LEN);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "master key persist failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGW(TAG, "fresh master key generated");
    return ESP_OK;
}

// ---- KV blob: serialization ------------------------------------------------
//
// Plaintext format inside the encrypted blob:
//   u32 magic = KV_BLOB_MAGIC
//   u32 version
//   u16 count
//   repeated `count` times:
//     u8  name_len     (1..BF_SECURE_KV_MAX_KEY_LEN)
//     u8[name_len] name
//     u16 value_len    (0..BF_SECURE_KV_MAX_VALUE_LEN)
//     u8[value_len] value
//
// Multi-byte integers are little-endian. Keep this simple and explicit so a
// Flutter-side reader could decode without C struct alignment surprises.

static void put_u16(uint8_t *p, uint16_t v) { p[0] = v & 0xff; p[1] = (v >> 8) & 0xff; }
static void put_u32(uint8_t *p, uint32_t v) { p[0]=v; p[1]=v>>8; p[2]=v>>16; p[3]=v>>24; }
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }
static uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static size_t kv_serialize(uint8_t *out, size_t cap)
{
    if (cap < 10) return 0;
    size_t off = 0;
    put_u32(out + off, KV_BLOB_MAGIC);   off += 4;
    put_u32(out + off, KV_BLOB_VERSION); off += 4;
    put_u16(out + off, (uint16_t)s_kv_count); off += 2;
    for (size_t i = 0; i < s_kv_count; i++) {
        size_t nlen = strlen(s_kv[i].name);
        size_t vlen = s_kv[i].value_len;
        if (off + 1 + nlen + 2 + vlen > cap) return 0;
        out[off++] = (uint8_t)nlen;
        memcpy(out + off, s_kv[i].name, nlen); off += nlen;
        put_u16(out + off, (uint16_t)vlen); off += 2;
        memcpy(out + off, s_kv[i].value, vlen); off += vlen;
    }
    return off;
}

static esp_err_t kv_deserialize(const uint8_t *in, size_t in_len)
{
    if (in_len < 10) return ESP_ERR_INVALID_SIZE;
    if (get_u32(in)     != KV_BLOB_MAGIC)   return ESP_ERR_INVALID_VERSION;
    if (get_u32(in + 4) != KV_BLOB_VERSION) return ESP_ERR_INVALID_VERSION;

    uint16_t n = get_u16(in + 8);
    if (n > BF_SECURE_KV_MAX_ENTRIES) return ESP_ERR_INVALID_SIZE;

    size_t off = 10;
    s_kv_count = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (off + 1 > in_len) return ESP_ERR_INVALID_SIZE;
        uint8_t nlen = in[off++];
        if (nlen == 0 || nlen > BF_SECURE_KV_MAX_KEY_LEN) return ESP_ERR_INVALID_SIZE;
        if (off + nlen + 2 > in_len) return ESP_ERR_INVALID_SIZE;
        memcpy(s_kv[i].name, in + off, nlen); off += nlen;
        s_kv[i].name[nlen] = '\0';

        uint16_t vlen = get_u16(in + off); off += 2;
        if (vlen > BF_SECURE_KV_MAX_VALUE_LEN) return ESP_ERR_INVALID_SIZE;
        if (off + vlen > in_len) return ESP_ERR_INVALID_SIZE;
        memcpy(s_kv[i].value, in + off, vlen); off += vlen;
        s_kv[i].value[vlen] = '\0';
        s_kv[i].value_len = vlen;

        s_kv_count++;
    }
    return ESP_OK;
}

// ---- KV blob: NVS persistence ---------------------------------------------

static esp_err_t kv_load_locked(void)
{
    if (s_kv_loaded) return ESP_OK;
    s_kv_count = 0;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS_KV, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        s_kv_loaded = true;
        return ESP_OK;       // first run: empty namespace
    }
    if (err != ESP_OK) return err;

    size_t blob_len = 0;
    err = nvs_get_blob(h, NVS_KEY_KV_BLOB, NULL, &blob_len);
    if (err == ESP_ERR_NVS_NOT_FOUND || blob_len == 0) {
        nvs_close(h);
        s_kv_loaded = true;
        return ESP_OK;       // namespace exists but empty
    }
    if (err != ESP_OK) { nvs_close(h); return err; }

    uint8_t *blob = malloc(blob_len);
    if (!blob) { nvs_close(h); return ESP_ERR_NO_MEM; }

    err = nvs_get_blob(h, NVS_KEY_KV_BLOB, blob, &blob_len);
    nvs_close(h);
    if (err != ESP_OK) { free(blob); return err; }

    uint8_t *plain = malloc(KV_PLAIN_CAP);
    if (!plain) { free(blob); return ESP_ERR_NO_MEM; }

    size_t plain_len = 0;
    err = aead_decrypt(blob, blob_len, plain, &plain_len);
    free(blob);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "kv blob decrypt failed — discarding (treating as empty)");
        free(plain);
        s_kv_loaded = true;     // present a clean namespace; user can rewrite
        return ESP_OK;
    }

    err = kv_deserialize(plain, plain_len);
    free(plain);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "kv deserialize failed: %s", esp_err_to_name(err));
        s_kv_count = 0;
    }
    s_kv_loaded = true;
    return ESP_OK;
}

static esp_err_t kv_save_locked(void)
{
    uint8_t plain[KV_PLAIN_CAP];
    size_t plain_len = kv_serialize(plain, sizeof(plain));
    if (plain_len == 0) return ESP_ERR_INVALID_SIZE;

    size_t blob_cap = plain_len + NONCE_LEN + TAG_LEN;
    uint8_t *blob = malloc(blob_cap);
    if (!blob) return ESP_ERR_NO_MEM;
    size_t blob_len = 0;
    esp_err_t err = aead_encrypt(plain, plain_len, blob, &blob_len);
    if (err != ESP_OK) { free(blob); return err; }

    nvs_handle_t h;
    err = nvs_open(NVS_NS_KV, NVS_READWRITE, &h);
    if (err != ESP_OK) { free(blob); return err; }
    err = nvs_set_blob(h, NVS_KEY_KV_BLOB, blob, blob_len);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    free(blob);
    return err;
}

static int kv_find_locked(const char *key)
{
    for (size_t i = 0; i < s_kv_count; i++) {
        if (strcmp(s_kv[i].name, key) == 0) return (int)i;
    }
    return -1;
}

// ---- Public KV API ---------------------------------------------------------

esp_err_t bf_secure_store_kv_get(const char *key, char *value_out, size_t cap, size_t *out_len)
{
    if (!s_ready || !key || !value_out || cap == 0) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    esp_err_t err = kv_load_locked();
    if (err != ESP_OK) { xSemaphoreGive(s_mtx); return err; }
    int idx = kv_find_locked(key);
    if (idx < 0) { xSemaphoreGive(s_mtx); return ESP_ERR_NOT_FOUND; }

    size_t vlen = s_kv[idx].value_len;
    if (vlen + 1 > cap) { xSemaphoreGive(s_mtx); return ESP_ERR_NO_MEM; }
    memcpy(value_out, s_kv[idx].value, vlen);
    value_out[vlen] = '\0';
    if (out_len) *out_len = vlen;
    xSemaphoreGive(s_mtx);
    return ESP_OK;
}

esp_err_t bf_secure_store_kv_set(const char *key, const char *value)
{
    if (!s_ready || !key || !value) return ESP_ERR_INVALID_ARG;
    size_t klen = strlen(key);
    size_t vlen = strlen(value);
    if (klen == 0 || klen > BF_SECURE_KV_MAX_KEY_LEN)   return ESP_ERR_INVALID_ARG;
    if (vlen > BF_SECURE_KV_MAX_VALUE_LEN)              return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_mtx, portMAX_DELAY);
    esp_err_t err = kv_load_locked();
    if (err != ESP_OK) { xSemaphoreGive(s_mtx); return err; }

    int idx = kv_find_locked(key);
    if (idx < 0) {
        if (s_kv_count >= BF_SECURE_KV_MAX_ENTRIES) {
            xSemaphoreGive(s_mtx);
            return ESP_ERR_NO_MEM;
        }
        idx = (int)s_kv_count++;
        memcpy(s_kv[idx].name, key, klen);
        s_kv[idx].name[klen] = '\0';
    }
    memcpy(s_kv[idx].value, value, vlen);
    s_kv[idx].value[vlen] = '\0';
    s_kv[idx].value_len = vlen;

    err = kv_save_locked();
    xSemaphoreGive(s_mtx);
    return err;
}

esp_err_t bf_secure_store_kv_erase(const char *key)
{
    if (!s_ready || !key) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    esp_err_t err = kv_load_locked();
    if (err != ESP_OK) { xSemaphoreGive(s_mtx); return err; }
    int idx = kv_find_locked(key);
    if (idx < 0) { xSemaphoreGive(s_mtx); return ESP_ERR_NOT_FOUND; }
    // Compact: shift tail down by one slot.
    for (size_t i = (size_t)idx; i + 1 < s_kv_count; i++) s_kv[i] = s_kv[i + 1];
    s_kv_count--;
    err = kv_save_locked();
    xSemaphoreGive(s_mtx);
    return err;
}

esp_err_t bf_secure_store_kv_list(char keys_out[][BF_SECURE_KV_MAX_KEY_LEN + 1],
                                  size_t max, size_t *n_out)
{
    if (!s_ready) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    esp_err_t err = kv_load_locked();
    if (err != ESP_OK) { xSemaphoreGive(s_mtx); return err; }
    if (n_out) *n_out = s_kv_count;
    size_t n = (s_kv_count < max) ? s_kv_count : max;
    for (size_t i = 0; i < n; i++) {
        strncpy(keys_out[i], s_kv[i].name, BF_SECURE_KV_MAX_KEY_LEN);
        keys_out[i][BF_SECURE_KV_MAX_KEY_LEN] = '\0';
    }
    xSemaphoreGive(s_mtx);
    return ESP_OK;
}

// ---- Userdata: SPIFFS-backed encrypted blob -------------------------------
//
// On-disk layout (single file `/userdata/blob.enc`):
//   u32 magic
//   u32 version
//   u32 plaintext_size
//   u8[NONCE_LEN] nonce
//   u8[plaintext_size] ciphertext
//   u8[TAG_LEN] tag
// Total = 24 + plaintext_size bytes.
//
// We treat the file as the source of truth — no in-memory cache. Each
// public call decrypts the whole blob into a heap buffer, performs the
// requested slice operation, and (for writes) re-encrypts and rewrites.
// 100 KB of plaintext fits comfortably in the available heap on ESP32-C6.

#define USERDATA_HEADER_LEN  (4 + 4 + 4 + NONCE_LEN)

static esp_err_t userdata_mount(void)
{
    if (s_userdata_mounted) return ESP_OK;
    esp_vfs_spiffs_conf_t cfg = {
        .base_path              = USERDATA_MOUNTPT,
        .partition_label        = USERDATA_PARTITION,
        .max_files              = 4,
        .format_if_mount_failed = true,
    };
    esp_err_t err = esp_vfs_spiffs_register(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "userdata SPIFFS mount failed: %s", esp_err_to_name(err));
        return err;
    }
    s_userdata_mounted = true;
    ESP_LOGI(TAG, "userdata partition mounted at %s", USERDATA_MOUNTPT);
    return ESP_OK;
}

// Read whole encrypted blob, decrypt into `*plain_out` (allocated on the
// heap, caller frees). On a missing/corrupt/empty blob, returns a fresh
// zero-length plaintext (so first-time reads see a clean 0-byte scratch).
static esp_err_t userdata_load_locked(uint8_t **plain_out, size_t *plain_len_out)
{
    *plain_out = NULL;
    *plain_len_out = 0;

    FILE *f = fopen(USERDATA_PATH, "rb");
    if (!f) {
        // Treat absence as empty.
        *plain_out = malloc(1);   // tiny buffer so callers can realloc safely
        if (!*plain_out) return ESP_ERR_NO_MEM;
        return ESP_OK;
    }

    fseek(f, 0, SEEK_END);
    long fs = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (fs < (long)(USERDATA_HEADER_LEN + TAG_LEN)) {
        fclose(f);
        ESP_LOGW(TAG, "userdata blob truncated (fs=%ld) — resetting", fs);
        *plain_out = malloc(1);
        return *plain_out ? ESP_OK : ESP_ERR_NO_MEM;
    }

    uint8_t header[USERDATA_HEADER_LEN];
    if (fread(header, 1, sizeof(header), f) != sizeof(header)) {
        fclose(f); return ESP_FAIL;
    }
    if (get_u32(header)     != USERDATA_MAGIC ||
        get_u32(header + 4) != USERDATA_VERSION) {
        fclose(f);
        ESP_LOGW(TAG, "userdata blob magic/version mismatch — resetting");
        *plain_out = malloc(1);
        return *plain_out ? ESP_OK : ESP_ERR_NO_MEM;
    }
    uint32_t plain_size = get_u32(header + 8);
    if (plain_size > BF_SECURE_USERDATA_CAP) {
        fclose(f); return ESP_ERR_INVALID_SIZE;
    }

    long expected = (long)USERDATA_HEADER_LEN + (long)plain_size + TAG_LEN;
    if (fs != expected) {
        fclose(f);
        ESP_LOGW(TAG, "userdata blob size mismatch fs=%ld expected=%ld — resetting",
                 fs, expected);
        *plain_out = malloc(1);
        return *plain_out ? ESP_OK : ESP_ERR_NO_MEM;
    }

    // Reassemble the GCM-input layout = nonce || cipher || tag in one buffer.
    size_t blob_len = (size_t)NONCE_LEN + plain_size + TAG_LEN;
    uint8_t *blob = malloc(blob_len);
    if (!blob) { fclose(f); return ESP_ERR_NO_MEM; }
    memcpy(blob, header + 12, NONCE_LEN);
    if (fread(blob + NONCE_LEN, 1, plain_size + TAG_LEN, f) != plain_size + TAG_LEN) {
        free(blob); fclose(f); return ESP_FAIL;
    }
    fclose(f);

    uint8_t *plain = malloc(plain_size > 0 ? plain_size : 1);
    if (!plain) { free(blob); return ESP_ERR_NO_MEM; }

    size_t plain_actual = 0;
    esp_err_t err = aead_decrypt(blob, blob_len, plain, &plain_actual);
    free(blob);
    if (err != ESP_OK || plain_actual != plain_size) {
        free(plain);
        ESP_LOGW(TAG, "userdata blob decrypt failed — resetting");
        *plain_out = malloc(1);
        return *plain_out ? ESP_OK : ESP_ERR_NO_MEM;
    }
    *plain_out = plain;
    *plain_len_out = plain_size;
    return ESP_OK;
}

static esp_err_t userdata_save_locked(const uint8_t *plain, size_t plain_len)
{
    if (plain_len > BF_SECURE_USERDATA_CAP) return ESP_ERR_INVALID_SIZE;

    size_t blob_len = NONCE_LEN + plain_len + TAG_LEN;
    uint8_t *blob = malloc(blob_len);
    if (!blob) return ESP_ERR_NO_MEM;
    size_t got = 0;
    esp_err_t err = aead_encrypt(plain, plain_len, blob, &got);
    if (err != ESP_OK) { free(blob); return err; }

    FILE *f = fopen(USERDATA_PATH, "wb");
    if (!f) { free(blob); return ESP_FAIL; }

    uint8_t header[USERDATA_HEADER_LEN];
    put_u32(header,     USERDATA_MAGIC);
    put_u32(header + 4, USERDATA_VERSION);
    put_u32(header + 8, (uint32_t)plain_len);
    memcpy(header + 12, blob, NONCE_LEN);    // nonce sits at the start of `blob`

    bool ok = fwrite(header, 1, sizeof(header), f) == sizeof(header) &&
              fwrite(blob + NONCE_LEN, 1, plain_len + TAG_LEN, f) == plain_len + TAG_LEN;
    fclose(f);
    free(blob);
    return ok ? ESP_OK : ESP_FAIL;
}

size_t bf_secure_store_userdata_size(void)
{
    if (!s_ready) return 0;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    uint8_t *plain = NULL;
    size_t   plain_len = 0;
    if (userdata_load_locked(&plain, &plain_len) == ESP_OK) free(plain);
    else plain_len = 0;
    xSemaphoreGive(s_mtx);
    return plain_len;
}

esp_err_t bf_secure_store_userdata_read(size_t offset, void *out, size_t len, size_t *actual)
{
    if (!s_ready || !out) return ESP_ERR_INVALID_ARG;
    if (actual) *actual = 0;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    uint8_t *plain = NULL;
    size_t   plain_len = 0;
    esp_err_t err = userdata_load_locked(&plain, &plain_len);
    if (err != ESP_OK) { xSemaphoreGive(s_mtx); return err; }

    if (offset >= plain_len) {
        free(plain);
        xSemaphoreGive(s_mtx);
        return ESP_OK;       // nothing to read past EOF; actual stays 0
    }
    size_t avail = plain_len - offset;
    size_t copy  = (len < avail) ? len : avail;
    memcpy(out, plain + offset, copy);
    if (actual) *actual = copy;
    free(plain);
    xSemaphoreGive(s_mtx);
    return ESP_OK;
}

esp_err_t bf_secure_store_userdata_write(size_t offset, const void *in, size_t len)
{
    if (!s_ready || !in) return ESP_ERR_INVALID_ARG;
    if (offset + len > BF_SECURE_USERDATA_CAP) return ESP_ERR_INVALID_SIZE;

    xSemaphoreTake(s_mtx, portMAX_DELAY);
    uint8_t *plain = NULL;
    size_t   plain_len = 0;
    esp_err_t err = userdata_load_locked(&plain, &plain_len);
    if (err != ESP_OK) { xSemaphoreGive(s_mtx); return err; }

    size_t need = offset + len;
    if (need > plain_len) {
        uint8_t *bigger = realloc(plain, need);
        if (!bigger) { free(plain); xSemaphoreGive(s_mtx); return ESP_ERR_NO_MEM; }
        plain = bigger;
        // Zero-fill any gap between old end and `offset`.
        if (offset > plain_len) memset(plain + plain_len, 0, offset - plain_len);
        plain_len = need;
    }
    memcpy(plain + offset, in, len);

    err = userdata_save_locked(plain, plain_len);
    free(plain);
    xSemaphoreGive(s_mtx);
    return err;
}

esp_err_t bf_secure_store_userdata_truncate(size_t new_size)
{
    if (!s_ready) return ESP_ERR_INVALID_ARG;
    if (new_size > BF_SECURE_USERDATA_CAP) return ESP_ERR_INVALID_SIZE;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    uint8_t *plain = NULL;
    size_t   plain_len = 0;
    esp_err_t err = userdata_load_locked(&plain, &plain_len);
    if (err != ESP_OK) { xSemaphoreGive(s_mtx); return err; }
    if (new_size > plain_len) {
        uint8_t *bigger = realloc(plain, new_size);
        if (!bigger) { free(plain); xSemaphoreGive(s_mtx); return ESP_ERR_NO_MEM; }
        plain = bigger;
        memset(plain + plain_len, 0, new_size - plain_len);
    }
    err = userdata_save_locked(plain, new_size);
    free(plain);
    xSemaphoreGive(s_mtx);
    return err;
}

esp_err_t bf_secure_store_userdata_clear(void)
{
    if (!s_ready) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    // Replace with a freshly-encrypted empty blob; deleting + re-creating
    // would leave a window where the file is missing, which our load path
    // already handles, but rewriting keeps readers consistent on next mount.
    esp_err_t err = userdata_save_locked((const uint8_t *)"", 0);
    xSemaphoreGive(s_mtx);
    return err;
}

// ---- CLI handlers ----------------------------------------------------------

static sk_err_t cmd_secure_get(sk_cli_ctx_t *ctx)
{
    const char *key = sk_cli_arg_named(ctx, "key");
    if (!key) { sk_cli_err(ctx, SK_ERR_MISSING_ARG, "{\"field\":\"key\"}"); return SK_OK; }

    char value[BF_SECURE_KV_MAX_VALUE_LEN + 1];
    size_t vlen = 0;
    esp_err_t err = bf_secure_store_kv_get(key, value, sizeof(value), &vlen);
    if (err == ESP_ERR_NOT_FOUND) { sk_cli_err(ctx, SK_ERR_NOT_FOUND, NULL); return SK_OK; }
    if (err != ESP_OK)            { sk_cli_err(ctx, SK_ERR_INTERNAL, NULL); return SK_OK; }

    // Use cJSON so embedded quotes / backslashes / control chars in `value`
    // are escaped correctly.
    cJSON *obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "key",   key);
    cJSON_AddStringToObject(obj, "value", value);
    char *json = cJSON_PrintUnformatted(obj);
    cJSON_Delete(obj);
    if (!json) { sk_cli_err(ctx, SK_ERR_INTERNAL, NULL); return SK_OK; }
    sk_cli_ok(ctx, json);
    cJSON_free(json);
    return SK_OK;
}

static sk_err_t cmd_secure_set(sk_cli_ctx_t *ctx)
{
    const char *key   = sk_cli_arg_named(ctx, "key");
    const char *value = sk_cli_arg_named(ctx, "value");
    if (!key || !value) {
        sk_cli_err(ctx, SK_ERR_MISSING_ARG,
                   key ? "{\"field\":\"value\"}" : "{\"field\":\"key\"}");
        return SK_OK;
    }
    esp_err_t err = bf_secure_store_kv_set(key, value);
    if (err != ESP_OK) {
        sk_cli_err(ctx, err == ESP_ERR_INVALID_ARG ? SK_ERR_INVALID_ARG :
                        err == ESP_ERR_NO_MEM    ? SK_ERR_NO_SPACE    :
                                                   SK_ERR_INTERNAL, NULL);
        return SK_OK;
    }
    sk_cli_ok(ctx, "{\"saved\":true}");
    return SK_OK;
}

static sk_err_t cmd_secure_erase(sk_cli_ctx_t *ctx)
{
    const char *key = sk_cli_arg_named(ctx, "key");
    if (!key) { sk_cli_err(ctx, SK_ERR_MISSING_ARG, "{\"field\":\"key\"}"); return SK_OK; }
    esp_err_t err = bf_secure_store_kv_erase(key);
    if (err == ESP_ERR_NOT_FOUND) { sk_cli_err(ctx, SK_ERR_NOT_FOUND, NULL); return SK_OK; }
    if (err != ESP_OK)            { sk_cli_err(ctx, SK_ERR_INTERNAL, NULL); return SK_OK; }
    sk_cli_ok(ctx, "{\"erased\":true}");
    return SK_OK;
}

static sk_err_t cmd_secure_list(sk_cli_ctx_t *ctx)
{
    char keys[BF_SECURE_KV_MAX_ENTRIES][BF_SECURE_KV_MAX_KEY_LEN + 1];
    size_t n = 0;
    if (bf_secure_store_kv_list(keys, BF_SECURE_KV_MAX_ENTRIES, &n) != ESP_OK) {
        sk_cli_err(ctx, SK_ERR_INTERNAL, NULL); return SK_OK;
    }
    cJSON *arr = cJSON_CreateArray();
    size_t shown = (n < BF_SECURE_KV_MAX_ENTRIES) ? n : BF_SECURE_KV_MAX_ENTRIES;
    for (size_t i = 0; i < shown; i++) cJSON_AddItemToArray(arr, cJSON_CreateString(keys[i]));
    cJSON *obj = cJSON_CreateObject();
    cJSON_AddItemToObject(obj, "keys", arr);
    cJSON_AddNumberToObject(obj, "count", (double)n);
    char *json = cJSON_PrintUnformatted(obj);
    cJSON_Delete(obj);
    if (!json) { sk_cli_err(ctx, SK_ERR_INTERNAL, NULL); return SK_OK; }
    sk_cli_ok(ctx, json);
    cJSON_free(json);
    return SK_OK;
}

// userdata.size — single integer.
static sk_err_t cmd_userdata_size(sk_cli_ctx_t *ctx)
{
    char buf[64];
    snprintf(buf, sizeof(buf),
             "{\"size\":%u,\"capacity\":%u}",
             (unsigned)bf_secure_store_userdata_size(),
             (unsigned)BF_SECURE_USERDATA_CAP);
    sk_cli_ok(ctx, buf);
    return SK_OK;
}

// CLI single-call read cap. Prevents an attacker from requesting a
// gigabyte-long base64 reply on a constrained transport. SKAPP streams
// in chunks; 4 KB per chunk keeps NVS/SPIFFS read+encode bounded.
#define USERDATA_CLI_CHUNK 4096

static long parse_long_arg(sk_cli_ctx_t *ctx, const char *name, long defval)
{
    // Machine-mode (SKAPP signed envelope): args is a JSON object and the
    // value can land as either a JSON number or a JSON string. The legacy
    // helper sk_cli_arg_named only returns strings — numeric values came
    // back as NULL and every numeric handler (userdata.size/read/write/
    // truncate, api.endpoint.add --delay-after, ...) defaulted to -1 and
    // failed with ERR_INVALID_ARG. Read directly from the cJSON tree so
    // both shapes work, while still accepting human-mode `--name <num>`.
    long v;
    if (sk_cli_arg_long(ctx, name, &v)) return v;
    return defval;
}

static sk_err_t cmd_userdata_read(sk_cli_ctx_t *ctx)
{
    long offset = parse_long_arg(ctx, "offset", 0);
    long len    = parse_long_arg(ctx, "len",    USERDATA_CLI_CHUNK);
    if (offset < 0 || len <= 0 || len > USERDATA_CLI_CHUNK) {
        sk_cli_err(ctx, SK_ERR_INVALID_ARG, NULL); return SK_OK;
    }

    uint8_t *buf = malloc((size_t)len);
    if (!buf) { sk_cli_err(ctx, SK_ERR_INTERNAL, NULL); return SK_OK; }
    size_t actual = 0;
    esp_err_t err = bf_secure_store_userdata_read((size_t)offset, buf, (size_t)len, &actual);
    if (err != ESP_OK) {
        free(buf);
        sk_cli_err(ctx, SK_ERR_INTERNAL, NULL);
        return SK_OK;
    }

    // base64 the actual bytes read.
    size_t b64_cap = ((actual + 2) / 3) * 4 + 1;
    unsigned char *b64 = malloc(b64_cap);
    if (!b64) { free(buf); sk_cli_err(ctx, SK_ERR_INTERNAL, NULL); return SK_OK; }
    size_t b64_len = 0;
    int rc = mbedtls_base64_encode(b64, b64_cap, &b64_len, buf, actual);
    free(buf);
    if (rc != 0) { free(b64); sk_cli_err(ctx, SK_ERR_INTERNAL, NULL); return SK_OK; }

    cJSON *obj = cJSON_CreateObject();
    cJSON_AddNumberToObject(obj, "offset", (double)offset);
    cJSON_AddNumberToObject(obj, "len",    (double)actual);
    cJSON_AddStringToObject(obj, "data_b64", (const char *)b64);
    cJSON_AddNumberToObject(obj, "total",  (double)bf_secure_store_userdata_size());
    char *json = cJSON_PrintUnformatted(obj);
    cJSON_Delete(obj);
    free(b64);
    if (!json) { sk_cli_err(ctx, SK_ERR_INTERNAL, NULL); return SK_OK; }
    sk_cli_ok(ctx, json);
    cJSON_free(json);
    return SK_OK;
}

static sk_err_t cmd_userdata_write(sk_cli_ctx_t *ctx)
{
    long offset = parse_long_arg(ctx, "offset", -1);
    const char *b64 = sk_cli_arg_named(ctx, "data_b64");
    if (offset < 0 || !b64) {
        sk_cli_err(ctx, SK_ERR_INVALID_ARG, NULL); return SK_OK;
    }
    size_t b64_len = strlen(b64);
    size_t bin_cap = (b64_len / 4) * 3 + 4;
    if (bin_cap > USERDATA_CLI_CHUNK + 16) {
        sk_cli_err(ctx, SK_ERR_INVALID_ARG, NULL); return SK_OK;
    }
    unsigned char *bin = malloc(bin_cap);
    if (!bin) { sk_cli_err(ctx, SK_ERR_INTERNAL, NULL); return SK_OK; }
    size_t bin_len = 0;
    int rc = mbedtls_base64_decode(bin, bin_cap, &bin_len,
                                   (const unsigned char *)b64, b64_len);
    if (rc != 0) {
        free(bin);
        sk_cli_err(ctx, SK_ERR_INVALID_ARG, "{\"reason\":\"base64\"}");
        return SK_OK;
    }
    esp_err_t err = bf_secure_store_userdata_write((size_t)offset, bin, bin_len);
    free(bin);
    if (err != ESP_OK) {
        sk_cli_err(ctx,
                   err == ESP_ERR_INVALID_SIZE ? SK_ERR_NO_SPACE : SK_ERR_INTERNAL,
                   NULL);
        return SK_OK;
    }
    char buf[64];
    snprintf(buf, sizeof(buf),
             "{\"written\":%u,\"total\":%u}",
             (unsigned)bin_len,
             (unsigned)bf_secure_store_userdata_size());
    sk_cli_ok(ctx, buf);
    return SK_OK;
}

static sk_err_t cmd_userdata_truncate(sk_cli_ctx_t *ctx)
{
    long size = parse_long_arg(ctx, "size", -1);
    if (size < 0 || size > BF_SECURE_USERDATA_CAP) {
        sk_cli_err(ctx, SK_ERR_INVALID_ARG, NULL); return SK_OK;
    }
    if (bf_secure_store_userdata_truncate((size_t)size) != ESP_OK) {
        sk_cli_err(ctx, SK_ERR_INTERNAL, NULL); return SK_OK;
    }
    sk_cli_ok(ctx, "{\"truncated\":true}");
    return SK_OK;
}

static sk_err_t cmd_userdata_clear(sk_cli_ctx_t *ctx)
{
    // Confirm-token gate: clearing the user blob is destructive enough that
    // we want a single-use token from device.confirm-token.get.
    const char *tok = sk_cli_confirm_token(ctx);
    if (!tok) { sk_cli_err(ctx, SK_ERR_CONFIRM_TOKEN_REQUIRED, NULL); return SK_OK; }
    if (sk_auth_confirm_consume(tok) != ESP_OK) {
        sk_cli_err(ctx, SK_ERR_CONFIRM_TOKEN_INVALID, NULL); return SK_OK;
    }
    if (bf_secure_store_userdata_clear() != ESP_OK) {
        sk_cli_err(ctx, SK_ERR_INTERNAL, NULL); return SK_OK;
    }
    sk_cli_ok(ctx, "{\"cleared\":true}");
    return SK_OK;
}

// All secure.* and userdata.* are SKAPP-only. requires_auth gates them at
// the dispatcher (USB CLI is unauthenticated → reject), and hidden=true
// keeps them out of `help` overview / `help all` so the CLI surface stays
// clean. SKAPP discovers them via device.commands which lists hidden too.
static const sk_cli_command_t s_cmds[] = {
    { .name = "secure.get",       .summary = "Read encrypted KV value (SKAPP-only)",
      .usage = "secure get --key <name>",
      .requires_auth = true,  .hidden = true, .handler = cmd_secure_get },
    { .name = "secure.set",       .summary = "Write encrypted KV value (SKAPP-only)",
      .usage = "secure set --key <name> --value <utf8>",
      .requires_auth = true,  .hidden = true, .handler = cmd_secure_set },
    { .name = "secure.erase",     .summary = "Delete an encrypted KV entry",
      .usage = "secure erase --key <name>",
      .requires_auth = true,  .hidden = true, .handler = cmd_secure_erase },
    { .name = "secure.list",      .summary = "List encrypted KV key names",
      .usage = "secure list",
      .requires_auth = true,  .hidden = true, .handler = cmd_secure_list },

    { .name = "userdata.size",    .summary = "Logical size of the user-script blob",
      .usage = "userdata size",
      .requires_auth = true,  .hidden = true, .handler = cmd_userdata_size },
    { .name = "userdata.read",    .summary = "Read a slice from the user-script blob (base64)",
      .usage = "userdata read --offset <n> --len <n>",
      .requires_auth = true,  .hidden = true, .handler = cmd_userdata_read },
    { .name = "userdata.write",   .summary = "Write a slice into the user-script blob",
      .usage = "userdata write --offset <n> --data_b64 <base64>",
      .requires_auth = true,  .hidden = true, .handler = cmd_userdata_write },
    { .name = "userdata.truncate", .summary = "Set logical size (zero-pads on growth)",
      .usage = "userdata truncate --size <n>",
      .requires_auth = true,  .hidden = true, .handler = cmd_userdata_truncate },
    { .name = "userdata.clear",   .summary = "Wipe the user-script blob (confirm token)",
      .usage = "userdata clear",
      .requires_auth = true,  .hidden = true, .critical = true, .handler = cmd_userdata_clear },
};

static void register_cli(void)
{
    // Topics intentionally NOT registered here. The secure/userdata commands
    // are all hidden=true; with no visible commands a topic registration
    // would render as an empty section anyway. The single source of truth
    // for visible topics lives in main.c.
    for (size_t i = 0; i < sizeof(s_cmds)/sizeof(s_cmds[0]); i++) {
        sk_cli_register(&s_cmds[i]);
    }
}

// ---- Factory reset hook ---------------------------------------------------

static void on_factory_reset(const sk_event_t *evt, void *user)
{
    (void)evt; (void)user;
    ESP_LOGW(TAG, "factory reset — wiping bf_secure_store");
    xSemaphoreTake(s_mtx, portMAX_DELAY);

    // KV namespace
    nvs_handle_t h;
    if (nvs_open(NVS_NS_KV, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h); nvs_commit(h); nvs_close(h);
    }
    s_kv_count = 0;
    s_kv_loaded = true;

    // Master key — generate fresh
    if (nvs_open(NVS_NS_INT, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h); nvs_commit(h); nvs_close(h);
    }
    esp_fill_random(s_master_key, KEY_LEN);
    if (nvs_open(NVS_NS_INT, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, NVS_KEY_MASTER, s_master_key, KEY_LEN);
        nvs_commit(h); nvs_close(h);
    }

    // Userdata — empty blob with the new key
    userdata_save_locked((const uint8_t *)"", 0);

    xSemaphoreGive(s_mtx);
}

// ---- Public init ----------------------------------------------------------

esp_err_t bf_secure_store_init(void)
{
    if (s_ready) return ESP_OK;
    s_mtx = xSemaphoreCreateMutex();
    if (!s_mtx) return ESP_ERR_NO_MEM;

    esp_err_t err = master_key_load_or_generate();
    if (err != ESP_OK) return err;

    err = userdata_mount();
    if (err != ESP_OK) {
        // Non-fatal: KV path still works without SPIFFS. userdata.* commands
        // will fail at runtime with ERR_INTERNAL until the partition exists.
        ESP_LOGW(TAG, "continuing without userdata partition");
    }

    register_cli();

    int sub;
    sk_event_bus_subscribe("device.factory-reset.requested",
                           on_factory_reset, NULL, &sub);

    s_ready = true;
    ESP_LOGI(TAG, "bf_secure_store ready (cap=%uB userdata, %d KV slots)",
             (unsigned)BF_SECURE_USERDATA_CAP, BF_SECURE_KV_MAX_ENTRIES);
    return ESP_OK;
}
