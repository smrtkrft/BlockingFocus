#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =====================================================================
// bf_vibration — titreşim motoru sürücüsü.
//
// 2026-05-08: bf_buzzer'dan ayrıldı. Buzzer ile titreşim ayrı sistemler;
// bu component sadece motor pin'ini sürer. Buzzer ileride gelirse
// kendi component'i olarak eklenir.
//
// Pin: PIN_VIBRATION (active-high), N-MOSFET gate üzerinden sürülür.
// API non-blocking — her çağrı arka plandaki worker task'ın kuyruğuna
// pulse atar, çağıran beklemez.
// =====================================================================

esp_err_t bf_vibration_init(void);

// Tek-pulse: 1..10000 ms. Test/UX kullanım — örn. timer son 5 sn uyarı.
void bf_vibration_pulse_ms(uint16_t ms);

// Multi-pulse burst: count adet pulse, her biri on_ms süresince HIGH,
// arasında off_ms boşluk. Toplam süre = count × on_ms + (count-1) × off_ms.
// count: 1..15 clamp. Timer expired'de "gel-git" desen için kullanılır.
void bf_vibration_burst(uint8_t count, uint16_t on_ms, uint16_t off_ms);

// Master switch — NVS-persisted. OFF iken sürücü pin'i HIGH'a almaz
// (worker loop çalışır, motor sessiz). Default ON. CLI: vibration on|off.
bool      bf_vibration_is_enabled(void);
esp_err_t bf_vibration_set_enabled(bool on);

// Warning-event gates — NVS-persisted. ON iken iç subscriber'lar
// karşılık gelen event'te titreşim pulsu yayınlar:
//   * tilt_warn     — `face.tilted`  (bf_face_detector ambiguous orientation)
//   * low_batt_warn — `battery.low`  (OK→LOW transition)
// CLI:
//   tilt.warn.on|off|status
//   low_batt.warn.on|off|status
//
// Naming history: tilt gate eski adı `face_up.warn` idi; UX 'cube düz
// dur' uyarısı, isim ona göre düzeltildi. NVS key (private) eski adı
// `face_warn` korundu, kullanıcının sakladığı toggle değeri kaybolmasın.
bool      bf_tilt_warn_is_enabled(void);
bool      bf_low_batt_warn_is_enabled(void);
esp_err_t bf_tilt_warn_set_enabled(bool on);
esp_err_t bf_low_batt_warn_set_enabled(bool on);

#ifdef __cplusplus
}
#endif
