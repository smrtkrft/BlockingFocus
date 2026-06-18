#pragma once

/*
 * Blocking Focus — Pin atamaları (Seeed XIAO ESP32-C6, rev 2 PCB).
 *
 * 2026-04-28 revizyonu: 2.4" SPI display + PCF8574T expander tasarımdan
 * çıkarıldı, 1.5" I2C OLED'e geçildi. Tüm donanım XIAO'nun 11 dış pinine
 * sığar; expander ve display power-gate yok.
 *
 * 2026-05-08 revizyonu: Buzzer tasarımdan çıkarıldı (D3 FREE),
 * titreşim D10'a yerleşti.
 *
 * 2026-05-29 PCB revizyonu (final): PCB lehimlenmiş yerleşim
 * yetkili kaynak olarak kabul edildi. Eski breadboard/perfboard
 * prototip dönemindeki ad-hoc rework'ler (D1→D8 buton geçişi,
 * D6/D7 "ölü pad" workaround) PCB'de geçerli değil — PCB'yi
 * tasarlayan XIAO'da bu padler sağlam. Atamalar PCB'ye hizalandı:
 *   - BUTTON  : D2  (GPIO2)   eskiden D8'di
 *   - ACCEL INT1 : D8 (GPIO19) eskiden D2'ydi
 *   - VBUS_SENSE : D6 (GPIO16) eskiden D9'du
 *
 * Detay: docs/pin_map.md, docs/bom.md.
 *
 * Light-sleep wake herhangi bir GPIO'da çalışır (LP-IO sadece deep
 * sleep için gerekli, biz kullanmıyoruz).
 */

// =====================================================================
// XIAO ESP32-C6 doğrudan GPIO atamaları (rev 2 PCB, 2026-05-29)
// =====================================================================

// ===== Pil voltaj okuyucu (ADC1) =====
#define PIN_BATTERY_ADC          0     // GPIO0  (D0)  ADC1_CH0

// External resistor divider on PIN_BATTERY_ADC: BAT+ → R1 → ADC pin → R2 → GND.
// VBAT_actual = ADC_reading × DIVIDER_RATIO_X100 / 100.
// Default 200 = 2.0× ratio (1:1 divider — e.g. 10k+10k, 100k+100k or 220k+220k).
#define BATTERY_DIVIDER_RATIO_X100   200

// ===== Kullanıcı girişi (PCB: D2) =====
// 2026-05-29: PCB yerleşimine göre D2'ye geri taşındı. D2 ADC1_CH2 capable
// ama buton için kullanılıyor — ADC potansiyeli kaybedildi, sorun değil.
#define PIN_USER_BUTTON          2     // GPIO2  (D2)  active-low, light-sleep wake

// ===== Accelerometer motion interrupt (PCB: D8) =====
// LIS3DH primary (0x18), LIS3DSH legacy (0x1E), MPU fallback (0x68) —
// bf_lis3dsh driver autodetect probe sırası. Aynı INT1 hattı her chip için.
// 2026-05-29: PCB yerleşimine göre D8'e taşındı (eskiden D2'ydi).
#define PIN_LIS3DSH_INT1        19     // GPIO19 (D8)  active-high, latched, wake

// ===== Buzzer kaldırıldı (2026-05-08) =====
// Buzzer ile titreşim ayrı sistemler; bu prototipte buzzer yok.
// D3 (GPIO21) artık FREE — ileride buzzer eklenirse kendi component'i
// (bf_buzzer) olarak gelir, tasarım titreşimden bağımsız.

// ===== I2C bus (Accel 0x18/0x1E/0x68 + OLED 0x3C/0x3D paylaşımlı) =====
#define PIN_I2C_SDA             22     // GPIO22 (D4)
#define PIN_I2C_SCL             23     // GPIO23 (D5)

// ===== Vibration motor (direct GPIO → N-MOSFET gate, flyback diyot şart) =====
// PCB yerleşimi: D10 (GPIO18) — değişmedi.
#define PIN_VIBRATION           18     // GPIO18 (D10) active-high

// ===== VBUS sense (USB plug detect, dijital input via 10k+10k bölücü) =====
// 2026-05-29: PCB yerleşimine göre D6'ya taşındı (eskiden D9'du).
// Önceki "D6 ölü pad" notu prototip XIAO modülüne özgüydü; PCB'yi
// tasarlayan XIAO'da D6 sağlam.
#define PIN_VBUS_SENSE          16     // GPIO16 (D6)  HIGH = USB takılı

// ===== Boş GPIO'lar (gelecek için rezerve, PCB'de açık bırakıldı) =====
// D1  = GPIO1    FREE (ADC1_CH1 capable)
// D3  = GPIO21   FREE (eski buzzer pini)
// D7  = GPIO17   FREE
// D9  = GPIO20   FREE (eski VBUS_SENSE pini)

// ===== XIAO on-board status LED (sk_led) =====
#define PIN_STATUS_LED         15     // GPIO15 board üstü user LED

// ===== Rezerve / kullanılmayan =====
#define PIN_BOOT_RESERVED       9     // XIAO BOOT — board içi, dış erişim yok
// GPIO12 = USB D-, GPIO13 = USB D+ (USB Serial/JTAG için rezerve)

// =====================================================================
// I2C cihaz adresleri (tek bus, tüm cihazlar)
// =====================================================================
#define I2C_ADDR_LIS3DH        0x18   // LIS3DH 7-bit, SDO=GND (primary 2026-05-08)
#define I2C_ADDR_LIS3DSH       0x1E   // LIS3DSH 7-bit, SA0=GND (legacy fallback)
#define I2C_ADDR_OLED          0x3C   // SSD1306 / SSD1327 default; alt 0x3D
