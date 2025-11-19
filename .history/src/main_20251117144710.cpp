/*
 * =================================================================================
 * ESP32 & Nextion - CAN BUS Radar Projesi
 * ---------------------------------------------------------------------------------
 * Versiyon: v2.6.0 (Tamamen Ayarlanabilir EEPROM)
 * Yazar:    G.T
 * Tarih:    14.11.2025
 *
 * --- v2.6.0 Değişiklik Notları ---
 * - YENİ ÖZELLİK (ANA): Tüm önemli ayarlar artık EEPROM'da saklanıyor.
 *   - Yanal zoom genişlikleri (LATERAL_RANGE_LEVEL_1..4) sabit olmaktan çıkarılıp
 *     EEPROM'dan yüklenen değişkenlere dönüştürüldü.
 * - GÜNCELLEME: EEPROM yapısı, 4 adet yanal genişlik ayarını içerecek şekilde
 *             64 byte'a genişletildi.
 * - GÜNCELLEME: Haberleşme protokolü güncellendi. "SAVE" komutu artık 7 adet
 *             parametre içeriyor.
 * - GÜNCELLEME: handleNextionInput() fonksiyonu, yeni 7 parametreli SAVE komutunu
 *             ayrıştıracak şekilde tamamen yeniden yazıldı.
 * =================================================================================
 */

// Gerekli Kütüphaneler
#include "driver/gpio.h"
#include "driver/twai.h"
#include <math.h>
#include <HardwareSerial.h>
#include <EEPROM.h>

// ======================= PROJE AYARLARI =======================

// --- Pin Ayarları ---
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

// --- EEPROM Ayarları ---
#define EEPROM_SIZE 64 // YENİ: Boyut artırıldı
const int EEPROM_MAGIC_KEY = 123;
const int ADDR_MAGIC_KEY = 0;
const int ADDR_WARN_ZONE = 4;
const int ADDR_DANGER_ZONE = 8;
const int ADDR_VEHICLE_WIDTH = 12;
const int ADDR_PASSWORD = 16;
// YENİ: Yanal genişlik adresleri
const int ADDR_LATERAL_L1 = 28;
const int ADDR_LATERAL_L2 = 32;
const int ADDR_LATERAL_L3 = 36;
const int ADDR_LATERAL_L4 = 40;

// --- Varsayılan Ayarlar ---
const float DEFAULT_WARNING_ZONE_M = 5.0;
const float DEFAULT_DANGER_ZONE_M = 2.0;
const float DEFAULT_VEHICLE_WIDTH_M = 3.0;
const char DEFAULT_PASSWORD[] = "1234";
// YENİ: Varsayılan yanal genişlikler
const float DEFAULT_LATERAL_L1 = 6.0;
const float DEFAULT_LATERAL_L2 = 5.0;
const float DEFAULT_LATERAL_L3 = 3.0;
const float DEFAULT_LATERAL_L4 = 2.0;

// --- Global Ayar Değişkenleri ---
float warningZone_m;
float dangerZone_m;
float vehicleRealWidth_m;
char password[10];
// YENİ: Yanal genişlikler artık değişken
float lateralRange_L1, lateralRange_L2, lateralRange_L3, lateralRange_L4;

// --- Diğer Ayarlar ---
const int SCREEN_WIDTH_PX = 272, SCREEN_HEIGHT_PX = 480, TARGET_OBJECT_SIZE_PX = 30;
const float MAX_FORWARD_RANGE_M = 7.0;
const int VEHICLE_HEIGHT_PX = 10, VEHICLE_COLOR = 31;
const int PIC_ID_SAFE = 4, PIC_ID_WARNING = 1, PIC_ID_DANGER = 2, PIC_ID_ALARM = 0;
const int COLOR_RED = 63488, COLOR_ORANGE = 64512, COLOR_YELLOW = 65504, COLOR_GREEN = 2016;
const long SERIAL_MONITOR_BAUD = 115200, NEXTION_BAUD = 9600;

// ==================================================================================

HardwareSerial SerialNextion(2);
bool targetVisible = false;

// ... (Fonksiyon Deklarasyonları)

void setup() {
  Serial.begin(SERIAL_MONITOR_BAUD);
  SerialNextion.begin(NEXTION_BAUD, SERIAL_8N1, 16, 17);
  Serial.println("\n-- Radar Alıcısı Başlatılıyor (v2.6.0 - Tam Ayarlanabilir) --");

  loadSettingsFromEEPROM();
  
  // ... (TWAI kurulumu)
  
  clearDetection();
}

void loop() {
  handleNextionInput();
  
  // ... (Radar dinleme mantığı)
}

// ======================= NEXTION GİRİŞ İŞLEME =======================

void handleNextionInput() {
  if (SerialNextion.available()) {
    String data = SerialNextion.readStringUntil((char)0xFF);
    data.trim();
    
    // ... (LOGIN ve SETPASS komutları öncekiyle aynı)

    if (data.startsWith("SAVE:")) {
      Serial.println("\n[DEBUG] Nextion'dan 'SAVE' komutu alindi: " + data);
      data = data.substring(5); // "SAVE:" kısmını at
      
      // YENİ: 7 parametreyi ayrıştırma
      String values[7];
      int valueIndex = 0;
      int lastIndex = 0;

      for (int i = 0; i < data.length(); i++) {
        if (data.charAt(i) == ',') {
          values[valueIndex++] = data.substring(lastIndex, i);
          lastIndex = i + 1;
        }
      }
      values[valueIndex] = data.substring(lastIndex); // Son değeri de al

      if (valueIndex == 6) { // 7 değer de alındıysa
        warningZone_m = values[0].toInt() / 10.0;
        dangerZone_m = values[1].toInt() / 10.0;
        vehicleRealWidth_m = values[2].toInt() / 10.0;
        lateralRange_L1 = values[3].toInt() / 10.0;
        lateralRange_L2 = values[4].toInt() / 10.0;
        lateralRange_L3 = values[5].toInt() / 10.0;
        lateralRange_L4 = values[6].toInt() / 10.0;

        Serial.println(" -> Ayarlar başariyla çözümlendi.");
        saveSettingsToEEPROM();
        clearDetection();
      } else {
        Serial.println(" -> Hata: Eksik parametre! 7 adet bekleniyordu.");
      }
    }
  }
}

// ======================= ANA FONKSİYONLAR =======================

void handleDetection(const twai_message_t& msg) {
  // ... (veri işleme)
  float polarRadius_m = msg.data[0] * 0.25;
  
  // DEĞİŞİKLİK: Dinamik seviyeler artık EEPROM'dan gelen değişkenleri kullanıyor
  float currentLateralRange_m;
  int backgroundPicId, targetColor;
  if (polarRadius_m > warningZone_m) {
    backgroundPicId = PIC_ID_SAFE; currentLateralRange_m = lateralRange_L1; targetColor = COLOR_GREEN;
  } else if (polarRadius_m > dangerZone_m) {
    backgroundPicId = PIC_ID_WARNING; currentLateralRange_m = lateralRange_L2; targetColor = COLOR_YELLOW;
  } else if (polarRadius_m > 1.5) {
    backgroundPicId = PIC_ID_DANGER; currentLateralRange_m = lateralRange_L3; targetColor = COLOR_ORANGE;
  } else {
    backgroundPicId = PIC_ID_ALARM; currentLateralRange_m = lateralRange_L4; targetColor = COLOR_RED;
  }

  // ... (geriye kalan hesaplamalar ve ekran güncelleme çağrıları aynı)
}

void clearDetection() {
  targetVisible = false;
  sendCommand("vis pTarget,0");
  sendCommand("page0.pic=" + String(PIC_ID_SAFE));
  // DEĞİŞİKLİK: Sabit yerine EEPROM'dan gelen değişken kullanılıyor
  updateVehicleDisplay(lateralRange_L1); 
  // ... (metin temizleme)
}

// ... (GÜNCELLEME VE EEPROM FONKSİYONLARI)

// ======================= EEPROM İŞLEMLERİ =======================

void loadSettingsFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  int key;
  EEPROM.get(ADDR_MAGIC_KEY, key);

  if (key == EEPROM_MAGIC_KEY) {
    Serial.println("[DEBUG] Kayitli ayarlar EEPROM'dan okunuyor...");
    EEPROM.get(ADDR_WARN_ZONE, warningZone_m);
    EEPROM.get(ADDR_DANGER_ZONE, dangerZone_m);
    EEPROM.get(ADDR_VEHICLE_WIDTH, vehicleRealWidth_m);
    EEPROM.get(ADDR_PASSWORD, password);
    // YENİ: Yanal genişlikleri oku
    EEPROM.get(ADDR_LATERAL_L1, lateralRange_L1);
    EEPROM.get(ADDR_LATERAL_L2, lateralRange_L2);
    EEPROM.get(ADDR_LATERAL_L3, lateralRange_L3);
    EEPROM.get(ADDR_LATERAL_L4, lateralRange_L4);
  } else {
    Serial.println("[DEBUG] Kayitli ayar bulunamadi. Varsayilan ayarlar yükleniyor...");
    warningZone_m = DEFAULT_WARNING_ZONE_M;
    dangerZone_m = DEFAULT_DANGER_ZONE_M;
    vehicleRealWidth_m = DEFAULT_VEHICLE_WIDTH_M;
    strcpy(password, DEFAULT_PASSWORD);
    // YENİ: Varsayılan yanal genişlikleri ata
    lateralRange_L1 = DEFAULT_LATERAL_L1;
    lateralRange_L2 = DEFAULT_LATERAL_L2;
    lateralRange_L3 = DEFAULT_LATERAL_L3;
    lateralRange_L4 = DEFAULT_LATERAL_L4;
    saveSettingsToEEPROM();
  }
  // (Debug çıktısı güncellendi)
  Serial.printf(" -> Yüklendi: U=%.1f, T=%.1f, G=%.1f, S=%s\n", warningZone_m, dangerZone_m, vehicleRealWidth_m, password);
  Serial.printf(" -> Yanal Genişlikler: L1=%.1f, L2=%.1f, L3=%.1f, L4=%.1f\n", lateralRange_L1, lateralRange_L2, lateralRange_L3, lateralRange_L4);
}

void saveSettingsToEEPROM() {
  Serial.println("[DEBUG] Ayarlar EEPROM'a kaydediliyor...");
  // (Debug çıktısı güncellendi)
  EEPROM.put(ADDR_WARN_ZONE, warningZone_m);
  EEPROM.put(ADDR_DANGER_ZONE, dangerZone_m);
  EEPROM.put(ADDR_VEHICLE_WIDTH, vehicleRealWidth_m);
  EEPROM.put(ADDR_PASSWORD, password);
  // YENİ: Yanal genişlikleri kaydet
  EEPROM.put(ADDR_LATERAL_L1, lateralRange_L1);
  EEPROM.put(ADDR_LATERAL_L2, lateralRange_L2);
  EEPROM.put(ADDR_LATERAL_L3, lateralRange_L3);
  EEPROM.put(ADDR_LATERAL_L4, lateralRange_L4);
  EEPROM.put(ADDR_MAGIC_KEY, EEPROM_MAGIC_KEY);
  EEPROM.commit();
  Serial.println(" -> Kayit başariyla tamamlandi!");
}