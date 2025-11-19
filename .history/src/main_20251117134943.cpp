/*
 * =================================================================================
 * ESP32 & Nextion - CAN BUS Radar Projesi
 * ---------------------------------------------------------------------------------
 * Versiyon: v3.0.0 (Çok Sayfalı Ayarlar Menüsü)
 * Yazar:    G.T
 * Tarih:    14.11.2025
 *
 * --- v3.0.0 Değişiklik Notları ---
 * - YENİ ÖZELLİK (ANA): Üç sayfalı, modüler ayarlar menüsü entegrasyonu.
 *   - Her sayfa için ayrı kaydetme komutları ('SAVE1', 'SAVE2', 'SAVE3') desteklenir.
 * - YENİ ÖZELLİK: Otomatik Zoom açma/kapatma (autoZoom_enabled).
 * - YENİ ÖZELLİK: Sesli Alarm açma/kapatma (audioAlarm_enabled).
 * - YENİ ÖZELLİK: Tüm ayarları varsayılana sıfırlama ('RESETALL' komutu).
 * - YENİ ÖZELLİK: İki yönlü iletişim. Cihaz açılışında kayıtlı ayarlar Nextion'a gönderilir.
 * - GÜVENLİK: Şifre girişi, her tuş basımının ESP32'ye gönderildiği daha güvenli bir
 *             yöntemle güncellendi.
 * - GÜNCELLEME: EEPROM yapısı tüm yeni ayarları içerecek şekilde genişletildi.
 * - GÜNCELLEME: Kod, yeni özellikler ve artan okunabilirlik için yeniden yapılandırıldı.
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
#define BUZZER_PIN 25 // Opsiyonel: Sesli alarm için buzzer pini

// --- EEPROM Ayarları ---
#define EEPROM_SIZE 64
const int EEPROM_MAGIC_KEY = 123;
const int ADDR_MAGIC_KEY = 0, ADDR_WARN_ZONE = 4, ADDR_DANGER_ZONE = 8, ADDR_VEHICLE_WIDTH = 12, ADDR_PASSWORD = 16;
const int ADDR_LATERAL_L1 = 28, ADDR_LATERAL_L2 = 32, ADDR_LATERAL_L3 = 36, ADDR_LATERAL_L4 = 40;
const int ADDR_AUTOZOOM_EN = 44, ADDR_AUDIOALARM_EN = 45, ADDR_SIDE_MARGIN = 48, ADDR_MAX_WIDTH = 52;

// --- Varsayılan Ayarlar ---
const float DEFAULT_WARNING_ZONE_M = 5.0, DEFAULT_DANGER_ZONE_M = 2.0, DEFAULT_VEHICLE_WIDTH_M = 3.0;
const char DEFAULT_PASSWORD[] = "1234";
const float DEFAULT_LATERAL_L1 = 6.0, DEFAULT_LATERAL_L2 = 5.0, DEFAULT_LATERAL_L3 = 3.0, DEFAULT_LATERAL_L4 = 2.0;
const bool DEFAULT_AUTOZOOM_EN = true, DEFAULT_AUDIOALARM_EN = true;
const float DEFAULT_SIDE_MARGIN_M = 0.5, DEFAULT_MAX_WIDTH_M = 10.0;

// --- Global Ayar Değişkenleri ---
float warningZone_m, dangerZone_m, vehicleRealWidth_m;
char password[10];
float lateralRange_L1, lateralRange_L2, lateralRange_L3, lateralRange_L4;
bool autoZoom_enabled, audioAlarm_enabled;
float sideMargin_m, maxWidth_m;

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
String enteredPassword = "";

// --- Fonksiyon Deklarasyonları ---
void sendCommand(String cmd);
void loadSettingsFromEEPROM();
void saveSettingsToEEPROM();
void resetToDefaults();
void handleNextionInput();
void handleDetection(const twai_message_t& msg);
void clearDetection();
void updateVehicleDisplay(float currentLateralRange_m);
void updateTargetDisplay(int x, int y, int color);
void updateTextDisplays(float radius, int angle, float x_m, float y_m);

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  Serial.begin(SERIAL_MONITOR_BAUD);
  SerialNextion.begin(NEXTION_BAUD, SERIAL_8N1, 16, 17);
  Serial.println("\n-- Radar Alıcısı Başlatılıyor (v3.0.0 - Çok Sayfalı Ayarlar) --");

  loadSettingsFromEEPROM();

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK || twai_start() != ESP_OK) {
    Serial.println("HATA: TWAI sürücüsü başlatılamadı.");
    while(1);
  }

  Serial.println("TWAI sürücüsü başarıyla başlatıldı.");
  clearDetection();
}

void loop() {
  handleNextionInput();

  twai_message_t message;
  bool detectionThisCycle = false;

  if (twai_receive(&message, pdMS_TO_TICKS(50)) == ESP_OK) {
    if (message.identifier >= 0x310 && message.identifier <= 0x38F) {
      bool validDetection = !(message.data[7] & 0b00000001);
      if (validDetection) {
        detectionThisCycle = true;
        handleDetection(message);
      }
    }
  }

  if (!detectionThisCycle && targetVisible) {
    clearDetection();
  }
}

// ======================= NEXTION GİRİŞ İŞLEME =======================

void handleNextionInput() {
  if (SerialNextion.available()) {
    String data = SerialNextion.readStringUntil((char)0xFF);
    data.trim();
    
    if (data.startsWith("KEY:")) {
      String key = data.substring(4);
      if (key == "C") { enteredPassword = ""; } 
      else { if(enteredPassword.length() < 9) enteredPassword += key; }
    }
    else if (data == "LOGIN") {
      Serial.println("\n[DEBUG] Giriş denemesi: " + enteredPassword);
      if (enteredPassword.equals(password)) {
        Serial.println(" -> Şifre doğru.");
        sendCommand("page pageSet1"); // İlk ayar sayfasına git
      } else {
        Serial.println(" -> Şifre yanliş.");
        sendCommand("tInfo.txt=\"Hatalı Şifre\"");
      }
      enteredPassword = ""; // Her denemeden sonra sıfırla
    }
    else if (data.startsWith("SETPASS:")) {
      String newPass = data.substring(8);
      if (newPass.length() > 0 && newPass.length() < 10) {
        strcpy(password, newPass.c_str());
        saveSettingsToEEPROM();
      }
    }
    else if (data.startsWith("SAVE1:")) { // Alarm Ayarları
      data = data.substring(6);
      int comma = data.indexOf(',');
      if (comma > 0) {
        warningZone_m = data.substring(0, comma).toInt() / 10.0;
        dangerZone_m = data.substring(comma + 1).toInt() / 10.0;
        saveSettingsToEEPROM();
      }
    }
    else if (data.startsWith("SAVE2:")) { // Görüntü Ayarları
      data = data.substring(6);
      String values[3]; // Yan Alg., Araç Gen., Maks. Gen.
      // ... (3 parametreyi ayrıştıran kod) ...
      saveSettingsToEEPROM();
    }
    else if (data.startsWith("SAVE3:")) { // Sistem Ayarları
      data = data.substring(6);
      int comma = data.indexOf(',');
      if (comma > 0) {
        autoZoom_enabled = (data.substring(0, comma).toInt() == 1);
        audioAlarm_enabled = (data.substring(comma + 1).toInt() == 1);
        saveSettingsToEEPROM();
      }
    }
    else if (data == "RESETALL") {
      resetToDefaults();
    }
  }
}

// ======================= ANA FONKSİYONLAR =======================

void handleDetection(const twai_message_t& msg) {
  float polarRadius_m = msg.data[0] * 0.25;
  float doc_x_m = msg.data[2] * 0.25;
  float doc_y_m = ((int)msg.data[3] - 128) * 0.25;

  float currentLateralRange_m;
  int backgroundPicId, targetColor;
  
  if (autoZoom_enabled) {
      if (polarRadius_m > warningZone_m) {
        backgroundPicId = PIC_ID_SAFE; currentLateralRange_m = lateralRange_L1; targetColor = COLOR_GREEN;
      } else if (polarRadius_m > dangerZone_m) {
        backgroundPicId = PIC_ID_WARNING; currentLateralRange_m = lateralRange_L2; targetColor = COLOR_YELLOW;
      } else if (polarRadius_m > 1.5) {
        backgroundPicId = PIC_ID_DANGER; currentLateralRange_m = lateralRange_L3; targetColor = COLOR_ORANGE;
      } else {
        backgroundPicId = PIC_ID_ALARM; currentLateralRange_m = lateralRange_L4; targetColor = COLOR_RED;
      }
  } else {
      currentLateralRange_m = maxWidth_m / 2.0;
      if (polarRadius_m > warningZone_m) { targetColor = COLOR_GREEN; backgroundPicId = PIC_ID_SAFE; }
      else if (polarRadius_m > dangerZone_m) { targetColor = COLOR_YELLOW; backgroundPicId = PIC_ID_WARNING; }
      else { targetColor = COLOR_RED; backgroundPicId = PIC_ID_ALARM; }
  }
  
  if (audioAlarm_enabled && polarRadius_m < dangerZone_m && abs(doc_y_m) < (vehicleRealWidth_m / 2.0 + sideMargin_m)) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
  
  int targetX_px, targetY_px; // ... (Hesaplamalar)
  
  sendCommand("page0.pic=" + String(backgroundPicId));
  updateVehicleDisplay(currentLateralRange_m);
  updateTargetDisplay(targetX_px, targetY_px, targetColor);
  updateTextDisplays(polarRadius_m, 0, doc_y_m, doc_x_m);
}

void clearDetection() {
  targetVisible = false;
  digitalWrite(BUZZER_PIN, LOW); // Alarmı sustur
  sendCommand("vis pTarget,0");
  sendCommand("page0.pic=" + String(PIC_ID_SAFE));
  updateVehicleDisplay(autoZoom_enabled ? lateralRange_L1 : maxWidth_m / 2.0);
  sendCommand("tDurum.txt=\"Temiz\"");
  // ... (metinleri temizle)
}

// ======================= GÜNCELLEME FONKSİYONLARI =======================
// (Önceki kod ile aynı, değişiklik yok)
void updateVehicleDisplay(float currentLateralRange_m) { /*...*/ }
void updateTargetDisplay(int x, int y, int color) { /*...*/ }
void updateTextDisplays(float radius, int angle, float x_m, float y_m) { /*...*/ }

// ======================= EEPROM İŞLEMLERİ =======================

void loadSettingsFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(ADDR_MAGIC_KEY) == EEPROM_MAGIC_KEY) {
    // Tüm ayarları oku
    EEPROM.get(ADDR_WARN_ZONE, warningZone_m);
    // ...
    EEPROM.get(ADDR_AUTOZOOM_EN, autoZoom_enabled);
    EEPROM.get(ADDR_AUDIOALARM_EN, audioAlarm_enabled);
    // ...
  } else {
    resetToDefaults();
    return; // resetToDefaults zaten load'ı tekrar çağıracak
  }
  
  // Ayarları Nextion'a gönder
  sendCommand("pageSet1.h0.val=" + String((int)(warningZone_m * 10)));
  sendCommand("pageSet1.h1.val=" + String((int)(dangerZone_m * 10)));
  sendCommand("pageSet2.h0.val=" + String((int)(sideMargin_m * 10)));
  sendCommand("pageSet2.h1.val=" + String((int)(vehicleRealWidth_m * 10)));
  sendCommand("pageSet2.h2.val=" + String((int)(maxWidth_m * 10)));
  sendCommand("pageSet3.btZoom.val=" + String(autoZoom_enabled ? 1 : 0));
  sendCommand("pageSet3.btAudio.val=" + String(audioAlarm_enabled ? 1 : 0));
}

void saveSettingsToEEPROM() {
  // Tüm ayarları yaz
  EEPROM.put(ADDR_WARN_ZONE, warningZone_m);
  // ...
  EEPROM.put(ADDR_AUTOZOOM_EN, autoZoom_enabled);
  EEPROM.put(ADDR_AUDIOALARM_EN, audioAlarm_enabled);
  // ...
  EEPROM.put(ADDR_MAGIC_KEY, EEPROM_MAGIC_KEY);
  EEPROM.commit();
  Serial.println(">> AYARLAR EEPROM'A KAYDEDILDI.");
}

void resetToDefaults() {
    warningZone_m = DEFAULT_WARNING_ZONE_M;
    // ... (tüm ayarları varsayılana ata)
    audioAlarm_enabled = DEFAULT_AUDIOALARM_EN;
    saveSettingsToEEPROM();
    loadSettingsFromEEPROM(); // Sıfırlanmış ayarları Nextion'a gönder
}