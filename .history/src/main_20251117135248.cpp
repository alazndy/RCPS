/*
 * =================================================================================
 * ESP32 & Nextion - CAN BUS Radar Projesi
 * ---------------------------------------------------------------------------------
 * Versiyon: v3.0.0 (Çok Sayfalı Ayarlar Menüsü - Tam Sürüm)
 * Yazar:    G.T
 * Tarih:    14.11.2025
 *
 * --- v3.0.0 Değişiklik Notları ---
 * - YENİ ÖZELLİK (ANA): Üç sayfalı, modüler ayarlar menüsü entegrasyonu.
 * - YENİ ÖZELLİK: Yan Algılama Payı (sideMargin_m) eklendi.
 * - YENİ ÖZELLİK: Maks. Genişlik (maxWidth_m) eklendi (Otomatik Zoom kapalıyken kullanılır).
 * - YENİ ÖZELLİK: Otomatik Zoom açma/kapatma (autoZoom_enabled).
 * - YENİ ÖZELLİK: Sesli Alarm açma/kapatma (audioAlarm_enabled).
 * - YENİ ÖZELLİK: Tüm ayarları varsayılana sıfırlama ('RESETALL' komutu).
 * - YENİ ÖZELLİK: İki yönlü iletişim. Cihaz açılışında kayıtlı ayarlar Nextion'a gönderilir.
 * - GÜVENLİK: Şifre girişi, her tuş basımının ESP32'ye gönderildiği daha güvenli bir
 *             yöntemle güncellendi.
 * - GÜNCELLEME: EEPROM yapısı tüm yeni ayarları içerecek şekilde genişletildi.
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
const int ADDR_AUTOZOOM_EN = 44, ADDR_AUDIOALARM_EN = 45;
const int ADDR_SIDE_MARGIN = 48, ADDR_MAX_WIDTH = 52;

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
void sendSettingsToNextion();

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
        sendSettingsToNextion(); // Sayfaya gitmeden önce ayarları gönder
        sendCommand("page pageSet1");
      } else {
        Serial.println(" -> Şifre yanliş.");
        sendCommand("tInfo.txt=\"Hatalı Şifre\"");
      }
      enteredPassword = "";
    }
    else if (data.startsWith("SETPASS:")) {
      String newPass = data.substring(8);
      if (newPass.length() > 0 && newPass.length() < 10) {
        strcpy(password, newPass.c_str());
        saveSettingsToEEPROM();
      }
    }
    else if (data.startsWith("SAVE1:")) {
      data = data.substring(6);
      int comma = data.indexOf(',');
      if (comma > 0) {
        warningZone_m = data.substring(0, comma).toFloat() / 10.0;
        dangerZone_m = data.substring(comma + 1).toFloat() / 10.0;
        saveSettingsToEEPROM();
      }
    }
    else if (data.startsWith("SAVE2:")) {
      data = data.substring(6);
      String values[3];
      int valueIndex = 0;
      int lastIndex = 0;
      for (int i = 0; i < data.length(); i++) {
        if (data.charAt(i) == ',') {
          if (valueIndex < 2) values[valueIndex++] = data.substring(lastIndex, i);
          lastIndex = i + 1;
        }
      }
      values[valueIndex] = data.substring(lastIndex);
      if (valueIndex == 2) {
        sideMargin_m = values[0].toFloat() / 10.0;
        vehicleRealWidth_m = values[1].toFloat() / 10.0;
        maxWidth_m = values[2].toFloat() / 10.0;
        saveSettingsToEEPROM();
      }
    }
    else if (data.startsWith("SAVE3:")) {
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
  int   polarAngle_deg = (int)msg.data[1] - 128;
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
  
  float scale_x = doc_y_m / currentLateralRange_m;
  int targetX_px = (int)(((float)SCREEN_WIDTH_PX / 2.0) + (scale_x * ((float)SCREEN_WIDTH_PX / 2.0)) + 0.5);
  int targetY_px = (int)(((MAX_FORWARD_RANGE_M - doc_x_m) / MAX_FORWARD_RANGE_M) * SCREEN_HEIGHT_PX);
  targetX_px = constrain(targetX_px, 0, SCREEN_WIDTH_PX - TARGET_OBJECT_SIZE_PX);
  targetY_px = constrain(targetY_px, 0, SCREEN_HEIGHT_PX - TARGET_OBJECT_SIZE_PX);
  
  sendCommand("page0.pic=" + String(backgroundPicId));
  updateVehicleDisplay(currentLateralRange_m);
  updateTargetDisplay(targetX_px, targetY_px, targetColor);
  updateTextDisplays(polarRadius_m, polarAngle_deg, doc_y_m, doc_x_m);
}

void clearDetection() {
  targetVisible = false;
  digitalWrite(BUZZER_PIN, LOW);
  sendCommand("vis pTarget,0");
  sendCommand("page0.pic=" + String(PIC_ID_SAFE));
  updateVehicleDisplay(autoZoom_enabled ? lateralRange_L1 : maxWidth_m / 2.0);
  sendCommand("tDurum.txt=\"Temiz\"");
  sendCommand("tMesafe.txt=\"--\"");
  sendCommand("tAci.txt=\"--\"");
  sendCommand("tX.txt=\"--\"");
  sendCommand("tY.txt=\"--\"");
}

// ======================= GÜNCELLEME FONKSİYONLARI =======================

void updateVehicleDisplay(float currentLateralRange_m) {
  float pixels_per_meter = (float)SCREEN_WIDTH_PX / (currentLateralRange_m * 2.0);
  int vehicle_width_px = (int)((vehicleRealWidth_m * pixels_per_meter) + 0.5);
  int vehicle_x_px = (int)(((float)SCREEN_WIDTH_PX - vehicle_width_px) / 2.0 + 0.5);
  int vehicle_y_px = SCREEN_HEIGHT_PX - VEHICLE_HEIGHT_PX;

  sendCommand("rVehicle.w=" + String(vehicle_width_px));
  delay(10);
  sendCommand("rVehicle.x=" + String(vehicle_x_px));
  sendCommand("rVehicle.y=" + String(vehicle_y_px));
  sendCommand("rVehicle.h=" + String(VEHICLE_HEIGHT_PX));
  sendCommand("rVehicle.bco=" + String(VEHICLE_COLOR));
}

void updateTargetDisplay(int x, int y, int color) {
  if (!targetVisible) {
    sendCommand("vis pTarget,1");
    targetVisible = true;
  }
  sendCommand("pTarget.bco=" + String(color));
  sendCommand("pTarget.x=" + String(x));
  sendCommand("pTarget.y=" + String(y));
}

void updateTextDisplays(float radius, int angle, float x_m, float y_m) {
  sendCommand("tDurum.txt=\"Algilandi\"");
  sendCommand("tMesafe.txt=\"" + String(radius, 2) + " m\"");
  sendCommand("tAci.txt=\"" + String(angle) + " derece\"");
  sendCommand("tX.txt=\"X: " + String(x_m, 2) + " m\"");
  sendCommand("tY.txt=\"Y: " + String(y_m, 2) + " m\"");
}

// ======================= EEPROM İŞLEMLERİ =======================

void loadSettingsFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(ADDR_MAGIC_KEY) == EEPROM_MAGIC_KEY) {
    Serial.println("[DEBUG] Kayitli ayarlar EEPROM'dan okunuyor...");
    EEPROM.get(ADDR_WARN_ZONE, warningZone_m);
    EEPROM.get(ADDR_DANGER_ZONE, dangerZone_m);
    EEPROM.get(ADDR_VEHICLE_WIDTH, vehicleRealWidth_m);
    EEPROM.get(ADDR_PASSWORD, password);
    EEPROM.get(ADDR_LATERAL_L1, lateralRange_L1);
    EEPROM.get(ADDR_LATERAL_L2, lateralRange_L2);
    EEPROM.get(ADDR_LATERAL_L3, lateralRange_L3);
    EEPROM.get(ADDR_LATERAL_L4, lateralRange_L4);
    EEPROM.get(ADDR_AUTOZOOM_EN, autoZoom_enabled);
    EEPROM.get(ADDR_AUDIOALARM_EN, audioAlarm_enabled);
    EEPROM.get(ADDR_SIDE_MARGIN, sideMargin_m);
    EEPROM.get(ADDR_MAX_WIDTH, maxWidth_m);
  } else {
    Serial.println("[DEBUG] Kayitli ayar bulunamadi. Varsayilan ayarlar yükleniyor...");
    resetToDefaults();
    return;
  }
  
  sendSettingsToNextion();
}

void saveSettingsToEEPROM() {
  Serial.println("[DEBUG] Ayarlar EEPROM'a kaydediliyor...");
  EEPROM.put(ADDR_WARN_ZONE, warningZone_m);
  EEPROM.put(ADDR_DANGER_ZONE, dangerZone_m);
  EEPROM.put(ADDR_VEHICLE_WIDTH, vehicleRealWidth_m);
  EEPROM.put(ADDR_PASSWORD, password);
  EEPROM.put(ADDR_LATERAL_L1, lateralRange_L1);
  EEPROM.put(ADDR_LATERAL_L2, lateralRange_L2);
  EEPROM.put(ADDR_LATERAL_L3, lateralRange_L3);
  EEPROM.put(ADDR_LATERAL_L4, lateralRange_L4);
  EEPROM.put(ADDR_AUTOZOOM_EN, autoZoom_enabled);
  EEPROM.put(ADDR_AUDIOALARM_EN, audioAlarm_enabled);
  EEPROM.put(ADDR_SIDE_MARGIN, sideMargin_m);
  EEPROM.put(ADDR_MAX_WIDTH, maxWidth_m);
  EEPROM.put(ADDR_MAGIC_KEY, EEPROM_MAGIC_KEY);
  EEPROM.commit();
  Serial.println(" -> Kayit başariyla tamamlandi!");
}

void resetToDefaults() {
    warningZone_m = DEFAULT_WARNING_ZONE_M;
    dangerZone_m = DEFAULT_DANGER_ZONE_M;
    vehicleRealWidth_m = DEFAULT_VEHICLE_WIDTH_M;
    strcpy(password, DEFAULT_PASSWORD);
    lateralRange_L1 = DEFAULT_LATERAL_L1;
    lateralRange_L2 = DEFAULT_LATERAL_L2;
    lateralRange_L3 = DEFAULT_LATERAL_L3;
    lateralRange_L4 = DEFAULT_LATERAL_L4;
    autoZoom_enabled = DEFAULT_AUTOZOOM_EN;
    audioAlarm_enabled = DEFAULT_AUDIOALARM_EN;
    sideMargin_m = DEFAULT_SIDE_MARGIN_M;
    maxWidth_m = DEFAULT_MAX_WIDTH_M;
    saveSettingsToEEPROM();
    loadSettingsFromEEPROM(); // Sıfırlanmış ayarları Nextion'a gönder
}

void sendSettingsToNextion() {
  Serial.println("[DEBUG] Mevcut ayarlar Nextion'a gönderiliyor...");
  sendCommand("pageSet1.h0.val=" + String((int)(warningZone_m * 10)));
  sendCommand("pageSet1.h1.val=" + String((int)(dangerZone_m * 10)));
  sendCommand("pageSet2.h0.val=" + String((int)(sideMargin_m * 10)));
  sendCommand("pageSet2.h1.val=" + String((int)(vehicleRealWidth_m * 10)));
  sendCommand("pageSet2.h2.val=" + String((int)(maxWidth_m * 10)));
  sendCommand("pageSet3.btZoom.val=" + String(autoZoom_enabled ? 1 : 0));
  sendCommand("pageSet3.btAudio.val=" + String(audioAlarm_enabled ? 1 : 0));