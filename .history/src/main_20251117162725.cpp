/*
 * =================================================================================
 * ESP32 & Nextion - CAN BUS Radar Projesi
 * ---------------------------------------------------------------------------------
 * Versiyon: v2.5.1 (Düzeltilmiş - Tam Sürüm)
 * Yazar:    G.T
 * Tarih:    14.11.2025
 *
 * --- v2.5.1 Değişiklik Notları ---
 * - DÜZELTME (Kritik): Bir önceki versiyonda yanlışlıkla içi boş olarak verilen
 *             updateVehicleDisplay(), updateTargetDisplay() ve updateTextDisplays()
 *             fonksiyonlarının gövdeleri düzeltildi. Bu, radar verilerinin
 *             ekrana tekrar yansıtılmasını sağlar.
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
#define EEPROM_SIZE 32
const int EEPROM_MAGIC_KEY = 123;
const int ADDR_MAGIC_KEY = 0, ADDR_WARN_ZONE = 4, ADDR_DANGER_ZONE = 8, ADDR_VEHICLE_WIDTH = 12, ADDR_PASSWORD = 16;

// --- Varsayılan Ayarlar ---
const float DEFAULT_WARNING_ZONE_M = 5.0;
const float DEFAULT_DANGER_ZONE_M = 2.0;
const float DEFAULT_VEHICLE_WIDTH_M = 3.0;
const char DEFAULT_PASSWORD[] = "1234";

// --- Global Ayar Değişkenleri ---
float warningZone_m;
float dangerZone_m;
float vehicleRealWidth_m;
char password[10];

// --- Diğer Ayarlar ---
const int SCREEN_WIDTH_PX = 272, SCREEN_HEIGHT_PX = 480, TARGET_OBJECT_SIZE_PX = 30;
const float MAX_FORWARD_RANGE_M = 7.0;
const int VEHICLE_HEIGHT_PX = 10, VEHICLE_COLOR = 31;
const float LATERAL_RANGE_LEVEL_1 = 6.0, LATERAL_RANGE_LEVEL_2 = 5.0, LATERAL_RANGE_LEVEL_3 = 3.0, LATERAL_RANGE_LEVEL_4 = 2.0;
const int PIC_ID_SAFE = 4, PIC_ID_WARNING = 1, PIC_ID_DANGER = 2, PIC_ID_ALARM = 0;
const int COLOR_RED = 63488, COLOR_ORANGE = 64512, COLOR_YELLOW = 65504, COLOR_GREEN = 2016;
const long SERIAL_MONITOR_BAUD = 115200, NEXTION_BAUD = 9600;

// ==================================================================================

HardwareSerial SerialNextion(2);
bool targetVisible = false;

// --- Fonksiyon Deklarasyonları ---
void saveSettingsToEEPROM();
void loadSettingsFromEEPROM();
void handleDetection(const twai_message_t& msg);
void clearDetection();
void updateVehicleDisplay(float currentLateralRange_m);
void updateTargetDisplay(int x, int y, int color);
void updateTextDisplays(float radius, int angle, float x_m, float y_m);
void handleNextionInput();

void sendCommand(String cmd) {
  SerialNextion.print(cmd);
  SerialNextion.write(0xFF);
  SerialNextion.write(0xFF);
  SerialNextion.write(0xFF);
}

void setup() {
  Serial.begin(SERIAL_MONITOR_BAUD);
  SerialNextion.begin(NEXTION_BAUD, SERIAL_8N1, 16, 17);
  Serial.println("\n-- Radar Alıcısı Başlatılıyor (v2.5.1 - Düzeltilmiş) --");

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
    
    if (data.startsWith("LOGIN:")) {
      String enteredPass = data.substring(6);
      Serial.println("\n[DEBUG] Nextion'dan 'LOGIN' komutu alindi. Şifre: " + enteredPass);
      if (enteredPass.equals(password)) {
        Serial.println(" -> Şifre doğru. Ayarlar sayfasina yönlendiriliyor.");
        sendCommand("page pageSet");
      } else {
        Serial.println(" -> Şifre yanliş.");
        sendCommand("tInfo.txt=\"Hatalı Şifre\"");
        sendCommand("login_fail.val=1");
      }
    }
    else if (data.startsWith("SETPASS:")) {
      String newPass = data.substring(8);
      Serial.println("\n[DEBUG] Nextion'dan 'SETPASS' komutu alindi. Yeni Şifre: " + newPass);
      if (newPass.length() > 0 && newPass.length() < 10) {
        strcpy(password, newPass.c_str());
        saveSettingsToEEPROM();
        sendCommand("tInfoSet.txt=\"Şifre Kaydedildi\"");
      } else {
        Serial.println(" -> Hata: Geçersiz şifre formati.");
        sendCommand("tInfoSet.txt=\"Hata: Geçersiz Şifre\"");
      }
    }
    else if (data.startsWith("SAVE:")) {
      Serial.println("\n[DEBUG] Nextion'dan 'SAVE' komutu alindi: " + data);
      data = data.substring(5);
      
      int firstComma = data.indexOf(',');
      if (firstComma > 0) {
        warningZone_m = data.substring(0, firstComma).toInt() / 10.0;
        data = data.substring(firstComma + 1);
        int secondComma = data.indexOf(',');
        if (secondComma > 0) {
          dangerZone_m = data.substring(0, secondComma).toInt() / 10.0;
          vehicleRealWidth_m = data.substring(secondComma + 1).toInt() / 10.0;
          
          Serial.println(" -> Ayarlar başariyla çözümlendi.");
          saveSettingsToEEPROM();
          clearDetection();
        }
      }
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
  
  if (polarRadius_m > warningZone_m) {
    backgroundPicId = PIC_ID_SAFE; currentLateralRange_m = LATERAL_RANGE_LEVEL_1; targetColor = COLOR_GREEN;
  } else if (polarRadius_m > dangerZone_m) {
    backgroundPicId = PIC_ID_WARNING; currentLateralRange_m = LATERAL_RANGE_LEVEL_2; targetColor = COLOR_YELLOW;
  } else if (polarRadius_m > 1.5) {
    backgroundPicId = PIC_ID_DANGER; currentLateralRange_m = LATERAL_RANGE_LEVEL_3; targetColor = COLOR_ORANGE;
  } else {
    backgroundPicId = PIC_ID_ALARM; currentLateralRange_m = LATERAL_RANGE_LEVEL_4; targetColor = COLOR_RED;
  }

  float scale_x = doc_y_m / currentLateralRange_m;
  int targetX_px = (int)(((float)SCREEN_WIDTH_PX / 2.0) + (scale_x * ((float)SCREEN_WIDTH_PX / 2.0)) + 0.5);
  int targetY_px = (int)(((MAX_FORWARD_RANGE_M - doc_x_m) / MAX_FORWARD_RANGE_M) * SCREEN_HEIGHT_PX);
  targetX_px = constrain(targetX_px, 0, SCREEN_WIDTH_PX - TARGET_OBJECT_SIZE_PX);
  targetY_px = constrain(targetY_px, 0, SCREEN_HEIGHT_PX - TARGET_OBJECT_SIZE_PX);
  
  Serial.println("\n--- NESNE ALGILANDI ---");
  Serial.printf("  - Mesafe: %.2fm -> Zoom Seviyesi (Grid ID): %d\n", polarRadius_m, backgroundPicId);
  Serial.printf("  - Aktif Yanal Genişlik: %.1fm\n", currentLateralRange_m * 2);
  Serial.printf("  - Hedef Koor. (Metre): Yanal=%.2fm, İleri=%.2fm\n", doc_y_m, doc_x_m);
  Serial.printf("  - Hedef Koor. (Piksel): X=%d, Y=%d\n", targetX_px, targetY_px);
  Serial.println("-------------------------");
  
  sendCommand("page0.pic=" + String(backgroundPicId));
  updateVehicleDisplay(currentLateralRange_m);
  updateTargetDisplay(targetX_px, targetY_px, targetColor);
  updateTextDisplays(polarRadius_m, polarAngle_deg, doc_y_m, doc_x_m);
}

void clearDetection() {
  targetVisible = false;
  sendCommand("vis pTarget,0");
  sendCommand("page0.pic=" + String(PIC_ID_SAFE));
  updateVehicleDisplay(LATERAL_RANGE_LEVEL_1);
  sendCommand("tDurum.txt=\"Temiz\"");
  sendCommand("tMesafe.txt=\"--\"");
  sendCommand("tAci.txt=\"--\"");
  sendCommand("tX.txt=\"--\"");
  sendCommand("tY.txt=\"--\"");
  Serial.println("\n>> Alan temizlendi, grid ve hedef sıfırlandı.");
}

// ======================= GÜNCELLEME FONKSİYONLARI (DÜZELTİLDİ) =======================

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
  int key;
  EEPROM.get(ADDR_MAGIC_KEY, key);

  if (key == EEPROM_MAGIC_KEY) {
    Serial.println("[DEBUG] Kayitli ayarlar EEPROM'dan okunuyor...");
    EEPROM.get(ADDR_WARN_ZONE, warningZone_m);
    EEPROM.get(ADDR_DANGER_ZONE, dangerZone_m);
    EEPROM.get(ADDR_VEHICLE_WIDTH, vehicleRealWidth_m);
    EEPROM.get(ADDR_PASSWORD, password);
  } else {
    Serial.println("[DEBUG] Kayitli ayar bulunamadi. Varsayilan ayarlar yükleniyor...");
    warningZone_m = DEFAULT_WARNING_ZONE_M;
    dangerZone_m = DEFAULT_DANGER_ZONE_M;
    vehicleRealWidth_m = DEFAULT_VEHICLE_WIDTH_M;
    strcpy(password, DEFAULT_PASSWORD);
    saveSettingsToEEPROM();
  }
  Serial.printf(" -> Yüklenen Ayarlar: Uyari=%.1f, Tehlike=%.1f, Genislik=%.1f, Sifre=%s\n", warningZone_m, dangerZone_m, vehicleRealWidth_m, password);
}

void saveSettingsToEEPROM() {
  Serial.println("[DEBUG] Ayarlar EEPROM'a kaydediliyor...");
  Serial.printf(" -> Kaydedilecek: Uyari=%.1f, Tehlike=%.1f, Genislik=%.1f, Sifre=%s\n", warningZone_m, dangerZone_m, vehicleRealWidth_m, password);
  EEPROM.put(ADDR_WARN_ZONE, warningZone_m);
  EEPROM.put(ADDR_DANGER_ZONE, dangerZone_m);
  EEPROM.put(ADDR_VEHICLE_WIDTH, vehicleRealWidth_m);
  EEPROM.put(ADDR_PASSWORD, password);
  EEPROM.put(ADDR_MAGIC_KEY, EEPROM_MAGIC_KEY);
  EEPROM.commit();
  Serial.println(" -> Kayit başariyla tamamlandi!");
}