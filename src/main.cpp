/*
 * =================================================================================
 * Proje Adı:   ESP32 & Nextion - Gelişmiş CAN BUS Radar Sistemi
 * ---------------------------------------------------------------------------------
 * Versiyon:    v3.5.0 (Ayrıntılı Hata Ayıklama Modu)
 * Yazar:       G.T (AI Tarafından Düzenlendi ve Yorumlandı)
 * Tarih:       18.11.2025
 *
 * --- v3.5.0 Değişiklik Notları ---
 * - ÖZELLİK (Gelişmiş Hata Ayıklama): Tek bir DEBUG_MODE anahtarı yerine, projenin
 *              her modülü (CAN, Nextion, Radar, Buzzer, EEPROM) için ayrı ayrı
 *              açılıp kapatılabilen bir hata ayıklama kontrol paneli eklendi.
 *              Bu, sorun tespiti sırasında sadece istenen bilgileri görmeyi sağlar.
 * - YAPI: Tüm debug mesajları, ilgili modülün kendi makrosuyla çağrılacak
 *         şekilde yeniden düzenlendi.
 * =================================================================================
 */

// =================================================================================
// BÖLÜM 1: GEREKLİ KÜTÜPHANELER
// =================================================================================
#include "driver/gpio.h"
#include "driver/twai.h"
#include <math.h>
#include <HardwareSerial.h>
#include <EEPROM.h>


// =================================================================================
// BÖLÜM 2: PROJE AYARLARI VE SABİTLER
// =================================================================================

// --- Ayrıntılı Hata Ayıklama (Debug) Ayarları ---
// İstenen modülün hata ayıklama mesajlarını açmak için 1, kapatmak için 0 yapın.
#define DEBUG_CAN      0  // Ham CAN bus mesajlarını gösterir.
#define DEBUG_NEXTION  1  // Nextion'dan gelen komutları ve işlemlerini gösterir.
#define DEBUG_RADAR    0  // Radar veri çözümleme, karar ve hesaplama adımlarını gösterir.
#define DEBUG_BUZZER   1  // Buzzer'ın durum değişikliklerini (AÇIK/KAPALI) gösterir.
#define DEBUG_EEPROM   1  // EEPROM okuma, yazma ve sıfırlama işlemlerini gösterir.

// Hata ayıklama modlarına göre Seri Monitör komutlarını tanımlayan makrolar.
#if DEBUG_CAN == 1
  #define CAN_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define CAN_PRINTF(...)
#endif
#if DEBUG_NEXTION == 1
  #define NEXTION_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define NEXTION_PRINTF(...)
#endif
#if DEBUG_RADAR == 1
  #define RADAR_PRINTLN(x) Serial.println(x)
  #define RADAR_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define RADAR_PRINTLN(x)
  #define RADAR_PRINTF(...)
#endif
#if DEBUG_BUZZER == 1
  #define BUZZER_PRINTLN(x) Serial.println(x)
#else
  #define BUZZER_PRINTLN(x)
#endif
#if DEBUG_EEPROM == 1
  #define EEPROM_PRINTLN(x) Serial.println(x)
  #define EEPROM_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define EEPROM_PRINTLN(x)
  #define EEPROM_PRINTF(...)
#endif


// --- Fiziksel Pin Tanımlamaları ---
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4
#define BUZZER_PIN 25

// (Kodun geri kalanı önceki versiyonlarla aynıdır, sadece debug komutları değiştirilmiştir)
// --- EEPROM Ayarları ---
#define EEPROM_SIZE 64
const int EEPROM_MAGIC_KEY = 124;
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

// --- Arayüz ve Diğer Sabitler ---
const int   SCREEN_WIDTH_PX = 272, SCREEN_HEIGHT_PX = 480, TARGET_OBJECT_SIZE_PX = 30;
const float MAX_FORWARD_RANGE_M = 7.0;
const int   VEHICLE_HEIGHT_PX = 10, VEHICLE_COLOR = 31;
const int   PIC_ID_SAFE = 4, PIC_ID_WARNING = 1, PIC_ID_DANGER = 2, PIC_ID_ALARM = 0;
const int   COLOR_RED = 63488, COLOR_ORANGE = 64512, COLOR_YELLOW = 65504, COLOR_GREEN = 2016;
const long  SERIAL_MONITOR_BAUD = 115200, NEXTION_BAUD = 9600;

// --- Park Sensörü / Kademeli Alarm için Sabitler ---
const float SOLID_TONE_DISTANCE_M = 0.75;
const int   BEEP_ON_DURATION_MS = 60;
const int   BEEP_INTERVAL_YELLOW_MS = 400;
const int   BEEP_INTERVAL_ORANGE_MS = 200;
const int   BEEP_INTERVAL_RED_MS = 80;


// =================================================================================
// BÖLÜM 3: GLOBAL NESNELER VE FONKSİYON DEKLARASYONLARI
// =================================================================================

HardwareSerial SerialNextion(2);
bool targetVisible = false;

// --- Park Sensörü / Bip Mantığı için Global Değişkenler ---
bool buzzerShouldBeActive = false;
bool buzzerIsOn           = false;
unsigned long lastBuzzerToggleTime = 0;
int  currentBeepInterval  = BEEP_INTERVAL_YELLOW_MS;

// --- Fonksiyon Deklarasyonları (Prototipler) ---
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
void handleBuzzer();


// =================================================================================
// BÖLÜM 4: ANA PROGRAM FONKSİYONLARI (setup ve loop)
// =================================================================================

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  Serial.begin(SERIAL_MONITOR_BAUD);
  SerialNextion.begin(NEXTION_BAUD, SERIAL_8N1, 16, 17);
  Serial.println("\n-- Radar Alıcısı Başlatılıyor (v3.5.0 - Ayrıntılı Debug) --");

  loadSettingsFromEEPROM();

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK || twai_start() != ESP_OK) {
    Serial.println("HATA: TWAI (CAN) sürücüsü başlatılamadı. Donanımı kontrol edin.");
    while(1);
  }

  Serial.println("[INFO] TWAI (CAN) sürücüsü başarıyla başlatıldı.");
  clearDetection();
}

void loop() {
  handleNextionInput();

  twai_message_t message;
  bool detectionThisCycle = false;
  if (twai_receive(&message, pdMS_TO_TICKS(50)) == ESP_OK) {
    CAN_PRINTF("[CAN] Mesaj alındı. ID: 0x%03X\n", message.identifier);
    
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

  handleBuzzer();
}


// =================================================================================
// BÖLÜM 5: HABERLEŞME FONKSİYONLARI
// =================================================================================

void sendCommand(String cmd) {
  SerialNextion.print(cmd);
  SerialNextion.write(0xFF);
  SerialNextion.write(0xFF);
  SerialNextion.write(0xFF);
}

void handleNextionInput() {
  if (SerialNextion.available()) {
    String data = SerialNextion.readStringUntil((char)0xFF);
    data.trim();
    NEXTION_PRINTF("\n[NEXTION] Ham veri alındı: \"%s\"\n", data.c_str());
    
    if (data.startsWith("LOGIN:")) {
      String enteredPass = data.substring(6);
      if (enteredPass.equals(password)) {
        NEXTION_PRINTF("[NEXTION] Şifre doğru.\n");
        sendSettingsToNextion();
        sendCommand("page pageSet1");
      } else {
        NEXTION_PRINTF("[NEXTION] Şifre yanlış.\n");
        sendCommand("tInfo.txt=\"Hatalı Şifre\"");
        sendCommand("login_fail.val=1");
      }
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
        NEXTION_PRINTF("[NEXTION] SAVE1 işlendi. Yeni Uyarı: %.1fm, Tehlike: %.1fm\n", warningZone_m, dangerZone_m);
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
        NEXTION_PRINTF("[NEXTION] SAVE2 işlendi. Yan Boşluk: %.1f, Genişlik: %.1f, Maks Genişlik: %.1f\n", sideMargin_m, vehicleRealWidth_m, maxWidth_m);
        saveSettingsToEEPROM();
      }
    }
    else if (data.startsWith("SAVE3:")) {
      data = data.substring(6);
      int comma = data.indexOf(',');
      if (comma > 0) {
        autoZoom_enabled = (data.substring(0, comma).toInt() == 1);
        audioAlarm_enabled = (data.substring(comma + 1).toInt() == 1);
        NEXTION_PRINTF("[NEXTION] SAVE3 işlendi. OtoZoom: %d, SesliAlarm: %d\n", autoZoom_enabled, audioAlarm_enabled);
        saveSettingsToEEPROM();
      }
    }
    else if (data == "RESETALL") {
      NEXTION_PRINTF("[NEXTION] RESETALL komutu alındı.\n");
      resetToDefaults();
    }
  }
}


// =================================================================================
// BÖLÜM 6: RADAR MANTIĞI VE GÖRSELLEŞTİRME
// =================================================================================

void handleDetection(const twai_message_t& msg) {
  RADAR_PRINTLN("\n--- [RADAR] HEDEF ALGILANDI ---");
  
  // 1. Veri Çevirme
  float polarRadius_m = msg.data[0] * 0.25;
  int   polarAngle_deg = (int)msg.data[1] - 128;
  float doc_x_m = msg.data[2] * 0.25;
  float doc_y_m = ((int)msg.data[3] - 128) * 0.25;
  RADAR_PRINTF("  [Veri]   Mesafe: %.2fm, Açı: %d°, İleri(X): %.2fm, Yanal(Y): %.2fm\n", polarRadius_m, polarAngle_deg, doc_x_m, doc_y_m);

  // 2. Görsel Mantık
  float currentLateralRange_m;
  int backgroundPicId, targetColor;
  if (autoZoom_enabled) {
      if (polarRadius_m > warningZone_m) { backgroundPicId = PIC_ID_SAFE; currentLateralRange_m = lateralRange_L1; targetColor = COLOR_GREEN; } 
      else if (polarRadius_m > dangerZone_m) { backgroundPicId = PIC_ID_WARNING; currentLateralRange_m = lateralRange_L2; targetColor = COLOR_YELLOW; } 
      else if (polarRadius_m > 1.5) { backgroundPicId = PIC_ID_DANGER; currentLateralRange_m = lateralRange_L3; targetColor = COLOR_ORANGE; } 
      else { backgroundPicId = PIC_ID_ALARM; currentLateralRange_m = lateralRange_L4; targetColor = COLOR_RED; }
  } else {
      currentLateralRange_m = maxWidth_m / 2.0;
      if (polarRadius_m > warningZone_m) { backgroundPicId = PIC_ID_SAFE; targetColor = COLOR_GREEN; }
      else if (polarRadius_m > dangerZone_m) { backgroundPicId = PIC_ID_WARNING; targetColor = COLOR_YELLOW; }
      else { backgroundPicId = PIC_ID_ALARM; targetColor = COLOR_RED; }
  }
  RADAR_PRINTF("  [Görsel] Tehlike Seviyesi ID: %d, Aktif Yanal Menzil: %.1fm\n", backgroundPicId, currentLateralRange_m * 2.0);
  
  // 3. Kademeli Buzzer Mantığı
  if (audioAlarm_enabled && (polarRadius_m < warningZone_m) && (abs(doc_y_m) < (vehicleRealWidth_m / 2.0 + sideMargin_m))) {
    buzzerShouldBeActive = true;
    if (polarRadius_m <= SOLID_TONE_DISTANCE_M) {
      currentBeepInterval = 0;
      RADAR_PRINTF("  [Buzzer] KARAR: AKTİF (Sürekli Ses, Mesafe %.2fm <= %.2fm)\n", polarRadius_m, SOLID_TONE_DISTANCE_M);
    } 
    else if (backgroundPicId == PIC_ID_ALARM) {
      currentBeepInterval = BEEP_INTERVAL_RED_MS;
      RADAR_PRINTF("  [Buzzer] KARAR: AKTİF (Kırmızı Alarm Bip Sesi, Aralık: %d ms)\n", currentBeepInterval);
    } 
    else if (backgroundPicId == PIC_ID_DANGER) {
      currentBeepInterval = BEEP_INTERVAL_ORANGE_MS;
      RADAR_PRINTF("  [Buzzer] KARAR: AKTİF (Turuncu Alarm Bip Sesi, Aralık: %d ms)\n", currentBeepInterval);
    } 
    else if (backgroundPicId == PIC_ID_WARNING) {
      currentBeepInterval = BEEP_INTERVAL_YELLOW_MS;
      RADAR_PRINTF("  [Buzzer] KARAR: AKTİF (Sarı Alarm Bip Sesi, Aralık: %d ms)\n", currentBeepInterval);
    }
  } else {
    buzzerShouldBeActive = false;
    RADAR_PRINTLN("  [Buzzer] KARAR: PASİF (Koşullar sağlanmadı veya Güvenli Bölge)");
  }

  // 4. Piksel Hesaplama
  if (currentLateralRange_m < 0.1) { currentLateralRange_m = DEFAULT_MAX_WIDTH_M / 2.0; }
  float scale_x = doc_y_m / currentLateralRange_m;
  int targetX_px = (int)(((float)SCREEN_WIDTH_PX / 2.0) + (scale_x * ((float)SCREEN_WIDTH_PX / 2.0)) + 0.5);
  int targetY_px = (int)(((MAX_FORWARD_RANGE_M - doc_x_m) / MAX_FORWARD_RANGE_M) * SCREEN_HEIGHT_PX);
  targetX_px = constrain(targetX_px, 0, SCREEN_WIDTH_PX - TARGET_OBJECT_SIZE_PX);
  targetY_px = constrain(targetY_px, 0, SCREEN_HEIGHT_PX - TARGET_OBJECT_SIZE_PX);
  RADAR_PRINTF("  [Ekran]  Hesaplanan Piksel Koor: X=%d, Y=%d\n", targetX_px, targetY_px);
  
  // 5. Ekranı Güncelleme
  sendCommand("page0.pic=" + String(backgroundPicId));
  updateVehicleDisplay(currentLateralRange_m);
  updateTargetDisplay(targetX_px, targetY_px, targetColor);
  updateTextDisplays(polarRadius_m, polarAngle_deg, doc_y_m, doc_x_m);
  RADAR_PRINTLN("---------------------------------");
}

void handleBuzzer() {
  if (!buzzerShouldBeActive) {
    if (buzzerIsOn) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerIsOn = false;
      BUZZER_PRINTLN("[BUZZER] DURUM: PASİF (Susturuldu)");
    }
    return;
  }

  unsigned long currentTime = millis();

  if (currentBeepInterval == 0) {
    if (!buzzerIsOn) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerIsOn = true;
      BUZZER_PRINTLN("[BUZZER] DURUM: Sürekli Ses AÇIK");
    }
    return;
  }

  if (buzzerIsOn) {
    if (currentTime - lastBuzzerToggleTime >= BEEP_ON_DURATION_MS) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerIsOn = false;
      lastBuzzerToggleTime = currentTime;
      BUZZER_PRINTLN("[BUZZER] DURUM: Bip Sesi KAPALI (Beklemede)");
    }
  } else {
    if (currentTime - lastBuzzerToggleTime >= currentBeepInterval) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerIsOn = true;
      lastBuzzerToggleTime = currentTime;
      BUZZER_PRINTLN("[BUZZER] DURUM: Bip Sesi AÇIK");
    }
  }
}

void clearDetection() {
  Serial.println("\n[INFO] Hedef kayboldu, ekran temizleniyor...");
  targetVisible = false;
  buzzerShouldBeActive = false; 
  
  sendCommand("vis rTarget,0");
  sendCommand("page0.pic=" + String(PIC_ID_SAFE));
  updateVehicleDisplay(autoZoom_enabled ? lateralRange_L1 : maxWidth_m / 2.0);
  sendCommand("tDurum.txt=\"Temiz\"");
  sendCommand("tMesafe.txt=\"--\"");
  sendCommand("tAci.txt=\"--\"");
  sendCommand("tX.txt=\"--\"");
  sendCommand("tY.txt=\"--\"");
}

void updateVehicleDisplay(float currentLateralRange_m) {
  if (currentLateralRange_m < 0.1) { currentLateralRange_m = DEFAULT_MAX_WIDTH_M / 2.0; }
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
    sendCommand("vis rTarget,1");
    targetVisible = true;
  }
  sendCommand("rTarget.pco=" + String(color));
  sendCommand("rTarget.x=" + String(x));
  sendCommand("rTarget.y=" + String(y));
}

void updateTextDisplays(float radius, int angle, float x_m, float y_m) {
  sendCommand("tDurum.txt=\"Algilandi\"");
  sendCommand("tMesafe.txt=\"" + String(radius, 2) + " m\"");
  sendCommand("tAci.txt=\"" + String(angle) + " derece\"");
  sendCommand("tX.txt=\"Y: " + String(y_m, 2) + " m\"");
  sendCommand("tY.txt=\"X: " + String(x_m, 2) + " m\"");
}


// =================================================================================
// BÖLÜM 7: EEPROM YÖNETİMİ (KALICI HAFIZA)
// =================================================================================

void loadSettingsFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(ADDR_MAGIC_KEY) != EEPROM_MAGIC_KEY) {
    Serial.println("[EEPROM] Geçersiz veri bulundu. Tüm ayarlar varsayılana sıfırlanıyor...");
    resetToDefaults();
  } else {
    Serial.println("[EEPROM] Geçerli ayarlar bulundu, okunuyor...");
    EEPROM_PRINTLN("  -> Okunuyor: Uyarı, Tehlike, Genişlik, Şifre...");
    EEPROM.get(ADDR_WARN_ZONE, warningZone_m);
    EEPROM.get(ADDR_DANGER_ZONE, dangerZone_m);
    EEPROM.get(ADDR_VEHICLE_WIDTH, vehicleRealWidth_m);
    EEPROM.get(ADDR_PASSWORD, password);
    EEPROM_PRINTLN("  -> Okunuyor: Yanal Menziller...");
    EEPROM.get(ADDR_LATERAL_L1, lateralRange_L1);
    EEPROM.get(ADDR_LATERAL_L2, lateralRange_L2);
    EEPROM.get(ADDR_LATERAL_L3, lateralRange_L3);
    EEPROM.get(ADDR_LATERAL_L4, lateralRange_L4);
    EEPROM_PRINTLN("  -> Okunuyor: Seçenekler...");
    EEPROM.get(ADDR_AUTOZOOM_EN, autoZoom_enabled);
    EEPROM.get(ADDR_AUDIOALARM_EN, audioAlarm_enabled);
    EEPROM.get(ADDR_SIDE_MARGIN, sideMargin_m);
    EEPROM.get(ADDR_MAX_WIDTH, maxWidth_m);
    EEPROM_PRINTLN("[EEPROM] Okuma tamamlandı.");
  }
  sendSettingsToNextion();
}

void saveSettingsToEEPROM() {
  EEPROM_PRINTLN("[EEPROM] Ayarlar kaydediliyor...");
  EEPROM.put(ADDR_MAGIC_KEY, EEPROM_MAGIC_KEY);
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
  EEPROM.commit();
  EEPROM_PRINTLN(" -> Kayıt tamamlandı!");
}

void resetToDefaults() {
    Serial.println("[EEPROM] Varsayılan ayarlar yükleniyor ve kaydediliyor...");
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
}

void sendSettingsToNextion() {
  NEXTION_PRINTF("[NEXTION] Mevcut ayarlar ekrana gönderiliyor...\n");
  sendCommand("pageSet1.h0.val=" + String((int)(warningZone_m * 10)));
  sendCommand("pageSet1.h1.val=" + String((int)(dangerZone_m * 10)));
  sendCommand("pageSet2.h0.val=" + String((int)(sideMargin_m * 10)));
  sendCommand("pageSet2.h1.val=" + String((int)(vehicleRealWidth_m * 10)));
  sendCommand("pageSet2.h2.val=" + String((int)(maxWidth_m * 10)));
  sendCommand("pageSet3.btZoom.val=" + String(autoZoom_enabled ? 1 : 0));
  sendCommand("pageSet3.btAudio.val=" + String(audioAlarm_enabled ? 1 : 0));
}