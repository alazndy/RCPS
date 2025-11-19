/*
 * =================================================================================
 * Proje Adı:   ESP32 & Nextion - Gelişmiş CAN BUS Radar Sistemi
 * ---------------------------------------------------------------------------------
 * Versiyon:    v3.6.0 (Simulation Synced & Memory Safe)
 * Tarih:       19.11.2025
 *
 * --- v3.6.0 Değişiklikleri ---
 * 1. SİMÜLASYON UYUMU: React/TypeScript simülasyonundaki "Fixed Scale" mantığı
 *    birebir uyarlandı. Artık X ve Y eksenleri aynı oranda ölçekleniyor.
 *    Görüntü sündürülmüyor.
 * 2. ARAÇ GÖRSELİ DÜZELTMESİ: Nextion ekran genişliğini aşan araç çizimlerinin
 *    sola kayması engellendi (Constrain eklendi).
 * 3. GRID SİSTEMİ: Arka plan resimlerine (10m, 8m, 6m, 4m) tam uyumlu geçişler.
 * =================================================================================
 */

#include "driver/gpio.h"
#include "driver/twai.h"
#include <math.h>
#include <HardwareSerial.h>
#include <EEPROM.h>

// =================================================================================
// DEBUG AYARLARI
// =================================================================================
#define DEBUG_CAN      0
#define DEBUG_NEXTION  1
#define DEBUG_RADAR    0 
#define DEBUG_BUZZER   0
#define DEBUG_EEPROM   1

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

// =================================================================================
// DONANIM VE SABİTLER
// =================================================================================
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4
#define BUZZER_PIN 25

const long SERIAL_MONITOR_BAUD = 115200;
const long NEXTION_BAUD = 9600;
const int  RX_BUFFER_SIZE = 64;

// --- EEPROM ---
#define EEPROM_SIZE 64
const int EEPROM_MAGIC_KEY = 124;
const int ADDR_MAGIC_KEY = 0, ADDR_WARN_ZONE = 4, ADDR_DANGER_ZONE = 8, ADDR_VEHICLE_WIDTH = 12, ADDR_PASSWORD = 16;
const int ADDR_LATERAL_L1 = 28, ADDR_LATERAL_L2 = 32, ADDR_LATERAL_L3 = 36, ADDR_LATERAL_L4 = 40;
const int ADDR_AUTOZOOM_EN = 44, ADDR_AUDIOALARM_EN = 45;
const int ADDR_SIDE_MARGIN = 48, ADDR_MAX_WIDTH = 52;

// --- Varsayılanlar ---
const float DEFAULT_WARNING_ZONE_M = 5.0;
const float DEFAULT_DANGER_ZONE_M = 2.0;
const float DEFAULT_VEHICLE_WIDTH_M = 2.0; // Standart binek araç
const char  DEFAULT_PASSWORD[] = "1234";
// (Lateral Range ayarları artık sabit 10/8/6/4 mantığına geçtiği için EEPROM'daki bu değerler override edilecek)
const float DEFAULT_LATERAL_L1 = 10.0, DEFAULT_LATERAL_L2 = 8.0, DEFAULT_LATERAL_L3 = 6.0, DEFAULT_LATERAL_L4 = 4.0;
const bool  DEFAULT_AUTOZOOM_EN = true, DEFAULT_AUDIOALARM_EN = true;
const float DEFAULT_SIDE_MARGIN_M = 0.5, DEFAULT_MAX_WIDTH_M = 10.0;

// --- Global Değişkenler ---
float warningZone_m, dangerZone_m, vehicleRealWidth_m;
char  password[10];
float lateralRange_L1, lateralRange_L2, lateralRange_L3, lateralRange_L4; // Uyumluluk için tutuyoruz
bool  autoZoom_enabled, audioAlarm_enabled;
float sideMargin_m, maxWidth_m;

// --- Ekran Ayarları ---
const int   SCREEN_WIDTH_PX = 272;
const int   SCREEN_HEIGHT_PX = 480;
const int   TARGET_OBJECT_SIZE_PX = 30; // Hedef kutucuk boyutu
const int   VEHICLE_HEIGHT_PX = 10;
const int   VEHICLE_COLOR = 31; 

// Nextion Resim ID'leri
const int PIC_ID_SAFE = 4;    // 10m Genişlik
const int PIC_ID_WARNING = 1; // 8m Genişlik
const int PIC_ID_DANGER = 2;  // 6m Genişlik
const int PIC_ID_ALARM = 0;   // 4m Genişlik
const int COLOR_RED = 63488, COLOR_ORANGE = 64512, COLOR_YELLOW = 65504, COLOR_GREEN = 2016;

// --- Buzzer ---
const float SOLID_TONE_DISTANCE_M = 0.75;
const int   BEEP_ON_DURATION_MS = 60;
const int   BEEP_INTERVAL_YELLOW_MS = 400;
const int   BEEP_INTERVAL_ORANGE_MS = 200;
const int   BEEP_INTERVAL_RED_MS = 80;

// =================================================================================
// GLOBAL NESNELER
// =================================================================================
HardwareSerial SerialNextion(2);
bool targetVisible = false;
char rxBuffer[RX_BUFFER_SIZE]; 

bool buzzerShouldBeActive = false;
bool buzzerIsOn           = false;
unsigned long lastBuzzerToggleTime = 0;
int  currentBeepInterval  = BEEP_INTERVAL_YELLOW_MS;

// --- Prototip ---
void sendCommand(String cmd);
void loadSettingsFromEEPROM();
void saveSettingsToEEPROM();
void resetToDefaults();
void handleNextionInput();
void handleDetection(const twai_message_t& msg);
void clearDetection();
void updateVehicleDisplay(float currentMaxGridXMeters); // İmzası değişti
void updateTargetDisplay(int x, int y, int color);
void updateTextDisplays(float radius, int angle, float x_m, float y_m);
void sendSettingsToNextion();
void handleBuzzer();


// =================================================================================
// SETUP & LOOP
// =================================================================================
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  Serial.begin(SERIAL_MONITOR_BAUD);
  SerialNextion.begin(NEXTION_BAUD, SERIAL_8N1, 16, 17);
  
  Serial.println("\n======================================================");
  Serial.println("   ESP32 RADAR SİSTEMİ - v3.6.0 (Simülasyon Senkronize)");
  Serial.println("======================================================");

  loadSettingsFromEEPROM();

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK || twai_start() != ESP_OK) {
    Serial.println("[HATA] CAN Sürücüsü Başlatılamadı!");
    while(1);
  }

  Serial.println("[INFO] CAN BUS dinleniyor...");
  clearDetection();
}

void loop() {
  handleNextionInput();

  twai_message_t message;
  bool detectionThisCycle = false;
  
  if (twai_receive(&message, pdMS_TO_TICKS(50)) == ESP_OK) {
    // CAN ID Filtresi (Radar için)
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
// İLETİŞİM
// =================================================================================
void sendCommand(String cmd) {
  SerialNextion.print(cmd);
  SerialNextion.write(0xFF);
  SerialNextion.write(0xFF);
  SerialNextion.write(0xFF);
}

void handleNextionInput() {
  if (SerialNextion.available()) {
    int bytesRead = SerialNextion.readBytesUntil((char)0xFF, rxBuffer, RX_BUFFER_SIZE - 1);
    rxBuffer[bytesRead] = '\0';

    while(SerialNextion.available() && SerialNextion.peek() == 0xFF) {
      SerialNextion.read();
    }

    if (bytesRead == 0) return;
    NEXTION_PRINTF("\n[NEXTION] Rx: \"%s\"\n", rxBuffer);

    if (strncmp(rxBuffer, "LOGIN:", 6) == 0) {
      if (strcmp(rxBuffer + 6, password) == 0) {
        sendSettingsToNextion();
        sendCommand("page pageSet1");
      } else {
        sendCommand("tInfo.txt=\"Hatalı Sifre\"");
        sendCommand("login_fail.val=1");
      }
    }
    else if (strncmp(rxBuffer, "SETPASS:", 8) == 0) {
      char* newPass = rxBuffer + 8;
      if (strlen(newPass) > 0 && strlen(newPass) < 10) {
        strcpy(password, newPass);
        saveSettingsToEEPROM();
      }
    }
    else if (strncmp(rxBuffer, "SAVE1:", 6) == 0) {
      char* ptr = rxBuffer + 6;
      char* val1 = strtok(ptr, ",");
      char* val2 = strtok(NULL, ",");
      if (val1 && val2) {
        warningZone_m = atof(val1) / 10.0;
        dangerZone_m = atof(val2) / 10.0;
        saveSettingsToEEPROM();
      }
    }
    else if (strncmp(rxBuffer, "SAVE2:", 6) == 0) {
      char* ptr = rxBuffer + 6;
      char* vMargin = strtok(ptr, ",");
      char* vWidth = strtok(NULL, ",");
      char* vMax = strtok(NULL, ",");
      if (vMargin && vWidth && vMax) {
        sideMargin_m = atof(vMargin) / 10.0;
        vehicleRealWidth_m = atof(vWidth) / 10.0;
        maxWidth_m = atof(vMax) / 10.0;
        saveSettingsToEEPROM();
      }
    }
    else if (strncmp(rxBuffer, "SAVE3:", 6) == 0) {
      char* ptr = rxBuffer + 6;
      char* vZoom = strtok(ptr, ",");
      char* vAudio = strtok(NULL, ",");
      if (vZoom && vAudio) {
        autoZoom_enabled = (atoi(vZoom) == 1);
        audioAlarm_enabled = (atoi(vAudio) == 1);
        saveSettingsToEEPROM();
      }
    }
    else if (strcmp(rxBuffer, "RESETALL") == 0) {
      resetToDefaults();
    }
  }
}


// =================================================================================
// RADAR MANTIĞI (v3.6.0 GÜNCELLEMELERİ BURADA)
// =================================================================================

void handleDetection(const twai_message_t& msg) {
  RADAR_PRINTLN("\n--- [RADAR] TESPİT ---");
  
  // 1. Ham Veriyi Çevirme
  float polarRadius_m = msg.data[0] * 0.25;
  int   polarAngle_deg = (int)msg.data[1] - 128;
  float doc_x_m = msg.data[2] * 0.25;             // İleri (Simülasyon Y)
  float doc_y_m = ((int)msg.data[3] - 128) * 0.25;// Yanal (Simülasyon X)
  
  RADAR_PRINTF("  [DATA] R:%.2f, X:%.2f, Y:%.2f\n", polarRadius_m, doc_x_m, doc_y_m);

  // 2. Grid ve Zoom Ayarı (Simülasyon ile birebir)
  // Kurallar: >5m -> 10m, 3-5m -> 8m, 1.5-3m -> 6m, <1.5m -> 4m
  float currentMaxGridXMeters; 
  int backgroundPicId, targetColor;

  if (autoZoom_enabled) {
      if (polarRadius_m > 5.0) { 
          currentMaxGridXMeters = 10.0; 
          backgroundPicId = PIC_ID_SAFE; 
          targetColor = COLOR_GREEN; 
      } else if (polarRadius_m > 3.0) { 
          currentMaxGridXMeters = 8.0; 
          backgroundPicId = PIC_ID_WARNING; 
          targetColor = COLOR_YELLOW; 
      } else if (polarRadius_m > 1.5) { 
          currentMaxGridXMeters = 6.0; 
          backgroundPicId = PIC_ID_DANGER; 
          targetColor = COLOR_ORANGE; 
      } else { 
          currentMaxGridXMeters = 4.0; 
          backgroundPicId = PIC_ID_ALARM; 
          targetColor = COLOR_RED; 
      }
  } else {
      // Zoom kapalıysa en geniş açı
      currentMaxGridXMeters = 10.0;
      if (polarRadius_m > warningZone_m) { backgroundPicId = PIC_ID_SAFE; targetColor = COLOR_GREEN; }
      else if (polarRadius_m > dangerZone_m) { backgroundPicId = PIC_ID_WARNING; targetColor = COLOR_YELLOW; }
      else { backgroundPicId = PIC_ID_ALARM; targetColor = COLOR_RED; }
  }

  // 3. Eşit Ölçekleme (Fixed Scale / Uniform Scaling)
  // Hem X hem Y ekseni için aynı katsayı kullanılır.
  float fixedScale = (float)SCREEN_WIDTH_PX / currentMaxGridXMeters;

  // 4. Koordinat Haritalama (Simülasyon Mantığı)
  
  // X EKSENİ (YANAL): (Yanal_Mesafe + (ToplamGenişlik / 2)) * Ölçek
  float targetX_float = (doc_y_m + (currentMaxGridXMeters / 2.0)) * fixedScale;
  int targetX_px = (int)(targetX_float + 0.5);

  // Y EKSENİ (İLERİ): EkranBoyu - (İleri_Mesafe * Ölçek)
  float targetY_float = (float)SCREEN_HEIGHT_PX - (doc_x_m * fixedScale);
  int targetY_px = (int)(targetY_float + 0.5);

  // 5. Sınırlandırma (Ekranda kalması için)
  targetX_px = constrain(targetX_px, 0, SCREEN_WIDTH_PX - TARGET_OBJECT_SIZE_PX);
  targetY_px = constrain(targetY_px, 0, SCREEN_HEIGHT_PX - TARGET_OBJECT_SIZE_PX);

  // 6. Buzzer Mantığı
  if (audioAlarm_enabled && (polarRadius_m < warningZone_m) && (abs(doc_y_m) < (vehicleRealWidth_m / 2.0 + sideMargin_m))) {
    buzzerShouldBeActive = true;
    if (polarRadius_m <= SOLID_TONE_DISTANCE_M) currentBeepInterval = 0;
    else if (backgroundPicId == PIC_ID_ALARM)   currentBeepInterval = BEEP_INTERVAL_RED_MS;
    else if (backgroundPicId == PIC_ID_DANGER)  currentBeepInterval = BEEP_INTERVAL_ORANGE_MS;
    else                                        currentBeepInterval = BEEP_INTERVAL_YELLOW_MS;
  } else {
    buzzerShouldBeActive = false;
  }

  // 7. Ekran Güncelleme
  sendCommand("page0.pic=" + String(backgroundPicId));
  // Yeni fonksiyona 'grid genişliğini' gönderiyoruz, ölçeği kendi hesaplıyor
  updateVehicleDisplay(currentMaxGridXMeters); 
  updateTargetDisplay(targetX_px, targetY_px, targetColor);
  updateTextDisplays(polarRadius_m, polarAngle_deg, doc_y_m, doc_x_m);
}

void updateVehicleDisplay(float currentMaxGridXMeters) {
  // Güvenlik
  if (currentMaxGridXMeters < 0.1) currentMaxGridXMeters = 10.0;
  
  // Aynı ölçeği burada da hesapla
  float fixedScale = (float)SCREEN_WIDTH_PX / currentMaxGridXMeters;
  
  // Araç genişliğini piksel cinsinden hesapla
  int vehicle_width_px = (int)((vehicleRealWidth_m * fixedScale) + 0.5);

  // --- DÜZELTME: Araç ekrandan taşarsa genişliği sınırla ---
  if (vehicle_width_px > SCREEN_WIDTH_PX) {
    vehicle_width_px = SCREEN_WIDTH_PX;
  }
  if (vehicle_width_px < 2) {
    vehicle_width_px = 2;
  }

  // Aracı ortalamak için X konumunu hesapla
  int vehicle_x_px = (int)(((float)SCREEN_WIDTH_PX - vehicle_width_px) / 2.0 + 0.5);
  
  // Negatif olmasını engelle
  if (vehicle_x_px < 0) vehicle_x_px = 0;

  // Verileri Gönder
  sendCommand("rVehicle.x=" + String(vehicle_x_px));
  sendCommand("rVehicle.w=" + String(vehicle_width_px));
  sendCommand("rVehicle.y=" + String(SCREEN_HEIGHT_PX - VEHICLE_HEIGHT_PX));
  sendCommand("rVehicle.h=" + String(VEHICLE_HEIGHT_PX));
  sendCommand("rVehicle.bco=" + String(VEHICLE_COLOR));
}

void clearDetection() {
  targetVisible = false;
  buzzerShouldBeActive = false; 
  
  sendCommand("vis rTarget,0");
  sendCommand("page0.pic=" + String(PIC_ID_SAFE));
  
  // Temizlendiğinde varsayılan olarak en geniş açıyı (10m) göster
  updateVehicleDisplay(10.0);
  
  sendCommand("tDurum.txt=\"Temiz\"");
  sendCommand("tMesafe.txt=\"--\"");
  sendCommand("tAci.txt=\"--\"");
  sendCommand("tX.txt=\"--\"");
  sendCommand("tY.txt=\"--\"");
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
  sendCommand("tDurum.txt=\"HEDEF\"");
  sendCommand("tMesafe.txt=\"" + String(radius, 2) + "m\"");
  sendCommand("tAci.txt=\"" + String(angle) + "d\"");
  sendCommand("tX.txt=\"Y: " + String(y_m, 2) + "\""); 
  sendCommand("tY.txt=\"X: " + String(x_m, 2) + "\"");
}

void handleBuzzer() {
  if (!buzzerShouldBeActive) {
    if (buzzerIsOn) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerIsOn = false;
    }
    return;
  }
  unsigned long currentTime = millis();
  if (currentBeepInterval == 0) {
    if (!buzzerIsOn) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerIsOn = true;
    }
    return;
  }
  if (buzzerIsOn) {
    if (currentTime - lastBuzzerToggleTime >= BEEP_ON_DURATION_MS) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerIsOn = false;
      lastBuzzerToggleTime = currentTime;
    }
  } else {
    if (currentTime - lastBuzzerToggleTime >= currentBeepInterval) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerIsOn = true;
      lastBuzzerToggleTime = currentTime;
    }
  }
}


// =================================================================================
// EEPROM FONKSİYONLARI
// =================================================================================
void loadSettingsFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(ADDR_MAGIC_KEY) != EEPROM_MAGIC_KEY) {
    resetToDefaults();
  } else {
    EEPROM.get(ADDR_WARN_ZONE, warningZone_m);
    EEPROM.get(ADDR_DANGER_ZONE, dangerZone_m);
    EEPROM.get(ADDR_VEHICLE_WIDTH, vehicleRealWidth_m);
    EEPROM.get(ADDR_PASSWORD, password);
    EEPROM.get(ADDR_AUTOZOOM_EN, autoZoom_enabled);
    EEPROM.get(ADDR_AUDIOALARM_EN, audioAlarm_enabled);
    EEPROM.get(ADDR_SIDE_MARGIN, sideMargin_m);
    EEPROM.get(ADDR_MAX_WIDTH, maxWidth_m);
    // Lateral değerler artık kod içinde sabit (10/8/6/4) ama EEPROM yapısını bozmuyoruz
    EEPROM.get(ADDR_LATERAL_L1, lateralRange_L1);
    EEPROM.get(ADDR_LATERAL_L2, lateralRange_L2);
    EEPROM.get(ADDR_LATERAL_L3, lateralRange_L3);
    EEPROM.get(ADDR_LATERAL_L4, lateralRange_L4);
  }
  sendSettingsToNextion();
}

void saveSettingsToEEPROM() {
  EEPROM.put(ADDR_MAGIC_KEY, EEPROM_MAGIC_KEY);
  EEPROM.put(ADDR_WARN_ZONE, warningZone_m);
  EEPROM.put(ADDR_DANGER_ZONE, dangerZone_m);
  EEPROM.put(ADDR_VEHICLE_WIDTH, vehicleRealWidth_m);
  EEPROM.put(ADDR_PASSWORD, password);
  EEPROM.put(ADDR_AUTOZOOM_EN, autoZoom_enabled);
  EEPROM.put(ADDR_AUDIOALARM_EN, audioAlarm_enabled);
  EEPROM.put(ADDR_SIDE_MARGIN, sideMargin_m);
  EEPROM.put(ADDR_MAX_WIDTH, maxWidth_m);
  EEPROM.put(ADDR_LATERAL_L1, lateralRange_L1);
  EEPROM.put(ADDR_LATERAL_L2, lateralRange_L2);
  EEPROM.put(ADDR_LATERAL_L3, lateralRange_L3);
  EEPROM.put(ADDR_LATERAL_L4, lateralRange_L4);
  EEPROM.commit();
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
}

void sendSettingsToNextion() {
  sendCommand("pageSet1.h0.val=" + String((int)(warningZone_m * 10)));
  sendCommand("pageSet1.h1.val=" + String((int)(dangerZone_m * 10)));
  sendCommand("pageSet2.h0.val=" + String((int)(sideMargin_m * 10)));
  sendCommand("pageSet2.h1.val=" + String((int)(vehicleRealWidth_m * 10)));
  sendCommand("pageSet2.h2.val=" + String((int)(maxWidth_m * 10)));
  sendCommand("pageSet3.btZoom.val=" + String(autoZoom_enabled ? 1 : 0));
  sendCommand("pageSet3.btAudio.val=" + String(audioAlarm_enabled ? 1 : 0));
}