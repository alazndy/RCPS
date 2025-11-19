/*
 * =================================================================================
 * Proje Adı:   ESP32 & Nextion - Gelişmiş CAN BUS Radar Sistemi
 * ---------------------------------------------------------------------------------
 * Versiyon:    v3.5.1 (Stable / Memory Optimized)
 * Yazar:       G.T (AI Tarafından Düzenlendi)
 * Tarih:       19.11.2025
 *
 * --- v3.5.1 Değişiklik Notları ---
 * 1. OPTİMİZASYON (Bellek Yönetimi): Nextion ekranından gelen verilerin işlenmesinde
 *    kullanılan 'String' nesneleri tamamen kaldırıldı. Bunun yerine 'char array' 
 *    (C-String) yapısı kullanıldı. Bu, uzun süreli çalışmalarda oluşan "Heap 
 *    Fragmentation" (Bellek Parçalanması) sorununu ve cihazın kilitlenmesini önler.
 * 
 * 2. PERFORMANS: 'strtok', 'atoi', 'atof' gibi C fonksiyonları kullanılarak 
 *    veri ayrıştırma hızı artırıldı.
 * 
 * 3. DEBUG: Modüler hata ayıklama sistemi korundu.
 * =================================================================================
 */

// =================================================================================
// BÖLÜM 1: GEREKLİ KÜTÜPHANELER
// =================================================================================
#include "driver/gpio.h"       // ESP32 GPIO kontrolü
#include "driver/twai.h"       // ESP32 Dahili CAN Bus Sürücüsü (Two-Wire Automotive Interface)
#include <math.h>              // Matematiksel işlemler (Trigonometri vb.)
#include <HardwareSerial.h>    // Donanımsal Seri haberleşme
#include <EEPROM.h>            // Kalıcı hafıza yönetimi


// =================================================================================
// BÖLÜM 2: PROJE AYARLARI VE HATA AYIKLAMA (DEBUG)
// =================================================================================

// --- Ayrıntılı Hata Ayıklama (Debug) Kontrol Paneli ---
// '1' = Açık (Seri Monitörde görünür), '0' = Kapalı
#define DEBUG_CAN      0  // Ham CAN bus mesajlarını gösterir.
#define DEBUG_NEXTION  1  // Nextion komutlarını ve ayrıştırma işlemlerini gösterir.
#define DEBUG_RADAR    0  // Radar koordinat hesaplamalarını gösterir.
#define DEBUG_BUZZER   1  // Buzzer açma/kapama durumlarını gösterir.
#define DEBUG_EEPROM   1  // Hafıza okuma/yazma işlemlerini gösterir.

// Debug Makroları (Derleyici seviyesinde kod optimizasyonu sağlar)
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
#define CAN_TX_PIN GPIO_NUM_5  // CAN Transceiver TX Pini
#define CAN_RX_PIN GPIO_NUM_4  // CAN Transceiver RX Pini
#define BUZZER_PIN 25          // Buzzer Pini

// --- Bellek ve Haberleşme Ayarları ---
const long SERIAL_MONITOR_BAUD = 115200;
const long NEXTION_BAUD = 9600;
const int  RX_BUFFER_SIZE = 64; // Nextion'dan gelen veri için sabit bellek alanı

// --- EEPROM (Kalıcı Hafıza) Adres Haritası ---
#define EEPROM_SIZE 64
const int EEPROM_MAGIC_KEY = 124; // Hafızanın doğru başlatıldığını kontrol eden anahtar
const int ADDR_MAGIC_KEY = 0, ADDR_WARN_ZONE = 4, ADDR_DANGER_ZONE = 8, ADDR_VEHICLE_WIDTH = 12, ADDR_PASSWORD = 16;
const int ADDR_LATERAL_L1 = 28, ADDR_LATERAL_L2 = 32, ADDR_LATERAL_L3 = 36, ADDR_LATERAL_L4 = 40;
const int ADDR_AUTOZOOM_EN = 44, ADDR_AUDIOALARM_EN = 45;
const int ADDR_SIDE_MARGIN = 48, ADDR_MAX_WIDTH = 52;

// --- Varsayılan Fabrika Ayarları ---
const float DEFAULT_WARNING_ZONE_M = 5.0;
const float DEFAULT_DANGER_ZONE_M = 2.0;
const float DEFAULT_VEHICLE_WIDTH_M = 3.0;
const char  DEFAULT_PASSWORD[] = "1234";
const float DEFAULT_LATERAL_L1 = 6.0, DEFAULT_LATERAL_L2 = 5.0, DEFAULT_LATERAL_L3 = 3.0, DEFAULT_LATERAL_L4 = 2.0;
const bool  DEFAULT_AUTOZOOM_EN = true, DEFAULT_AUDIOALARM_EN = true;
const float DEFAULT_SIDE_MARGIN_M = 0.5, DEFAULT_MAX_WIDTH_M = 10.0;

// --- Global Ayar Değişkenleri (Çalışma Zamanı) ---
float warningZone_m, dangerZone_m, vehicleRealWidth_m;
char  password[10]; // Şifre için char array
float lateralRange_L1, lateralRange_L2, lateralRange_L3, lateralRange_L4;
bool  autoZoom_enabled, audioAlarm_enabled;
float sideMargin_m, maxWidth_m;

// --- Ekran ve Görselleştirme Sabitleri ---
const int   SCREEN_WIDTH_PX = 272, SCREEN_HEIGHT_PX = 480, TARGET_OBJECT_SIZE_PX = 30;
const float MAX_FORWARD_RANGE_M = 7.0;
const int   VEHICLE_HEIGHT_PX = 10, VEHICLE_COLOR = 31; // Nextion renk kodu (Mavi tonu)
const int   PIC_ID_SAFE = 4, PIC_ID_WARNING = 1, PIC_ID_DANGER = 2, PIC_ID_ALARM = 0;
const int   COLOR_RED = 63488, COLOR_ORANGE = 64512, COLOR_YELLOW = 65504, COLOR_GREEN = 2016;

// --- Sesli Uyarı (Buzzer) Zamanlamaları ---
const float SOLID_TONE_DISTANCE_M = 0.75; // Sürekli ötme mesafesi
const int   BEEP_ON_DURATION_MS = 60;     // "Bip" süresi
const int   BEEP_INTERVAL_YELLOW_MS = 400;
const int   BEEP_INTERVAL_ORANGE_MS = 200;
const int   BEEP_INTERVAL_RED_MS = 80;


// =================================================================================
// BÖLÜM 3: GLOBAL NESNELER VE DEĞİŞKENLER
// =================================================================================

HardwareSerial SerialNextion(2); // UART2 kullanıyoruz
bool targetVisible = false;      // Ekranda hedef var mı?

// İletişim için Sabit Bellek Tamponu (Heap Fragmentasyonunu önler)
char rxBuffer[RX_BUFFER_SIZE]; 

// Buzzer Mantığı Değişkenleri (Non-Blocking)
bool buzzerShouldBeActive = false;
bool buzzerIsOn           = false;
unsigned long lastBuzzerToggleTime = 0;
int  currentBeepInterval  = BEEP_INTERVAL_YELLOW_MS;

// --- Fonksiyon Prototipleri ---
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
// BÖLÜM 4: ANA PROGRAM (SETUP & LOOP)
// =================================================================================

void setup() {
  // 1. Pin Ayarları
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Başlangıçta sustur
  
  // 2. Seri Portları Başlat
  Serial.begin(SERIAL_MONITOR_BAUD);
  SerialNextion.begin(NEXTION_BAUD, SERIAL_8N1, 16, 17); // RX=16, TX=17
  
  Serial.println("\n======================================================");
  Serial.println("   ESP32 RADAR SİSTEMİ - v3.5.1 (Memory Optimized)");
  Serial.println("======================================================");

  // 3. Ayarları Yükle
  loadSettingsFromEEPROM();

  // 4. CAN BUS (TWAI) Sürücüsünü Başlat
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // Radar hızına göre 500kbps
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // Tüm mesajları dinle
  
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("[HATA] CAN Sürücüsü Yüklenemedi!");
    while(1); // Sistem durur
  }
  if (twai_start() != ESP_OK) {
    Serial.println("[HATA] CAN Sürücüsü Başlatılamadı!");
    while(1);
  }

  Serial.println("[INFO] CAN BUS dinleniyor...");
  clearDetection(); // Ekranı temizle
}

void loop() {
  // A. Nextion Ekranından Gelen Komutları Dinle
  handleNextionInput();

  // B. CAN Mesajlarını Dinle
  twai_message_t message;
  bool detectionThisCycle = false;
  
  // pdMS_TO_TICKS(50) -> 50ms bekle, mesaj gelmezse devam et
  if (twai_receive(&message, pdMS_TO_TICKS(50)) == ESP_OK) {
    CAN_PRINTF("[CAN] ID: 0x%03X Len: %d\n", message.identifier, message.data_length_code);
    
    // Filtreleme: Sadece Radar ID'lerini işle (Örn: 0x310 - 0x38F arası)
    if (message.identifier >= 0x310 && message.identifier <= 0x38F) {
      // Continental radarlarda genellikle 7. byte'ın son biti geçerlilik durumunu verir
      // !(0b00000001) işlemi ile bit kontrolü yapıyoruz
      bool validDetection = !(message.data[7] & 0b00000001); 
      
      if (validDetection) {
        detectionThisCycle = true;
        handleDetection(message);
      }
    }
  }

  // C. Timeout (Zaman Aşımı) Kontrolü
  // Eğer bu döngüde hiç geçerli tespit yoksa ve ekranda hedef varsa temizle
  if (!detectionThisCycle && targetVisible) {
    clearDetection();
  }

  // D. Buzzer Yönetimi (Non-Blocking)
  handleBuzzer();
}


// =================================================================================
// BÖLÜM 5: İLETİŞİM FONKSİYONLARI (Nextion - ESP32)
// =================================================================================

void sendCommand(String cmd) {
  SerialNextion.print(cmd);
  SerialNextion.write(0xFF);
  SerialNextion.write(0xFF);
  SerialNextion.write(0xFF);
}

// --- YENİLENMİŞ VERİ OKUMA FONKSİYONU (MEMORY SAFE) ---
void handleNextionInput() {
  if (SerialNextion.available()) {
    // 1. Veriyi güvenli bir şekilde sabit boyuttaki tampona (buffer) oku
    // readBytesUntil: 0xFF görene kadar VEYA buffer dolana kadar okur.
    int bytesRead = SerialNextion.readBytesUntil((char)0xFF, rxBuffer, RX_BUFFER_SIZE - 1);
    
    rxBuffer[bytesRead] = '\0'; // String sonlandırıcı karakter ekle (Null-terminator)

    // Nextion protokolü gereği gelen 3 adet 0xFF'in kalanlarını temizle
    while(SerialNextion.available() && SerialNextion.peek() == 0xFF) {
      SerialNextion.read();
    }

    if (bytesRead == 0) return; // Boş veri geldiyse işlem yapma

    NEXTION_PRINTF("\n[NEXTION] Alınan Veri: \"%s\"\n", rxBuffer);

    // 2. Veriyi İşle (String nesnesi yerine C fonksiyonları: strncmp, strtok, atoi)
    
    // --- KOMUT: LOGIN (Şifre Kontrolü) ---
    if (strncmp(rxBuffer, "LOGIN:", 6) == 0) {
      char* enteredPass = rxBuffer + 6; // "LOGIN:" sonrasındaki şifre kısmı
      
      if (strcmp(enteredPass, password) == 0) {
        NEXTION_PRINTF("[NEXTION] Giriş Başarılı.\n");
        sendSettingsToNextion();
        sendCommand("page pageSet1"); // Ayarlar sayfasına git
      } else {
        NEXTION_PRINTF("[NEXTION] Hatalı Şifre!\n");
        sendCommand("tInfo.txt=\"Hatalı Sifre\"");
        sendCommand("login_fail.val=1"); // Ekranda animasyon tetikle
      }
    }
    
    // --- KOMUT: SETPASS (Şifre Değiştir) ---
    else if (strncmp(rxBuffer, "SETPASS:", 8) == 0) {
      char* newPass = rxBuffer + 8;
      int len = strlen(newPass);
      if (len > 0 && len < 10) {
        strcpy(password, newPass); // Güvenli kopyalama
        saveSettingsToEEPROM();
        NEXTION_PRINTF("[NEXTION] Şifre güncellendi: %s\n", password);
      }
    }

    // --- KOMUT: SAVE1 (Uyarı ve Tehlike Bölgeleri) ---
    // Format: SAVE1:50,20 (5.0m ve 2.0m anlamına gelir, 10 ile çarpılmış halidir)
    else if (strncmp(rxBuffer, "SAVE1:", 6) == 0) {
      char* ptr = rxBuffer + 6;
      char* val1 = strtok(ptr, ","); // Virgüle kadar al
      char* val2 = strtok(NULL, ","); // Kalan kısımdan al

      if (val1 && val2) {
        warningZone_m = atof(val1) / 10.0;
        dangerZone_m = atof(val2) / 10.0;
        NEXTION_PRINTF("[NEXTION] SAVE1 -> Uyarı: %.1fm, Tehlike: %.1fm\n", warningZone_m, dangerZone_m);
        saveSettingsToEEPROM();
      }
    }

    // --- KOMUT: SAVE2 (Yanal Ayarlar) ---
    // Format: SAVE2:Margin,Width,MaxWidth
    else if (strncmp(rxBuffer, "SAVE2:", 6) == 0) {
      char* ptr = rxBuffer + 6;
      char* vMargin = strtok(ptr, ",");
      char* vWidth = strtok(NULL, ",");
      char* vMax = strtok(NULL, ",");

      if (vMargin && vWidth && vMax) {
        sideMargin_m = atof(vMargin) / 10.0;
        vehicleRealWidth_m = atof(vWidth) / 10.0;
        maxWidth_m = atof(vMax) / 10.0;
        NEXTION_PRINTF("[NEXTION] SAVE2 -> Yan: %.1f, Gen: %.1f, Max: %.1f\n", sideMargin_m, vehicleRealWidth_m, maxWidth_m);
        saveSettingsToEEPROM();
      }
    }

    // --- KOMUT: SAVE3 (Seçenekler) ---
    // Format: SAVE3:1,0 (Zoom: Açık, Ses: Kapalı)
    else if (strncmp(rxBuffer, "SAVE3:", 6) == 0) {
      char* ptr = rxBuffer + 6;
      char* vZoom = strtok(ptr, ",");
      char* vAudio = strtok(NULL, ",");

      if (vZoom && vAudio) {
        autoZoom_enabled = (atoi(vZoom) == 1);
        audioAlarm_enabled = (atoi(vAudio) == 1);
        NEXTION_PRINTF("[NEXTION] SAVE3 -> Zoom: %d, Ses: %d\n", autoZoom_enabled, audioAlarm_enabled);
        saveSettingsToEEPROM();
      }
    }

    // --- KOMUT: RESETALL (Fabrika Ayarları) ---
    else if (strcmp(rxBuffer, "RESETALL") == 0) {
      NEXTION_PRINTF("[NEXTION] Fabrika ayarlarına dönülüyor...\n");
      resetToDefaults();
    }
  }
}


// =================================================================================
// BÖLÜM 6: RADAR ALGORİTMASI VE GÖRSELLEŞTİRME
// =================================================================================

void handleDetection(const twai_message_t& msg) {
  RADAR_PRINTLN("\n--- [RADAR] HEDEF TESPİT EDİLDİ ---");
  
  // 1. Veri Çevirme (Continental Radar Standardı: 0.25 çarpanı)
  float polarRadius_m = msg.data[0] * 0.25;
  int   polarAngle_deg = (int)msg.data[1] - 128;
  float doc_x_m = msg.data[2] * 0.25;
  float doc_y_m = ((int)msg.data[3] - 128) * 0.25;
  
  RADAR_PRINTF("  [Veri] Mesafe: %.2fm, Açı: %d, X: %.2fm, Y: %.2fm\n", polarRadius_m, polarAngle_deg, doc_x_m, doc_y_m);

  // 2. AutoZoom ve Renk Mantığı
  float currentLateralRange_m;
  int backgroundPicId, targetColor;

  if (autoZoom_enabled) {
      // Zoom Seviyeleri (Mesafeye göre ekranın gösterdiği genişlik değişir)
      if (polarRadius_m > warningZone_m) { 
          backgroundPicId = PIC_ID_SAFE; 
          currentLateralRange_m = lateralRange_L1; // En geniş açı
          targetColor = COLOR_GREEN; 
      } else if (polarRadius_m > dangerZone_m) { 
          backgroundPicId = PIC_ID_WARNING; 
          currentLateralRange_m = lateralRange_L2; 
          targetColor = COLOR_YELLOW; 
      } else if (polarRadius_m > 1.5) { 
          backgroundPicId = PIC_ID_DANGER; 
          currentLateralRange_m = lateralRange_L3; 
          targetColor = COLOR_ORANGE; 
      } else { 
          backgroundPicId = PIC_ID_ALARM; 
          currentLateralRange_m = lateralRange_L4; // En dar açı (Maksimum Zoom)
          targetColor = COLOR_RED; 
      }
  } else {
      // Sabit Zoom
      currentLateralRange_m = maxWidth_m / 2.0;
      if (polarRadius_m > warningZone_m) { 
          backgroundPicId = PIC_ID_SAFE; targetColor = COLOR_GREEN; 
      } else if (polarRadius_m > dangerZone_m) { 
          backgroundPicId = PIC_ID_WARNING; targetColor = COLOR_YELLOW; 
      } else { 
          backgroundPicId = PIC_ID_ALARM; targetColor = COLOR_RED; 
      }
  }

  // 3. Buzzer Mantığı
  if (audioAlarm_enabled && (polarRadius_m < warningZone_m) && (abs(doc_y_m) < (vehicleRealWidth_m / 2.0 + sideMargin_m))) {
    buzzerShouldBeActive = true;
    if (polarRadius_m <= SOLID_TONE_DISTANCE_M) currentBeepInterval = 0;
    else if (backgroundPicId == PIC_ID_ALARM)   currentBeepInterval = BEEP_INTERVAL_RED_MS;
    else if (backgroundPicId == PIC_ID_DANGER)  currentBeepInterval = BEEP_INTERVAL_ORANGE_MS;
    else                                        currentBeepInterval = BEEP_INTERVAL_YELLOW_MS;
  } else {
    buzzerShouldBeActive = false;
  }

  // 4. Hedefin Ekran Konumunu Hesaplama
  if (currentLateralRange_m < 0.1) currentLateralRange_m = 0.1; 
  
  float scale_x = doc_y_m / currentLateralRange_m;
  int targetX_px = (int)(((float)SCREEN_WIDTH_PX / 2.0) + (scale_x * ((float)SCREEN_WIDTH_PX / 2.0)) + 0.5);
  int targetY_px = (int)(((MAX_FORWARD_RANGE_M - doc_x_m) / MAX_FORWARD_RANGE_M) * SCREEN_HEIGHT_PX);
  
  // Hedefi ekran içinde tut
  targetX_px = constrain(targetX_px, 0, SCREEN_WIDTH_PX - TARGET_OBJECT_SIZE_PX);
  targetY_px = constrain(targetY_px, 0, SCREEN_HEIGHT_PX - TARGET_OBJECT_SIZE_PX);
  
  // 5. Ekran Güncelleme Çağrıları
  sendCommand("page0.pic=" + String(backgroundPicId));
  
  // --- BURASI ÖNEMLİ: Doğru genişlik değeri burada gönderiliyor ---
  updateVehicleDisplay(currentLateralRange_m); 
  
  updateTargetDisplay(targetX_px, targetY_px, targetColor);
  updateTextDisplays(polarRadius_m, polarAngle_deg, doc_y_m, doc_x_m);
}
void handleBuzzer() {
  if (!buzzerShouldBeActive) {
    if (buzzerIsOn) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerIsOn = false;
      BUZZER_PRINTLN("[BUZZER] Susturuldu.");
    }
    return;
  }

  unsigned long currentTime = millis();

  // Sürekli Ses Modu
  if (currentBeepInterval == 0) {
    if (!buzzerIsOn) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerIsOn = true;
    }
    return;
  }

  // Kesikli Bip Modu
  if (buzzerIsOn) {
    // Eğer yeterince uzun süre açık kaldıysa kapat
    if (currentTime - lastBuzzerToggleTime >= BEEP_ON_DURATION_MS) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerIsOn = false;
      lastBuzzerToggleTime = currentTime;
    }
  } else {
    // Eğer bekleme süresi dolduysa tekrar aç
    if (currentTime - lastBuzzerToggleTime >= currentBeepInterval) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerIsOn = true;
      lastBuzzerToggleTime = currentTime;
    }
  }
}

void clearDetection() {
  // Hedef kaybolduğunda ekranı temizle
  targetVisible = false;
  buzzerShouldBeActive = false; 
  
  sendCommand("vis rTarget,0"); // Hedef karesini gizle
  sendCommand("page0.pic=" + String(PIC_ID_SAFE)); // Arka planı yeşil yap
  updateVehicleDisplay(autoZoom_enabled ? lateralRange_L1 : maxWidth_m / 2.0);
  
  // Metin kutularını temizle
  sendCommand("tDurum.txt=\"Temiz\"");
  sendCommand("tMesafe.txt=\"--\"");
  sendCommand("tAci.txt=\"--\"");
  sendCommand("tX.txt=\"--\"");
  sendCommand("tY.txt=\"--\"");
}

void updateVehicleDisplay(float currentLateralRange_m) {
  // 1. Güvenlik: Sıfıra bölünme hatasını önle
  if (currentLateralRange_m < 0.1) currentLateralRange_m = 0.1;

  // 2. Ölçekleme Hesaplaması
  // Ekranda gösterilen toplam genişlik = (Sol + Sağ) = currentLateralRange_m * 2
  float pixels_per_meter = (float)SCREEN_WIDTH_PX / (currentLateralRange_m * 2.0);

  // 3. Araç Genişliğini Hesapla
  int vehicle_width_px = (int)((vehicleRealWidth_m * pixels_per_meter) + 0.5);

  // --- KRİTİK DÜZELTME BAŞLANGICI ---
  
  // Sorunun Çözümü: Hesaplanan genişlik ekranın fiziksel genişliğinden büyükse,
  // Nextion bunu çizemez ve hesaplamalar şaşar (Sola kayma sorunu).
  // Bu yüzden genişliği ekran boyutuyla (SCREEN_WIDTH_PX) sınırlandırıyoruz.
  if (vehicle_width_px > SCREEN_WIDTH_PX) {
    vehicle_width_px = SCREEN_WIDTH_PX;
  }
  
  // Minimum genişlik kontrolü (Aracın ekrandan kaybolmaması için en az 2 piksel olsun)
  if (vehicle_width_px < 2) {
    vehicle_width_px = 2;
  }

  // 4. X Konumunu Hesapla (Aracı ortalamak için)
  // Formül: (Ekran Genişliği - Araç Genişliği) / 2
  int vehicle_x_px = (int)(((float)SCREEN_WIDTH_PX - vehicle_width_px) / 2.0 + 0.5);

  // X koordinatı negatif olamaz, olursa 0'a sabitle
  if (vehicle_x_px < 0) {
    vehicle_x_px = 0;
  }

  // --- KRİTİK DÜZELTME BİTİŞİ ---

  // 5. Nextion'a Veri Gönderimi
  // Önce X koordinatını, sonra Genişliği (W) göndermek görsel hataları azaltır.
  sendCommand("rVehicle.x=" + String(vehicle_x_px));
  sendCommand("rVehicle.w=" + String(vehicle_width_px));
  
  // Yükseklik ve Y konumu genellikle sabittir ama her ihtimale karşı güncelliyoruz
  sendCommand("rVehicle.y=" + String(SCREEN_HEIGHT_PX - VEHICLE_HEIGHT_PX));
  sendCommand("rVehicle.h=" + String(VEHICLE_HEIGHT_PX));
  sendCommand("rVehicle.bco=" + String(VEHICLE_COLOR));
}

void updateTargetDisplay(int x, int y, int color) {
  if (!targetVisible) {
    sendCommand("vis rTarget,1"); // Görünür yap
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
  // Nextion'a gönderilen X ve Y'yi kullanıcıya göre isimlendir (Y=Yanal, X=İleri)
  sendCommand("tX.txt=\"Y: " + String(y_m, 2) + "\""); 
  sendCommand("tY.txt=\"X: " + String(x_m, 2) + "\"");
}


// =================================================================================
// BÖLÜM 7: EEPROM (KALICI HAFIZA) FONKSİYONLARI
// =================================================================================

void loadSettingsFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  
  // Magic Key kontrolü: EEPROM daha önce yazılmış mı?
  if (EEPROM.read(ADDR_MAGIC_KEY) != EEPROM_MAGIC_KEY) {
    Serial.println("[EEPROM] Veri boş veya bozuk. Varsayılanlar yükleniyor...");
    resetToDefaults();
  } else {
    Serial.println("[EEPROM] Ayarlar yükleniyor...");
    EEPROM.get(ADDR_WARN_ZONE, warningZone_m);
    EEPROM.get(ADDR_DANGER_ZONE, dangerZone_m);
    EEPROM.get(ADDR_VEHICLE_WIDTH, vehicleRealWidth_m);
    EEPROM.get(ADDR_PASSWORD, password); // Char array doğrudan okunur
    EEPROM.get(ADDR_LATERAL_L1, lateralRange_L1);
    EEPROM.get(ADDR_LATERAL_L2, lateralRange_L2);
    EEPROM.get(ADDR_LATERAL_L3, lateralRange_L3);
    EEPROM.get(ADDR_LATERAL_L4, lateralRange_L4);
    EEPROM.get(ADDR_AUTOZOOM_EN, autoZoom_enabled);
    EEPROM.get(ADDR_AUDIOALARM_EN, audioAlarm_enabled);
    EEPROM.get(ADDR_SIDE_MARGIN, sideMargin_m);
    EEPROM.get(ADDR_MAX_WIDTH, maxWidth_m);
    EEPROM_PRINTLN("[EEPROM] Okuma tamamlandı.");
  }
  sendSettingsToNextion();
}

void saveSettingsToEEPROM() {
  EEPROM_PRINTLN("[EEPROM] Ayarlar hafızaya yazılıyor...");
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
  EEPROM.commit(); // Fiziksel yazma işlemi
  EEPROM_PRINTLN(" -> Yazma Başarılı!");
}

void resetToDefaults() {
    Serial.println("[RESET] Fabrika ayarlarına dönülüyor...");
    warningZone_m = DEFAULT_WARNING_ZONE_M;
    dangerZone_m = DEFAULT_DANGER_ZONE_M;
    vehicleRealWidth_m = DEFAULT_VEHICLE_WIDTH_M;
    strcpy(password, DEFAULT_PASSWORD); // String kopyalama
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
  NEXTION_PRINTF("[NEXTION] Ayarlar arayüze senkronize ediliyor...\n");
  // Nextion'da float olmadığı için değerleri 10 ile çarpıp int olarak gönderiyoruz
  sendCommand("pageSet1.h0.val=" + String((int)(warningZone_m * 10)));
  sendCommand("pageSet1.h1.val=" + String((int)(dangerZone_m * 10)));
  
  sendCommand("pageSet2.h0.val=" + String((int)(sideMargin_m * 10)));
  sendCommand("pageSet2.h1.val=" + String((int)(vehicleRealWidth_m * 10)));
  sendCommand("pageSet2.h2.val=" + String((int)(maxWidth_m * 10)));
  
  // Checkbox (Switch) butonları için 0 veya 1 gönder
  sendCommand("pageSet3.btZoom.val=" + String(autoZoom_enabled ? 1 : 0));
  sendCommand("pageSet3.btAudio.val=" + String(audioAlarm_enabled ? 1 : 0));
}