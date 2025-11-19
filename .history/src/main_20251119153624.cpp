/*
 * =================================================================================================
 * PROJE KİMLİĞİ
 * =================================================================================================
 * Proje Adı:   ESP32 & Nextion - Gelişmiş CAN BUS Radar Sistemi
 * Platform:    ESP32 (PlatformIO / Arduino IDE)
 * Versiyon:    v3.6.0 (Simülasyon Senkronize & Bellek Korumalı)
 * Tarih:       19.11.2025
 *
 * --- SÜRÜM NOTLARI (v3.6.0) ---
 * 1. GÖRSEL MOTORU (Fixed Scale): React simülasyonundaki "Eşit Ölçekleme" mantığı birebir
 *    koda işlendi. Artık X ve Y eksenleri aynı katsayı ile ölçekleniyor, görüntü sündürülmüyor.
 * 
 * 2. DÜZELTME (Araç Çizimi): Nextion ekran genişliğini (272px) aşan araç çizimlerinin,
 *    koordinat hatası yüzünden sola yapışması engellendi. (Constrain mantığı eklendi).
 * 
 * 3. BELLEK GÜVENLİĞİ: 'String' nesneleri iletişim döngüsünden çıkarıldı. C-String (char array)
 *    kullanılarak uzun süreli çalışmalarda kilitlenme (heap fragmentation) riski sıfırlandı.
 * =================================================================================================
 */

// -------------------------------------------------------------------------------------------------
// 1. KÜTÜPHANELER
// -------------------------------------------------------------------------------------------------
#include "driver/gpio.h"       // ESP32 GPIO kontrolü için
#include "driver/twai.h"       // ESP32 Dahili CAN Sürücüsü (MCP2515 gerekmez)
#include <math.h>              // Matematiksel işlemler (abs, trigonometri vb.)
#include <HardwareSerial.h>    // Nextion ile donanımsal seri haberleşme
#include <EEPROM.h>            // Ayarların kalıcı hafızada saklanması


// -------------------------------------------------------------------------------------------------
// 2. HATA AYIKLAMA (DEBUG) AYARLARI
// -------------------------------------------------------------------------------------------------
// İstenen modülün loglarını açmak için '1', kapatmak için '0' yapın.
// Bu yöntem işlemciyi yormadan sadece gerekli bilgiyi görmenizi sağlar.
#define DEBUG_CAN      0  // Ham CAN verilerini göster
#define DEBUG_NEXTION  1  // Ekran komutlarını göster
#define DEBUG_RADAR    0  // Koordinat hesaplarını göster
#define DEBUG_BUZZER   0  // Ses durumunu göster
#define DEBUG_EEPROM   1  // Hafıza işlemlerini göster

// Derleyici Makroları (Kod optimizasyonu sağlar, kapalıysa derlemeye dahil edilmez)
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


// -------------------------------------------------------------------------------------------------
// 3. DONANIM PIN TANIMLAMALARI
// -------------------------------------------------------------------------------------------------
#define CAN_TX_PIN GPIO_NUM_5  // CAN Transceiver TX
#define CAN_RX_PIN GPIO_NUM_4  // CAN Transceiver RX
#define BUZZER_PIN 25          // Buzzer (Aktif Buzzer önerilir)


// -------------------------------------------------------------------------------------------------
// 4. SİSTEM SABİTLERİ VE AYARLAR
// -------------------------------------------------------------------------------------------------
const long SERIAL_MONITOR_BAUD = 115200; // PC bağlantı hızı
const long NEXTION_BAUD        = 9600;   // Ekran bağlantı hızı
const int  RX_BUFFER_SIZE      = 64;     // Nextion'dan gelen veri için güvenli tampon boyutu

// --- EEPROM Adres Haritası ---
#define EEPROM_SIZE 64
const int EEPROM_MAGIC_KEY    = 124; // Hafızanın bozulup bozulmadığını anlamak için anahtar
const int ADDR_MAGIC_KEY      = 0;
const int ADDR_WARN_ZONE      = 4;
const int ADDR_DANGER_ZONE    = 8;
const int ADDR_VEHICLE_WIDTH  = 12;
const int ADDR_PASSWORD       = 16;
const int ADDR_LATERAL_L1     = 28; // (Not: v3.6.0'da bu değerler sabittir, uyumluluk için tutuluyor)
const int ADDR_LATERAL_L2     = 32;
const int ADDR_LATERAL_L3     = 36;
const int ADDR_LATERAL_L4     = 40;
const int ADDR_AUTOZOOM_EN    = 44;
const int ADDR_AUDIOALARM_EN  = 45;
const int ADDR_SIDE_MARGIN    = 48;
const int ADDR_MAX_WIDTH      = 52;

// --- Varsayılan Fabrika Ayarları ---
const float DEFAULT_WARNING_ZONE_M    = 5.0;
const float DEFAULT_DANGER_ZONE_M     = 2.0;
const float DEFAULT_VEHICLE_WIDTH_M   = 2.0; 
const char  DEFAULT_PASSWORD[]        = "1234";
const float DEFAULT_SIDE_MARGIN_M     = 0.5;
const float DEFAULT_MAX_WIDTH_M       = 10.0;
const bool  DEFAULT_AUTOZOOM_EN       = true;
const bool  DEFAULT_AUDIOALARM_EN     = true;
// Uyumluluk için varsayılanlar
const float DEFAULT_LATERAL_L1 = 10.0, DEFAULT_LATERAL_L2 = 8.0, DEFAULT_LATERAL_L3 = 6.0, DEFAULT_LATERAL_L4 = 4.0;

// --- Ekran Fiziksel Özellikleri ---
const int   SCREEN_WIDTH_PX       = 272;
const int   SCREEN_HEIGHT_PX      = 480;
const int   TARGET_OBJECT_SIZE_PX = 30; // Hedef kare boyutu
const int   VEHICLE_HEIGHT_PX     = 10; // Araç temsili çizgisinin kalınlığı
const int   VEHICLE_COLOR         = 31; // Nextion Renk Kodu (Mavi)

// --- Nextion Resim ID'leri (Simülasyon Gridlerine Uygun) ---
const int PIC_ID_SAFE    = 4;  // Grid Genişliği: 10m
const int PIC_ID_WARNING = 1;  // Grid Genişliği: 8m
const int PIC_ID_DANGER  = 2;  // Grid Genişliği: 6m
const int PIC_ID_ALARM   = 0;  // Grid Genişliği: 4m

// --- Nextion Renk Kodları ---
const int COLOR_RED      = 63488;
const int COLOR_ORANGE   = 64512;
const int COLOR_YELLOW   = 65504;
const int COLOR_GREEN    = 2016;

// --- Buzzer Zamanlamaları ---
const float SOLID_TONE_DISTANCE_M   = 0.75; // Sürekli ötme mesafesi
const int   BEEP_ON_DURATION_MS     = 60;   // Bip sesinin uzunluğu
const int   BEEP_INTERVAL_YELLOW_MS = 400;
const int   BEEP_INTERVAL_ORANGE_MS = 200;
const int   BEEP_INTERVAL_RED_MS    = 80;


// -------------------------------------------------------------------------------------------------
// 5. GLOBAL DEĞİŞKENLER VE NESNELER
// -------------------------------------------------------------------------------------------------
HardwareSerial SerialNextion(2); // UART2 üzerinden Nextion
bool targetVisible = false;      // Ekranda aktif bir hedef var mı?
char rxBuffer[RX_BUFFER_SIZE];   // İletişim tamponu (String yerine char array)

// Çalışma Zamanı Ayarları (EEPROM'dan yüklenir)
float warningZone_m, dangerZone_m, vehicleRealWidth_m;
char  password[10];
float lateralRange_L1, lateralRange_L2, lateralRange_L3, lateralRange_L4;
bool  autoZoom_enabled, audioAlarm_enabled;
float sideMargin_m, maxWidth_m;

// Buzzer Durum Değişkenleri
bool buzzerShouldBeActive = false;
bool buzzerIsOn           = false;
unsigned long lastBuzzerToggleTime = 0;
int  currentBeepInterval  = BEEP_INTERVAL_YELLOW_MS;


// -------------------------------------------------------------------------------------------------
// 6. FONKSİYON PROTOTİPLERİ (Ön Bildirimler)
// -------------------------------------------------------------------------------------------------
void sendCommand(String cmd);
void loadSettingsFromEEPROM();
void saveSettingsToEEPROM();
void resetToDefaults();
void handleNextionInput();
void handleDetection(const twai_message_t& msg);
void clearDetection();
void updateVehicleDisplay(float currentMaxGridXMeters);
void updateTargetDisplay(int x, int y, int color);
void updateTextDisplays(float radius, int angle, float x_m, float y_m);
void sendSettingsToNextion();
void handleBuzzer();


// -------------------------------------------------------------------------------------------------
// 7. ANA KURULUM (SETUP)
// -------------------------------------------------------------------------------------------------
void setup() {
  // A. Pin Ayarları
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  // B. Seri Haberleşme Başlatma
  Serial.begin(SERIAL_MONITOR_BAUD);
  SerialNextion.begin(NEXTION_BAUD, SERIAL_8N1, 16, 17);
  
  Serial.println("\n======================================================");
  Serial.println("   ESP32 RADAR SİSTEMİ - v3.6.0 (Simülasyon Senkronize)");
  Serial.println("======================================================");

  // C. Hafızadan Ayarları Oku
  loadSettingsFromEEPROM();

  // D. CAN BUS (TWAI) Kurulumu
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // Radar iletişim hızı
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // Tüm mesajları dinle
  
  // CAN Başlatma Kontrolü
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK || twai_start() != ESP_OK) {
    Serial.println("[HATA] CAN Sürücüsü Başlatılamadı! Kablolamayı kontrol edin.");
    while(1); // Hata durumunda sonsuz döngüde bekle
  }

  Serial.println("[INFO] Sistem hazır, CAN BUS dinleniyor...");
  clearDetection(); // Ekranı temizleyerek başla
}


// -------------------------------------------------------------------------------------------------
// 8. ANA DÖNGÜ (LOOP)
// -------------------------------------------------------------------------------------------------
void loop() {
  // 1. Adım: Nextion'dan gelen dokunmatik verileri kontrol et
  handleNextionInput();

  // 2. Adım: CAN BUS üzerinden radar verisi kontrol et
  twai_message_t message;
  bool detectionThisCycle = false;
  
  // 50ms zaman aşımı ile mesaj bekle
  if (twai_receive(&message, pdMS_TO_TICKS(50)) == ESP_OK) {
    
    // Sadece Radar ID aralığını işle (Örn: 0x310 - 0x38F)
    if (message.identifier >= 0x310 && message.identifier <= 0x38F) {
      
      // Continental Radar Geçerlilik Kontrolü (7. Byte'ın son biti)
      bool validDetection = !(message.data[7] & 0b00000001); 
      
      if (validDetection) {
        detectionThisCycle = true;
        handleDetection(message); // Ana radar mantığını çalıştır
      }
    }
  }

  // 3. Adım: Eğer bu döngüde hedef yoksa ve ekranda eski hedef kalmışsa temizle
  if (!detectionThisCycle && targetVisible) {
    clearDetection();
  }

  // 4. Adım: Buzzer'ı kontrol et (Bloklamadan çalışır)
  handleBuzzer();
}


// -------------------------------------------------------------------------------------------------
// 9. İLETİŞİM VE AYRIŞTIRMA (Parsing)
// -------------------------------------------------------------------------------------------------
void sendCommand(String cmd) {
  SerialNextion.print(cmd);
  SerialNextion.write(0xFF);
  SerialNextion.write(0xFF);
  SerialNextion.write(0xFF);
}

// =================================================================================
// NİHAİ ÇÖZÜM: Akıllı Veri Okuma (strstr ile Çöp Veriyi Atlar)
// =================================================================================
void handleNextionInput() {
  if (SerialNextion.available()) {
    // 1. Veriyi Oku
    // Timeout süresini biraz artırarak verinin tam gelmesini bekle
    SerialNextion.setTimeout(50); 
    int bytesRead = SerialNextion.readBytesUntil((char)0xFF, rxBuffer, RX_BUFFER_SIZE - 1);
    rxBuffer[bytesRead] = '\0'; 

    // Tamponu temizle
    while(SerialNextion.available() && SerialNextion.peek() == 0xFF) {
      SerialNextion.read();
    }

    if (bytesRead == 0) return;

    // --- DEBUG ---
    // Gelen ham veriyi görmek istersen şu satırı aç:
    // Serial.printf("[RAW]: %s\n", rxBuffer);

    char* cmdPtr; // Komutu bulacağımız adres

    // --- LOGIN (Şifre Kontrolü) ---
    // Nextion'dan "LOGIN:1234" geldiğinde burası yakalar
    if ((cmdPtr = strstr(rxBuffer, "LOGIN:")) != NULL) {
      char* enteredPass = cmdPtr + 6; // "LOGIN:" sonrasındaki şifre
      
      // Şifrenin sonundaki boşluk veya satır sonu karakterlerini temizle
      char* cleanPass = strtok(enteredPass, " \r\n");
      if(cleanPass == NULL) cleanPass = enteredPass; // Eğer strtok başarısızsa ham hali al

      // ESP32 Hafızasındaki (password) ile karşılaştır
      if (strcmp(cleanPass, password) == 0) {
        NEXTION_PRINTF("[LOGIN] Basarili.\n");
        sendSettingsToNextion();     // Ayarları ekrana gönder
        sendCommand("page pageSet1"); // Sayfayı değiştir
      } else {
        NEXTION_PRINTF("[LOGIN] Hatali: %s\n", cleanPass);
        sendCommand("tInfo.txt=\"Hatalı Sifre\""); // Ekrana hata yaz
        sendCommand("login_fail.val=1");           // Titreşim/Animasyon tetikle
      }
    }
    
    // --- SETPASS (Şifre Değiştirme) ---
    else if ((cmdPtr = strstr(rxBuffer, "SETPASS:")) != NULL) {
      char* newPass = cmdPtr + 8;
      char* cleanPass = strtok(newPass, " \r\n");
      
      if (cleanPass && strlen(cleanPass) > 0 && strlen(cleanPass) < 10) {
        strcpy(password, cleanPass);
        saveSettingsToEEPROM();
        NEXTION_PRINTF("[SETPASS] Yeni Sifre: %s\n", password);
      }
    }

    // --- SAVE1 (Bölgeler) ---
    else if ((cmdPtr = strstr(rxBuffer, "SAVE1:")) != NULL) {
      char* ptr = cmdPtr + 6;
      char* val1 = strtok(ptr, ",");
      char* val2 = strtok(NULL, ",");
      if (val1 && val2) {
        warningZone_m = atof(val1) / 10.0;
        dangerZone_m = atof(val2) / 10.0;
        saveSettingsToEEPROM();
      }
    }

    // --- SAVE2 (Araç) ---
    else if ((cmdPtr = strstr(rxBuffer, "SAVE2:")) != NULL) {
      char* ptr = cmdPtr + 6;
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

    // --- SAVE3 (Ses ve Zoom) ---
    else if ((cmdPtr = strstr(rxBuffer, "SAVE3:")) != NULL) {
      char* ptr = cmdPtr + 6;
      char* vZoom = strtok(ptr, ",");
      char* vAudio = strtok(NULL, ",");

      if (vZoom && vAudio) {
        autoZoom_enabled = (atoi(vZoom) == 1);
        bool newAudioState = (atoi(vAudio) == 1);
        
        NEXTION_PRINTF("[SAVE3] Zoom:%d, Ses:%d\n", autoZoom_enabled, newAudioState);

        // Ses kapatıldıysa Buzzer'ı ANINDA sustur
        if (!newAudioState) {
            digitalWrite(BUZZER_PIN, LOW);
            buzzerIsOn = false;
            buzzerShouldBeActive = false;
        }
        
        audioAlarm_enabled = newAudioState;
        saveSettingsToEEPROM();
      }
    }

    // --- RESET ---
    else if (strstr(rxBuffer, "RESETALL") != NULL) {
      resetToDefaults();
    }
  }
}
// -------------------------------------------------------------------------------------------------
// 10. RADAR HESAPLAMA VE GÖRSELLEŞTİRME MOTORU (CORE)
// -------------------------------------------------------------------------------------------------
void handleDetection(const twai_message_t& msg) {
  RADAR_PRINTLN("\n--- [RADAR] HEDEF SAPTANDI ---");
  
  // A. Ham Veriyi Fiziksel Birime Çevirme (Radar Protokolü)
  float polarRadius_m = msg.data[0] * 0.25;
  int   polarAngle_deg = (int)msg.data[1] - 128;
  float doc_x_m = msg.data[2] * 0.25;             // Simülasyondaki Y ekseni (İleri)
  float doc_y_m = ((int)msg.data[3] - 128) * 0.25;// Simülasyondaki X ekseni (Yanal)
  
  RADAR_PRINTF("  [DATA] Mesafe:%.2fm, X(İleri):%.2fm, Y(Yanal):%.2fm\n", polarRadius_m, doc_x_m, doc_y_m);

  // B. Otomatik Zoom ve Grid Genişliği Belirleme
  // React Simülasyonu ile aynı mantık: 10m -> 8m -> 6m -> 4m kademeleri
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
      // Zoom kapalıysa sabit en geniş açı (10m)
      currentMaxGridXMeters = 10.0;
      if (polarRadius_m > warningZone_m) { backgroundPicId = PIC_ID_SAFE; targetColor = COLOR_GREEN; }
      else if (polarRadius_m > dangerZone_m) { backgroundPicId = PIC_ID_WARNING; targetColor = COLOR_YELLOW; }
      else { backgroundPicId = PIC_ID_ALARM; targetColor = COLOR_RED; }
  }

  // C. EŞİT ÖLÇEKLEME (Fixed Scale / Uniform Scaling)
  // Simülasyondaki gibi, 1 metre hem yatayda hem dikeyde aynı piksel sayısına denk gelir.
  // Görüntünün sündürülmesini engeller.
  float fixedScale = (float)SCREEN_WIDTH_PX / currentMaxGridXMeters;

  // D. Koordinat Haritalama (Metreden Piksele)
  
  // X EKSENİ (YANAL):
  // Formül: (YanalMesafe + (GridGenişliği / 2)) * Ölçek
  float targetX_float = (doc_y_m + (currentMaxGridXMeters / 2.0)) * fixedScale;
  int targetX_px = (int)(targetX_float + 0.5);

  // Y EKSENİ (İLERİ):
  // Formül: EkranBoyu - (İleriMesafe * Ölçek)
  float targetY_float = (float)SCREEN_HEIGHT_PX - (doc_x_m * fixedScale);
  int targetY_px = (int)(targetY_float + 0.5);

  // E. Sınırlandırma (Constrain)
  // Hedefin ekran dışına çıkıp çizimi bozmasını engelle
  targetX_px = constrain(targetX_px, 0, SCREEN_WIDTH_PX - TARGET_OBJECT_SIZE_PX);
  targetY_px = constrain(targetY_px, 0, SCREEN_HEIGHT_PX - TARGET_OBJECT_SIZE_PX);

  // F. Buzzer (Sesli İkaz) Mantığı
  // Sadece araç genişliği + güvenlik payı içindeki cisimlere öter
  if (audioAlarm_enabled && (polarRadius_m < warningZone_m) && (abs(doc_y_m) < (vehicleRealWidth_m / 2.0 + sideMargin_m))) {
    buzzerShouldBeActive = true;
    if (polarRadius_m <= SOLID_TONE_DISTANCE_M) currentBeepInterval = 0;
    else if (backgroundPicId == PIC_ID_ALARM)   currentBeepInterval = BEEP_INTERVAL_RED_MS;
    else if (backgroundPicId == PIC_ID_DANGER)  currentBeepInterval = BEEP_INTERVAL_ORANGE_MS;
    else                                        currentBeepInterval = BEEP_INTERVAL_YELLOW_MS;
  } else {
    buzzerShouldBeActive = false;
  }

  // G. Ekran Güncelleme
  sendCommand("page0.pic=" + String(backgroundPicId));
  // Not: Araç genişliğini de aynı 'fixedScale' ile hesaplaması için grid bilgisini gönderiyoruz
  updateVehicleDisplay(currentMaxGridXMeters); 
  updateTargetDisplay(targetX_px, targetY_px, targetColor);
  updateTextDisplays(polarRadius_m, polarAngle_deg, doc_y_m, doc_x_m);
}

void updateVehicleDisplay(float currentMaxGridXMeters) {
  // Sıfıra bölünme koruması
  if (currentMaxGridXMeters < 0.1) currentMaxGridXMeters = 10.0;
  
  // Aynı ölçeği (Fixed Scale) burada da kullanıyoruz
  float fixedScale = (float)SCREEN_WIDTH_PX / currentMaxGridXMeters;
  
  // Araç genişliğini piksel cinsinden hesapla
  int vehicle_width_px = (int)((vehicleRealWidth_m * fixedScale) + 0.5);

  // --- KRİTİK DÜZELTME (Araç Genişliği Sınırı) ---
  // Eğer zoom çok artarsa (örn. 4m modunda) ve araç genişse (2.5m),
  // hesaplanan piksel ekranı taşabilir. Nextion ekran boyutundan büyük
  // genişlikleri çizemediği için koordinatlar şaşar.
  if (vehicle_width_px > SCREEN_WIDTH_PX) {
    vehicle_width_px = SCREEN_WIDTH_PX;
  }
  // Minimum görünürlük sınırı
  if (vehicle_width_px < 2) {
    vehicle_width_px = 2;
  }

  // Aracı ekranda ortala
  int vehicle_x_px = (int)(((float)SCREEN_WIDTH_PX - vehicle_width_px) / 2.0 + 0.5);
  
  // Negatif koordinat koruması
  if (vehicle_x_px < 0) vehicle_x_px = 0;

  // Nextion'a Gönder
  sendCommand("rVehicle.x=" + String(vehicle_x_px));
  sendCommand("rVehicle.w=" + String(vehicle_width_px));
  sendCommand("rVehicle.y=" + String(SCREEN_HEIGHT_PX - VEHICLE_HEIGHT_PX));
  sendCommand("rVehicle.h=" + String(VEHICLE_HEIGHT_PX));
  sendCommand("rVehicle.bco=" + String(VEHICLE_COLOR));
}

void clearDetection() {
  targetVisible = false;
  buzzerShouldBeActive = false; 
  
  // Hedefi gizle, arka planı güvenli moda al
  sendCommand("vis rTarget,0");
  sendCommand("page0.pic=" + String(PIC_ID_SAFE));
  
  // Araç genişliğini varsayılan 10m görünümüne sıfırla
  updateVehicleDisplay(10.0);
  
  // Metinleri temizle
  sendCommand("tDurum.txt=\"Temiz\"");
  sendCommand("tMesafe.txt=\"--\"");
  sendCommand("tAci.txt=\"--\"");
  sendCommand("tX.txt=\"--\"");
  sendCommand("tY.txt=\"--\"");
}

void updateTargetDisplay(int x, int y, int color) {
  // Hedef daha önce gizliyse görünür yap
  if (!targetVisible) {
    sendCommand("vis rTarget,1");
    targetVisible = true;
  }
  // Konum ve renk güncelle
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
  // --- MASTER SWITCH (ANA ŞALTER) ---
  // Eğer ses ayarı kapalıysa, alarm durumu ne olursa olsun buzzer'ı SUSTUR.
  if (!audioAlarm_enabled) {
    if (buzzerIsOn) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerIsOn = false;
      BUZZER_PRINTLN("[BUZZER] Ayar kapalı olduğu için susturuldu.");
    }
    return; // Fonksiyondan hemen çık
  }

  // --- NORMAL MANTIK ---
  if (!buzzerShouldBeActive) {
    if (buzzerIsOn) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerIsOn = false;
    }
    return;
  }

  unsigned long currentTime = millis();

  // Sürekli Ses (Kritik Yakınlık)
  if (currentBeepInterval == 0) {
    if (!buzzerIsOn) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerIsOn = true;
    }
    return;
  }

  // Kesikli Ses
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


// -------------------------------------------------------------------------------------------------
// 11. EEPROM (KALICI HAFIZA) YÖNETİMİ
// -------------------------------------------------------------------------------------------------
void loadSettingsFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  // Magic Key kontrolü ile hafızanın boş olup olmadığını anla
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
  EEPROM.commit(); // Veriyi fiziksel olarak yaz
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
  // Nextion tam sayı (integer) çalıştığı için değerleri 10 ile çarpıp gönderiyoruz
  sendCommand("pageSet1.h0.val=" + String((int)(warningZone_m * 10)));
  sendCommand("pageSet1.h1.val=" + String((int)(dangerZone_m * 10)));
  sendCommand("pageSet2.h0.val=" + String((int)(sideMargin_m * 10)));
  sendCommand("pageSet2.h1.val=" + String((int)(vehicleRealWidth_m * 10)));
  sendCommand("pageSet2.h2.val=" + String((int)(maxWidth_m * 10)));
  sendCommand("pageSet3.btZoom.val=" + String(autoZoom_enabled ? 1 : 0));
  sendCommand("pageSet3.btAudio.val=" + String(audioAlarm_enabled ? 1 : 0));
}