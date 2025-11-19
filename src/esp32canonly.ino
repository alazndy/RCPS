/*
 * ESP32 Nextion için CAN Radar Alıcı Kodu
 * SN65HVD230 ve ESP32-WROOM32D için
 * Brigade BS-9100T Sensör
 * ADC RCPS-1SA Projesi
 * G.T
 * 13.11.2025
 */

#include "driver/gpio.h"
#include "driver/twai.h"
#include <math.h>
#include <HardwareSerial.h> // YENİ: Nextion için ikinci seri portu kullanmayı sağlar

// CAN pinleri
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

// YENİ: Nextion için ayrı bir seri port tanımlıyoruz (UART2).
// ESP32 GPIO 17 (TX2) -> Nextion RX
// ESP32 GPIO 16 (RX2) -> Nextion TX
HardwareSerial SerialNextion(2);

// YENİ: Nextion'a komut göndermeyi kolaylaştıran fonksiyon.
// Her komutun sonuna gerekli olan 3 adet 0xFF byte'ını otomatik ekler.
void sendCommand(String cmd) {
  SerialNextion.print(cmd);
  SerialNextion.write(0xFF);
  SerialNextion.write(0xFF);
  SerialNextion.write(0xFF);
}

void setup() {
  // Hata ayıklama için USB üzerinden çalışan standart Seri Monitör
  Serial.begin(115200);
  while (!Serial);
  Serial.println("-- Waveshare ESP32-S3 & Brigade Radar Alıcısı (Doğru Pinler) --");
  
  // YENİ: Nextion ile haberleşecek seri portu başlatıyoruz.
  // Nextion'ın varsayılan hızı 9600'dür. Farklı bir hıza ayarladıysanız buradan değiştirin.
  SerialNextion.begin(9600, SERIAL_8N1, 16, 17); // (baud, config, RX_PIN, TX_PIN)

  // TWAI (CAN) Sürücüsü Kurulumu
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN,
    (gpio_num_t)CAN_RX_PIN,
    TWAI_MODE_NORMAL
  );
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK || twai_start() != ESP_OK) {
    Serial.println("HATA: TWAI sürücüsü başlatılamadı. Lütfen pin bağlantılarını kontrol edin.");
    sendCommand("tDurum.txt=\"CAN Hatasi\""); // YENİ: Hata durumunda ekrana bilgi gönder
    while(1);
  }

  Serial.println("TWAI sürücüsü başarıyla başlatıldı. Mesajlar dinleniyor...");
  sendCommand("tDurum.txt=\"Dinleniyor...\""); // YENİ: Başlangıçta ekrana durum bilgisi gönder
  Serial.println("---------------------------------------------------------------------");
}

void loop() {
  twai_message_t message;

  if (twai_receive(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
    if (message.identifier >= 0x310 && message.identifier <= 0x38F) {
      bool validDetection = !(message.data[7] & 0b00000001);

      if (validDetection) {
        // Verileri anlamlı hale getir
        float polarRadius_m = message.data[0] * 0.25;
        int polarAngle_deg = (int)message.data[1] - 128;
        float doc_x_m = message.data[2] * 0.25;
        float doc_y_m = ((int)message.data[3] - 128) * 0.25;

        // --- DEĞİŞİKLİK: Verileri hem Seri Monitör'e hem de Nextion'a gönder ---

        // 1. Seri Monitör'e yazdırmaya devam et (Hata ayıklama için kullanışlıdır)
        Serial.println("+++ NESNE ALGILANDI +++");
        Serial.print("  - Mesafe (Polar): "); Serial.print(polarRadius_m); Serial.println(" metre");
        Serial.print("  - Açı (Polar): "); Serial.print(polarAngle_deg); Serial.println(" derece");
        Serial.print("  - Koordinatlar (X,Y): "); Serial.print(doc_x_m); Serial.print("m, "); Serial.print(doc_y_m); Serial.println("m");
        Serial.println("---------------------------------------------------------------------");

        // 2. Nextion Ekranına komutları gönder
        // Komut formatı: nesne_adi.ozellik="deger"
        // String() fonksiyonu ile sayısal değerleri metne çeviriyoruz.
        // İkinci parametre ondalık hassasiyetini belirtir (örn: String(polarRadius_m, 2))
        sendCommand("tDurum.txt=\"Algilandi\"");
        sendCommand("tMesafe.txt=\"" + String(polarRadius_m, 2) + " m\"");
        sendCommand("tAci.txt=\"" + String(polarAngle_deg) + " derece\"");
        sendCommand("tX.txt=\"X: " + String(doc_x_m, 2) + " m\"");
        sendCommand("tY.txt=\"Y: " + String(doc_y_m, 2) + " m\"");
      }
    }
  }
}