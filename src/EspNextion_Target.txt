/*
 * ESP32 Nextion 4.3" Dikey Ekran için CAN Radar Alıcı Kodu (Görsel Hedef Takibi)
 * G.T
 * 13.11.2025
 */

#include "driver/gpio.h"
#include "driver/twai.h"
#include <math.h>
#include <HardwareSerial.h>

// CAN pinleri
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

// --- EKRAN VE RADAR KONFİGÜRASYONU (4.3" 272x480 Dikey Ekran için) ---
const int SCREEN_WIDTH_PX = 272;  // Ekranınızın piksel cinsinden genişliği (Dikey mod)
const int SCREEN_HEIGHT_PX = 480; // Ekranınızın piksel cinsinden yüksekliği (Dikey mod)
const float MAX_FORWARD_RANGE_M = 7.0; // İleri doğru maksimum mesafe
const float MAX_LATERAL_RANGE_M = 3.5; // Merkezden sağa/sola maksimum yanal mesafe
// --------------------------------------------------------------------

// Nextion için seri port (UART2)
HardwareSerial SerialNextion(2);

// Nextion'a komut gönderme fonksiyonu
void sendCommand(String cmd) {
  SerialNextion.print(cmd);
  SerialNextion.write(0xFF);
  SerialNextion.write(0xFF);
  SerialNextion.write(0xFF);
}

void setup() {
  Serial.begin(115200);
  SerialNextion.begin(9600, SERIAL_8N1, 16, 17); // RX:16, TX:17

  Serial.println("-- Radar Alıcısı Başlatıldı (4.3\" Dikey Ekran) --");

  // TWAI (CAN) Sürücüsü Kurulumu
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK || twai_start() != ESP_OK) {
    Serial.println("HATA: TWAI sürücüsü başlatılamadı.");
    sendCommand("tDurum.txt=\"CAN Hatasi\"");
    while(1);
  }

  Serial.println("TWAI sürücüsü başarıyla başlatıldı. Mesajlar dinleniyor...");
  sendCommand("tDurum.txt=\"Dinleniyor...\"");
  sendCommand("vis pTarget,0"); // Başlangıçta hedefi gizle
}

bool targetVisible = false;

void loop() {
  twai_message_t message;
  bool detectionThisCycle = false;

  if (twai_receive(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
    if (message.identifier >= 0x310 && message.identifier <= 0x38F) {
      bool validDetection = !(message.data[7] & 0b00000001);

      if (validDetection) {
        detectionThisCycle = true;
        
        float doc_x_m = message.data[2] * 0.25;
        float doc_y_m = ((int)message.data[3] - 128) * 0.25;

        // --- METRE'den PİKSEL'e KOORDİNAT DÖNÜŞÜMÜ ---
        int targetY_px = (int)(((MAX_FORWARD_RANGE_M - doc_x_m) / MAX_FORWARD_RANGE_M) * SCREEN_HEIGHT_PX);
        float scale = doc_y_m / MAX_LATERAL_RANGE_M;
        int targetX_px = (int)((SCREEN_WIDTH_PX / 2.0) + (scale * (SCREEN_WIDTH_PX / 2.0)));
        
        // Sınırlama: Hedef nesnenizin boyutunu (örneğin 20x20 piksel) dikkate alarak ekran dışına taşmasını engelle
        int object_size = 20; 
        targetX_px = constrain(targetX_px, 0, SCREEN_WIDTH_PX - object_size);
        targetY_px = constrain(targetY_px, 0, SCREEN_HEIGHT_PX - object_size);

        // Hedefi göster ve yeni pozisyonuna taşı
        if (!targetVisible) {
          sendCommand("vis pTarget,1");
          targetVisible = true;
        }
        sendCommand("pTarget.x=" + String(targetX_px));
        sendCommand("pTarget.y=" + String(targetY_px));

        // Metin verilerini de güncelleyebilirsiniz (isteğe bağlı)
        sendCommand("tDurum.txt=\"Algilandi\"");
      }
    }
  }

  // Eğer bu döngüde algılama olmadıysa ve hedef görünürse, gizle
  if (!detectionThisCycle && targetVisible) {
    sendCommand("vis pTarget,0");
    targetVisible = false;
    sendCommand("tDurum.txt=\"Temiz\"");
  }
}