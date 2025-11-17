# ESP32 CAN-Bus Radar ve Nextion HMI Projesi (RCPS-1SA)

Bu proje, bir ESP32 geliştirme kartı kullanarak CAN-Bus üzerinden **Brigade BS-9100T** gibi bir radar sensöründen gelen verileri okur ve bu verileri bir **Nextion HMI** ekranda hem metinsel olarak hem de görsel bir radar arayüzünde gösterir.

Proje, `EspNextion_Target.ino` dosyasında tanımlandığı üzere, algılanan nesneleri ekranda hareket eden bir hedef olarak görselleştirir.

##  ዋና Özellikler

- **CAN-Bus İletişimi:** 500 Kbit/s hızında CAN-Bus verilerini dinler ve işler.
- **Radar Veri İşleme:** Brigade sensörlerinden gelen polar ve kartezyen koordinat verilerini ayrıştırır.
- **Nextion HMI Entegrasyonu:** UART üzerinden Nextion ekran ile haberleşir.
- **Görsel Arayüz:** Algılanan nesnelerin konumunu, mesafesini ve açısını ekranda gösterir. `EspNextion_Target.ino` ile hedefleri görsel olarak harita üzerinde çizer.
- **Durum Bildirimi:** CAN-Bus bağlantı durumu, veri alımı gibi bilgileri ekrana yazdırır.

## ⚙️ Donanım ve Pin Bağlantıları

| Komponent | Pin | ESP32 Pini | Açıklama |
| :--- | :--- | :--- | :--- |
| **CAN Transceiver** | CTX | `GPIO 5` | CAN-Bus Veri Gönderme |
| | CRX | `GPIO 4` | CAN-Bus Veri Alma |
| **Nextion HMI Ekran**| RX | `GPIO 17` (TX2) | ESP32'den Ekrana Veri |
| | TX | `GPIO 16` (RX2) | Ekrandan ESP32'ye Veri |
| **ESP32** | 5V | - | Güç |
| | GND | - | Toprak |

**Önemli:** Kullandığınız CAN alıcı-verici modülünün (örn: `SN65HVD230`) ESP32'nin 3.3V mantık seviyesi ile uyumlu olduğundan emin olun.

## 🔧 Kurulum ve Kullanım

Bu proje **PlatformIO** ortamı için yapılandırılmıştır.

1.  **PlatformIO Kurulumu:** Eğer kurulu değilse, VS Code için [PlatformIO IDE eklentisini](https://platformio.org/install/ide?install=vscode) kurun.
2.  **Projeyi Açma:** Bu repoyu klonladıktan sonra VS Code içinde `File > Open Folder...` menüsünden `RCPS` klasörünü açın.
3.  **Kütüphaneler:** `platformio.ini` dosyasında belirtilen kütüphaneler otomatik olarak yüklenecektir. Ekstra kütüphaneleriniz varsa `lib_extra_dirs` altında belirttiğiniz yolda olduğundan emin olun.
4.  **Derleme ve Yükleme:** PlatformIO arayüzündeki "Upload" butonuna basarak projeyi ESP32 kartınıza yükleyebilirsiniz.

## 💻 Kod Yapısı

-   `src/esp32canonly.ino`: CAN-Bus'tan gelen radar verilerini okur ve bu verileri (mesafe, açı, koordinatlar) Nextion ekrandaki metin alanlarına yazdırır.
-   `src/EspNextion_Target.ino`: Daha gelişmiş bir versiyondur. Gelen koordinat verilerini ekranın piksel boyutlarına göre ölçeklendirerek, `pTarget` adında bir resim nesnesini ekranda hareket ettirir ve canlı bir hedef takibi sağlar.
-   `platformio.ini`: Proje yapılandırma dosyasıdır. Kart tipi (`esp32dev`), framework (`arduino`) ve kütüphane yolları gibi ayarları içerir.
-   `include/` ve `lib/`: Kendi özel başlık dosyalarınızı ve kütüphanelerinizi ekleyebileceğiniz klasörler.
