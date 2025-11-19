## RCPS Radar Collision Prevention System
# ESP32 & Nextion - Gelişmiş CAN BUS Radar Sistemi

Bu proje, bir ESP32 mikrodenetleyici kullanarak bir CAN BUS radarından gelen verileri işleyen ve sonuçları bir Nextion HMI ekranda görselleştiren gelişmiş bir radar sistemidir. Sistem, hedefleri mesafelerine ve konumlarına göre sınıflandırır, görsel ve sesli uyarılar üretir ve kullanıcı tarafından yapılandırılabilir ayarlara sahiptir.

**Versiyon:** v3.7.0 (Nextion Auth & Smart Parser)

---

## 🌟 Temel Özellikler

-   **CAN BUS Entegrasyonu:** Radar sensöründen gelen verileri `TWAI` (Two-Wire Automotive Interface) sürücüsü aracılığıyla alır ve işler.
    -   **Mesaj Filtreleme:** Belirli CAN ID aralığındaki (`0x310` - `0x38F`) mesajları dinler.
    -   **Veri Çözümleme:** Gelen CAN verisinden polar mesafe, açı, ileri ve yanal mesafeleri (metre cinsinden) çıkarır.
-   **Nextion HMI Arayüzü:** Algılanan hedefleri, tehlike bölgelerini ve araç konumunu dinamik olarak bir Nextion ekranda gösterir. Ayarlar için dokunmatik bir arayüz sunar.
    -   **Akıllı Ayrıştırıcı:** Nextion'dan gelen komutları `strstr` kullanarak güvenilir bir şekilde ayrıştırır, "Touch Event" gibi istenmeyen verileri göz ardı eder.
    -   **Nextion Kimlik Doğrulaması:** Şifre kontrolü ve ayar menüsü erişimi tamamen Nextion HMI tarafından yönetilir. ESP32 sadece ayar komutlarını işler.
-   **Dinamik Görselleştirme:**
    -   **Otomatik Zoom:** "Auto Zoom" özelliği sayesinde, hedefin mesafesine göre ekranın yanal görüş mesafesini otomatik olarak ayarlar (10m, 8m, 6m, 4m kademeleri).
    -   **Sabit Ölçekleme:** X ve Y eksenleri eşit ölçeklenerek daha doğru bir görsel temsil sağlar.
-   **Kademeli Sesli Alarm:** Hedefin yakınlığına göre farklı aralıklarla (sarı, turuncu, kırmızı bölge) veya sürekli (çok yakın) ses çıkaran bir buzzer ile sesli uyarı sağlar.
    -   **Master Ses Kontrolü:** Sesli alarm kapatıldığında buzzer donanımsal olarak anında susturulur.
-   **Kalıcı Ayarlar (EEPROM):** Uyarı/tehlike bölgeleri, araç genişliği, yan boşluklar ve maksimum tarama genişliği gibi ayarlar ESP32'nin dahili EEPROM'una kaydedilerek güç kesintilerinde bile korunur.
-   **Gelişmiş Hata Ayıklama:** Kod, `CAN`, `Nextion`, `Radar`, `Buzzer` ve `EEPROM` modülleri için ayrı ayrı etkinleştirilebilen bir hata ayıklama sistemine sahiptir. Bu, sorun gidermeyi kolaylaştırır.

---

## 🛠️ Donanım ve Yazılım Gereksinimleri

### Donanım
-   **Mikrodenetleyici:** ESP32 Geliştirme Kartı (Örn: ESP32-DevKitC, `esp32dev` PlatformIO kart tanımı)
-   **Ekran:** Nextion HMI Dokunmatik Ekran (Örn: 3.5 inç veya 4.3 inç)
-   **Radar Sensörü:** CAN BUS (TWAI) arayüzüne sahip bir radar modülü (Örn: Otomotiv radar sensörleri).
-   **CAN Alıcı-Verici:** ESP32'nin 3.3V mantık seviyelerini CAN Bus'ın fiziksel katmanına dönüştürmek için bir CAN alıcı-verici modülü (Örn: SN65HVD230, TJA1050).
-   **Buzzer:** Sesli uyarılar için aktif veya pasif bir buzzer. (Pasif buzzer kullanılıyorsa, basit bir transistör devresi ile kontrol edilmesi önerilir.)
-   **Güç Kaynağı:** ESP32 ve diğer bileşenler için uygun bir 5V güç kaynağı.
-   **Bağlantı Kabloları**

#### Bileşen Detayları ve Özellikleri

Projede kullanılan temel donanım bileşenlerinin detaylı özellikleri aşağıda verilmiştir:

*   **ESP32 Mikrodenetleyici:**
    *   **İşlemci:** Tensilica Xtensa Dual-Core 32-bit LX6 mikroişlemci
    *   **Saat Hızı:** 240 MHz'e kadar
    *   **Bağlantı:** Entegre Wi-Fi (802.11 b/g/n) ve Bluetooth (v4.2 BR/EDR ve BLE)
    *   **Bellek:** 520 KB SRAM, 4MB Flash (genellikle)
    *   **GPIO:** Çok sayıda çok amaçlı GPIO pini, ADC, DAC, I2C, SPI, UART, TWAI (CAN) desteği.
    *   **Güç:** 2.3V - 3.6V çalışma voltajı (genellikle 3.3V ile beslenir)

*   **Nextion HMI Dokunmatik Ekran:**
    *   **Tip:** Akıllı Seri HMI (Human Machine Interface) Dokunmatik Ekran
    *   **Boyutlar:** Çeşitli boyutlarda mevcuttur (örn: 3.5", 4.3", 5.0", 7.0"). Projede kullanılan HMI dosyası ekran çözünürlüğüne göre optimize edilmelidir.
    *   **İletişim:** UART Seri Port (TTL) üzerinden ESP32 ile iletişim kurar (9600 baud).
    *   **Özellikler:** Entegre dokunmatik panel, dahili flaş bellek (kullanıcı arayüzü ve resimler için), GPIO kontrolü (bazı modellerde).
    *   **Güç:** Genellikle 5V DC ile beslenir.

*   **CAN Transceiver (Örn: SN65HVD230, TJA1050):**
    *   **İşlev:** CAN protokol kontrolörü ile fiziksel CAN Bus hattı arasında arayüz sağlar. CAN sinyallerini diferansiyel sinyallere çevirir ve tersini yapar.
    *   **Mantık Seviyesi:** Genellikle 3.3V ve 5V mantık seviyeleriyle uyumludur.
    *   **Hız:** Yüksek hızlı CAN iletişimini destekler (Projede 500 kbit/s).
    *   **Koruma:** Genellikle kısa devre koruması, aşırı sıcaklık koruması gibi özelliklere sahiptir.

*   **Buzzer:**
    *   **Tip:** Aktif veya Pasif Buzzer. Proje, buzzer'ı dijital pin kontrolü ile AÇIK/KAPALI durumuna getirir. Aktif buzzer veya transistörle kontrol edilen pasif buzzer için uygundur.
    *   **Güç:** Genellikle 3.3V veya 5V ile çalışır.

*   **Radar Sensör (Brigade Backsense® BS-9100 / BS-9100T):**
    *   **Model:** BS-9100 / BS-9100T (FMCW Radar Nesne Algılama Sistemi)
    *   **Algılama Uzunluğu:** 0.25m - 60m (İletişim kurabilen maksimum 16 nesne algılar)
    *   **Algılama Genişliği:** 0m - 16m
    *   **Nominal Tolerans:** ±0.25m
    *   **Radar Işın Açısı:** Yatay 140°, Dikey 16° (sensör ön yüzeyine simetrik olarak dik)
    *   **Mesafe Çözünürlüğü:** 0.25m
    *   **Açılış Süresi:** ≤ 2.5 saniye (Sistemin hazır olması için güç açma)
    *   **Eşzamanlı Algılanan Nesne Sayısı:** Sensör başına maksimum 16 nesne
    *   **Sistem Başına Maksimum Sensör:** 8
    *   **Çalışma Frekansı:** 77GHz (FMCW - Frekans Modülasyonlu Sürekli Dalga)
    *   **Boyutlar:** 160 x 100 x 40 mm
    *   **Ağırlık:** 0.34kg (kablo dahil)
    *   **Çalışma Sıcaklığı:** -40°C ila +85°C
    *   **IP Koruması:** IP69K (toz ve basınçlı su jetlerinden korumalı), Konnektör için IP66K, IP67
    *   **Giriş Voltaj Aralığı:** 9 - 32 Vdc
    *   **Giriş Akımı (sensör başına):** Tipik 0.23A @ 12Vdc / Tipik 0.12A @ 24Vdc
    *   **CAN Bus Standardı:** CAN 2.0A Temel Çerçeve Formatı (11-bit Tanımlayıcı)
    *   **CAN Baud Hızı:** 500 kbit/s (Sabit, yapılandırılamaz)
    *   **CAN Algılama Mesajı ID Aralığı:** Sensör ID'sine ve nesnenin yakınlığına bağlı olarak 0x310 - 0x38F aralığında değişir. (Örn: En yakın nesne için 0x310, 0x320, ..., 0x380)

### Yazılım
-   **Geliştirme Ortamı:** [PlatformIO IDE](https://platformio.org/) (VS Code eklentisi önerilir)
-   **Framework:** [Arduino Framework for ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/)
-   **Gerekli Kütüphaneler:**
    -   `driver/gpio.h`, `driver/twai.h` (ESP-IDF'in bir parçası, Arduino ESP32 çekirdeği ile gelir)
    -   `<math.h>` (Standart C kütüphanesi)
    -   `<HardwareSerial.h>` (Arduino çekirdeği ile gelir)
    -   `<EEPROM.h>` (Arduino çekirdeği ile gelir)

---

## 🔌 Bağlantı Şemaları (Metin Tabanlı)

**ÖNEMLİ NOT:** Aşağıdaki bağlantı şemaları metin tabanlıdır. Daha iyi bir görsel rehberlik için, projenizi kurarken bu şemaları referans alarak kendi görsel bağlantı diyagramlarınızı oluşturmanız veya ilgili donanım belgelerine başvurmanız şiddetle önerilir.

#### 1. ESP32 ile CAN Transceiver (Örn: SN65HVD230, TJA1050) Bağlantısı

CAN Transceiver modülü, ESP32'nin TWAI (CAN) sinyallerini fiziksel CAN Bus hattına uygun hale getirir.

| ESP32 Pin Adı | ESP32 GPIO Numarası | CAN Transceiver Pin Adı | Açıklama |
| :------------ | :------------------ | :---------------------- | :------------------------------------------- |
| `CAN_TX` | `GPIO_NUM_5` | `TX` | CAN Veri Hattı (Transmit) |
| `CAN_RX` | `GPIO_NUM_4` | `RX` | CAN Veri Hattı (Receive) |
| `GND` | `GND` | `GND` | Toprak Bağlantısı |
| `3V3` | `3V3` | `VCC` | Güç Bağlantısı (3.3V) |
| `CAN_H` | N/A | `CAN_H` | CAN Bus Yüksek Hattı (Harici CAN cihazlarına) |
| `CAN_L` | N/A | `CAN_L` | CAN Bus Düşük Hattı (Harici CAN cihazlarına) |

#### 2. ESP32 ile Nextion HMI Ekran Bağlantısı

Nextion ekran, ESP32 ile UART (Seri Haberleşme) üzerinden iletişim kurar. Projede `Serial2` kullanılmıştır.

| ESP32 Pin Adı | ESP32 GPIO Numarası | Nextion Ekran Pin Adı | Açıklama |
| :------------ | :------------------ | :-------------------- | :------------------------------------------- |
| `RX2` | `GPIO_NUM_16` | `TX` | ESP32'den Nextion'a Veri Gönderimi |
| `TX2` | `GPIO_NUM_17` | `RX` | Nextion'dan ESP32'ye Veri Alımı |
| `GND` | `GND` | `GND` | Toprak Bağlantısı |
| `5V` | `5V` | `5V` | Güç Bağlantısı (Nextion genellikle 5V ile çalışır) |

#### 3. ESP32 ile Buzzer Bağlantısı

Buzzer, ESP32'nin dijital bir çıkış pini üzerinden kontrol edilir.

| ESP32 Pin Adı | ESP32 GPIO Numarası | Buzzer Pin Adı | Açıklama |
| :------------ | :------------------ | :------------- | :------------------------------------------- |
| `BUZZER_PIN` | `GPIO_NUM_25` | `+` (Pozitif) | Buzzer'ın pozitif bacağına bağlanır |
| `GND` | `GND` | `-` (Negatif) | Buzzer'ın negatif bacağına bağlanır |

**Not:** Pasif buzzer kullanılıyorsa, ses üretmek için PWM sinyali gerekebilir. Bu projede buzzer'ın sadece AÇIK/KAPALI durumları kontrol edilmektedir, bu da aktif buzzer veya basit bir transistör devresi ile pasif buzzer kontrolü için uygundur.

---

## ⚙️ Proje Yapılandırması ve Kurulum

1.  **PlatformIO Projesi:** Bu proje bir PlatformIO projesidir. PlatformIO CLI veya VS Code eklentisini kullanarak projeyi açın.
2.  **Donanım Bağlantıları:** Yukarıdaki "Bağlantı Şemaları" bölümünü referans alarak tüm donanım bileşenlerini ESP32'ye doğru şekilde bağlayın.
3.  **Nextion HMI Dosyası:** `RCPS1SA.HMI` dosyasını Nextion editörü aracılığıyla Nextion ekranınıza yükleyin. Bu dosya, kullanıcı arayüzünü ve şifre doğrulama mantığını içerir.
4.  **Derleme ve Yükleme:** PlatformIO arayüzünü kullanarak projeyi derleyin (`Build`) ve ESP32 kartına yükleyin (`Upload`).

---

## 🔗 İlgili Simülasyon Projeleri

Bu projenin geliştirilmesi ve test edilmesi süreçlerinde faydalanabileceğiniz veya farklı amaçlarla kullanabileceğiniz ilişkili simülasyon projeleri aşağıda listelenmiştir:

-   **Radar Sensör Simülasyonu (bs9100tsim):**
    -   **Amaç:** Gerçek bir CAN BUS radar sensörünün davranışını taklit ederek, ESP32 radar sistemine veri sağlamak. Bu sayede, fiziksel bir sensör donanımına ihtiyaç duymadan projenin radar veri işleme, görselleştirme ve sesli uyarı mantığını geliştirebilir ve test edebilirsiniz.
    -   **İşlevsellik:** Belirli CAN ID aralığında (`0x310` - `0x38F`) simüle edilmiş radar verileri (mesafe, açı, X/Y koordinatları) üretir ve CAN Bus üzerinden yayınlar. Bu veriler, ana ESP32 projesi tarafından alınır ve işlenir.
    -   [GitHub Deposu: alazndy/bs9100tsim](https://github.com/alazndy/bs9100tsim)

-   **Nextion Ekran Simülasyonu (RCPS-Sim):**
    -   **Amaç:** Nextion HMI ekranının kullanıcı arayüzünü ve ESP32 ile olan seri iletişimini bilgisayar ortamında simüle etmek. Bu, gerçek bir Nextion ekranına sahip olmadan veya donanım bağlantılarıyla uğraşmadan, grafik arayüz güncellemelerini ve ayar komutlarının işlenmesini test etmenizi sağlar.
    -   **İşlevsellik:** ESP32'den gelen Nextion komutlarını yorumlar ve sanal bir arayüzde görselleştirir. Ayrıca, kullanıcı etkileşimlerini (buton basmaları, ayar değişiklikleri) simüle ederek ESP32'ye geri komutlar gönderebilir.
    -   [GitHub Deposu: alazndy/RCPS-Sim](https://github.com/alazndy/RCPS-Sim)

---

## 🚀 Kullanım

-   **İlk Başlatma:** Cihaz ilk kez başlatıldığında veya EEPROM'da geçerli ayarlar bulunmadığında, varsayılan ayarlar otomatik olarak yüklenir ve EEPROM'a kaydedilir.
-   **Radar Ekranı:** Ana ekran, algılanan hedefleri aracınıza göre konumlandırır.
    -   **Hedef Görselleştirme:** Hedefin rengi tehlike seviyesini belirtir (Yeşil -> Sarı -> Turuncu -> Kırmızı).
    -   **Metin Bilgileri:** Ekranın üst kısmında mesafe, açı, X ve Y koordinatları gibi anlık hedef bilgileri gösterilir.
-   **Ayarlar Menüsü:**
    -   Nextion ekranındaki ilgili butona dokunarak ayarlar menüsüne erişin.
    -   **Şifre Doğrulama:** Ayarlar menüsüne erişim Nextion HMI tarafından yönetilen bir şifre ile korunmaktadır. Varsayılan şifre `1234`'tür.
    -   **Ayarlar Sayfaları:**
        -   **Sayfa 1 (Bölgeler):** Radar için **Uyarı Bölgesi** ve **Tehlike Bölgesi** mesafelerini (metre cinsinden) ayarlayın.
        -   **Sayfa 2 (Araç Boyutları):** Aracınızın **Yan Boşluklarını**, **Gerçek Genişliğini** ve radarın algılayabileceği **Maksimum Yanal Tarama Genişliğini** (metre cinsinden) ayarlayın.
        -   **Sayfa 3 (Sistem Seçenekleri):** **Otomatik Zoom** özelliğini (hedef mesafesine göre ekran ölçeğini ayarlar) ve **Sesli Alarm** özelliğini açıp kapatın.
    -   **Şifre Değiştirme:** Nextion arayüzü üzerinden şifrenizi değiştirebilirsiniz.
    -   **Varsayılanlara Sıfırlama:** Tüm ayarları fabrika varsayılan değerlerine döndürebilirsiniz.

---

## 👨‍💻 Kod Yapısı

Kod, daha iyi okunabilirlik ve yönetim için mantıksal bölümlere ayrılmıştır:

-   **PROJE KİMLİĞİ:** Proje adı, versiyon, tarih ve sürüm notları gibi genel bilgiler.
-   **DEBUG AYARLARI:** `DEBUG_CAN`, `DEBUG_NEXTION`, `DEBUG_RADAR`, `DEBUG_BUZZER`, `DEBUG_EEPROM` makroları ile her modül için ayrı ayrı hata ayıklama mesajlarını etkinleştirme/devre dışı bırakma.
-   **DONANIM VE SABİTLER:**
    -   **Pin Tanımlamaları:** `CAN_TX_PIN`, `CAN_RX_PIN`, `BUZZER_PIN` gibi donanım pinlerinin GPIO numaraları.
    -   **Seri Haberleşme Ayarları:** `SERIAL_MONITOR_BAUD`, `NEXTION_BAUD` gibi baud hızları.
    -   **EEPROM Ayarları:** `EEPROM_SIZE`, `EEPROM_MAGIC_KEY` ve ayarların EEPROM'daki adresleri (`ADDR_WARN_ZONE`, `ADDR_DANGER_ZONE` vb.).
    -   **Varsayılan Ayarlar:** `DEFAULT_WARNING_ZONE_M`, `DEFAULT_VEHICLE_WIDTH_M` gibi başlangıç değerleri.
    -   **Ekran Özellikleri:** `SCREEN_WIDTH_PX`, `SCREEN_HEIGHT_PX`, `TARGET_OBJECT_SIZE_PX` gibi Nextion ekran boyutları ve görsel sabitler.
    -   **Nextion Resim ID'leri:** Farklı tehlike seviyeleri için kullanılan arka plan resimlerinin ID'leri.
    -   **Renkler:** Nextion ekranında kullanılan renk kodları.
    -   **Buzzer Ayarları:** `SOLID_TONE_DISTANCE_M`, `BEEP_ON_DURATION_MS`, `BEEP_INTERVAL_YELLOW_MS` gibi buzzer davranışını kontrol eden sabitler.
-   **GLOBAL DEĞİŞKENLER:** `HardwareSerial SerialNextion`, `targetVisible`, `rxBuffer` gibi global nesneler ve ayar değişkenleri (`warningZone_m`, `autoZoom_enabled` vb.).
-   **PROTOTİPLER:** Tüm fonksiyonların prototip bildirimleri.
-   **SETUP:** `setup()` fonksiyonu, pinleri ayarlar, seri haberleşmeyi başlatır, EEPROM'dan ayarları yükler ve TWAI (CAN) sürücüsünü başlatır.
-   **LOOP:** `loop()` fonksiyonu, sürekli olarak Nextion'dan gelen komutları işler (`handleNextionInput`), CAN mesajlarını dinler (`twai_receive`, `handleDetection`), hedef kaybolduğunda ekranı temizler (`clearDetection`) ve buzzer'ı yönetir (`handleBuzzer`).
-   **HABERLEŞME (Nextion -> ESP32):**
    -   `sendCommand(String cmd)`: Nextion ekrana komut göndermek için kullanılır.
    -   `handleNextionInput()`: Nextion'dan gelen verileri okur, `strstr` ile komutları ayrıştırır ve `SAVE1`, `SAVE2`, `SAVE3`, `RESETALL` gibi ayar komutlarını işler.
-   **RADAR GÖRSELLEŞTİRME MOTORU:**
    -   `handleDetection(const twai_message_t& msg)`: Gelen CAN mesajını işler, polar ve kartezyen koordinatları hesaplar, otomatik zoom mantığını uygular, buzzer davranışını belirler ve Nextion ekranını günceller.
    -   `updateVehicleDisplay(float currentMaxGridXMeters)`: Araç görselini ve genişliğini ekranda günceller.
    -   `clearDetection()`: Hedef kaybolduğunda ekranı temizler ve varsayılan duruma getirir.
    -   `updateTargetDisplay(int x, int y, int color)`: Algılanan hedefin konumunu ve rengini ekranda günceller.
    -   `updateTextDisplays(float radius, int angle, float x_m, y_m)`: Mesafe, açı, X ve Y koordinatları gibi metin bilgilerini ekranda günceller.
    -   `handleBuzzer()`: Buzzer'ın sesli alarm mantığını yönetir (sürekli ton, aralıklı bip sesleri).
-   **EEPROM:**
    -   `loadSettingsFromEEPROM()`: EEPROM'dan kaydedilmiş ayarları yükler veya geçerli ayar bulunamazsa varsayılanları yükler.
    -   `saveSettingsToEEPROM()`: Mevcut ayarları EEPROM'a kaydeder.
    -   `resetToDefaults()`: Tüm ayarları fabrika varsayılan değerlerine döndürür ve EEPROM'a kaydeder.
    -   `sendSettingsToNextion()`: Mevcut ayarları Nextion ekrana göndererek arayüzdeki değerleri günceller.

---
