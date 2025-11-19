# ESP32 & Nextion - Gelişmiş CAN BUS Radar Sistemi

Bu proje, bir ESP32 mikrodenetleyici kullanarak bir CAN BUS radarından gelen verileri işleyen ve sonuçları bir Nextion HMI ekranda görselleştiren gelişmiş bir radar sistemidir. Sistem, hedefleri mesafelerine ve konumlarına göre sınıflandırır, görsel ve sesli uyarılar üretir ve kullanıcı tarafından yapılandırılabilir ayarlara sahiptir.

**Versiyon:** 3.5.0

---

## 🌟 Temel Özellikler

- **CAN BUS Entegrasyonu:** Radar sensöründen gelen verileri `TWAI` (Two-Wire Automotive Interface) sürücüsü aracılığıyla alır ve işler.
- **Nextion HMI Arayüzü:** Algılanan hedefleri, tehlike bölgelerini ve araç konumunu dinamik olarak bir Nextion ekranda gösterir. Ayarlar için dokunmatik bir arayüz sunar.
- **Dinamik Görselleştirme:** "Auto Zoom" özelliği sayesinde, hedefin mesafesine göre ekranın yanal görüş mesafesini otomatik olarak ayarlar (L1-L4 kademeleri).
- **Kademeli Sesli Alarm:** Hedefin yakınlığına göre farklı aralıklarla (sarı, turuncu, kırmızı bölge) veya sürekli (çok yakın) ses çıkaran bir buzzer ile sesli uyarı sağlar.
- **Kalıcı Ayarlar (EEPROM):** Uyarı/tehlike bölgeleri, araç genişliği, yan boşluklar ve kullanıcı şifresi gibi ayarlar EEPROM'a kaydedilerek güç kesintilerinde bile korunur.
- **Gelişmiş Hata Ayıklama:** Kod, `CAN`, `Nextion`, `Radar`, `Buzzer` ve `EEPROM` modülleri için ayrı ayrı etkinleştirilebilen bir hata ayıklama sistemine sahiptir. Bu, sorun gidermeyi kolaylaştırır.
- **Şifre Koruması:** Ayarlar menüsüne erişim şifre ile korunmaktadır.

---

## 🛠️ Donanım ve Yazılım Gereksinimleri

### Donanım
- **Mikrodenetleyici:** ESP32 Geliştirme Kartı (`esp32dev`)
- **Ekran:** Nextion HMI Dokunmatik Ekran
- **Radar Sensörü:** CAN BUS (TWAI) arayüzüne sahip bir radar modülü.
- **CAN Alıcı-Verici:** ESP32 ile CAN BUS arasına bağlamak için bir CAN alıcı-verici modülü (örn: TJA1050, SN65HVD230).
- **Buzzer:** Sesli uyarılar için aktif veya pasif bir buzzer.
- **Bağlantı Kabloları**

### Yazılım
- **Geliştirme Ortamı:** [PlatformIO IDE](https://platformio.org/)
- **Framework:** [Arduino](https://www.arduino.cc/)

---

## ⚙️ Proje Yapılandırması ve Kurulum

1.  **PlatformIO Projesi:** Bu proje bir PlatformIO projesidir. PlatformIO CLI veya VS Code eklentisini kullanarak projeyi açın.
2.  **Kütüphaneler:** Gerekli tüm kütüphaneler (`driver/gpio`, `driver/twai`, `EEPROM`, vb.) ESP32 için Arduino çekirdeği ile birlikte standart olarak gelir. Ek bir kütüphane kurulumu gerekmez.
3.  **Pin Bağlantıları:** `src/main.cpp` dosyasında tanımlanan pin bağlantılarını kendi donanımınıza göre yapın:
    - **CAN BUS:**
        - `CAN_TX_PIN`: `GPIO_NUM_5`
        - `CAN_RX_PIN`: `GPIO_NUM_4`
    - **Nextion Ekran (Serial2):**
        - `TX`: `GPIO_NUM_17`
        - `RX`: `GPIO_NUM_16`
    - **Buzzer:**
        - `BUZZER_PIN`: `25`
4.  **Derleme ve Yükleme:** PlatformIO arayüzünü kullanarak projeyi derleyin (`Build`) ve ESP32 kartına yükleyin (`Upload`).

---

## 🚀 Kullanım

- **İlk Başlatma:** Cihaz ilk kez başlatıldığında, EEPROM'da geçerli bir ayar bulamazsa varsayılan ayarları yükler.
- **Radar Ekranı:** Ana ekran, algılanan hedefleri aracınıza göre konumlandırır. Hedefin rengi tehlike seviyesini belirtir (Yeşil -> Sarı -> Turuncu -> Kırmızı).
- **Ayarlar Menüsü:** Ekranda ayarlar menüsüne girmek için ilgili butona dokunun. Varsayılan şifre: `1234`.
- **Ayarlar:**
    - **Sayfa 1 (Bölgeler):** Uyarı ve Tehlike bölgelerinin mesafesini ayarlayın.
    - **Sayfa 2 (Araç):** Aracın yan boşluklarını, gerçek genişliğini ve maksimum yanal tarama genişliğini ayarlayın.
    - **Sayfa 3 (Seçenekler):** Otomatik Zoom ve Sesli Alarm özelliklerini açıp kapatın.
    - Şifrenizi değiştirebilir ve tüm ayarları varsayılana sıfırlayabilirsiniz.

---

## 👨‍💻 Kod Yapısı

Kod, daha iyi okunabilirlik ve yönetim için bölümlere ayrılmıştır:

- **BÖLÜM 1: Kütüphaneler:** Gerekli kütüphaneler dahil edilir.
- **BÖLÜM 2: Ayarlar ve Sabitler:** Pin tanımlamaları, hata ayıklama anahtarları, EEPROM adresleri ve varsayılan değerler burada bulunur.
- **BÖLÜM 3: Global Nesneler ve Deklarasyonlar:** Global değişkenler ve fonksiyon prototipleri tanımlanır.
- **BÖLÜM 4: Ana Program (setup ve loop):** `setup()` fonksiyonu donanımı ve servisleri başlatır. `loop()` fonksiyonu sürekli olarak Nextion ve CAN verilerini dinler.
- **BÖLÜM 5: Haberleşme Fonksiyonları:** Nextion ekranından gelen komutları işler ve ekrana komut gönderir.
- **BÖLÜM 6: Radar Mantığı ve Görselleştirme:** CAN verisini işler, tehlike seviyesini belirler, piksel hesaplamalarını yapar ve ekranı günceller.
- **BÖLÜM 7: EEPROM Yönetimi:** Ayarları kalıcı hafızaya kaydeder, okur ve varsayılanlara sıfırlar.
