# İki Tekerlekli Ev İçi Hasta Destek ve İlaç Taşıma Denge Robotu

## Yıldız Teknik Üniversitesi — Elektrik-Elektronik Fakültesi

Çok Disiplinli Tasarım Projesi (ÇDTP)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python 3.10-3.12](https://img.shields.io/badge/Python-3.10--3.12-blue.svg)](https://www.python.org/)
[![Hardware](https://img.shields.io/badge/Hardware-ESP32%20DevKit%20V1-002B49.svg)](https://www.espressif.com/)
[![Simulation](https://img.shields.io/badge/Simulation-Webots%20R2025a-red.svg)](https://cyberbotics.com/)
[![Sensors](https://img.shields.io/badge/Sensors-MPU6050%206--DoF-00AEEF.svg)](https://invensense.tdk.com/)
[![Control](https://img.shields.io/badge/Control-Cascade%20Dual--Loop%20PID-orange.svg)](webots/controllers/balance_controller/balance_controller.py)
[![Comms](https://img.shields.io/badge/Comms-ESP--NOW%202.4GHz-brightgreen.svg)](https://www.espressif.com/en/solutions/low-power-solutions/esp-now)
[![Simulation Tests](https://img.shields.io/badge/Simulation%20Tests-3%20passed-brightgreen.svg)](tests/)

Bu proje; Yıldız Teknik Üniversitesi Elektrik-Elektronik Fakültesi Çok Disiplinli Tasarım Projesi (ÇDTP) kapsamında geliştirilen, **ters sarkaç (Inverted Pendulum) prensibiyle iki tekerleği üzerinde aktif dengelenen, giyilebilir eldiven arayüzüyle yönlendirilen ve ev içinde standart bir ilaç kutusunu otonom olarak taşıyıp şarj istasyonuna yanaşan mobil sağlık asistanı robotudur**.

> **English Overview:** Autonomous Two-Wheeled Self-Balancing Indoor Patient Support and Medicine Carrier Robot developed as a senior multidisciplinary design project at Yildiz Technical University. It features an active inverted pendulum stabilization architecture, wearable gesture-control glove teleoperation via low-latency ESP-NOW, an autonomous 38 kHz infrared homing dock system, physical payload security (standard blister medicine box), and a digital twin validated in Webots R2025a with cascade dual-loop PID, base-omega feedforward, and clamping anti-windup.

---

## Proje ve Şartname Geliştirme Durumu

| Şartname Modülü | Kapsam | Durum | Doğrulama Ortamı |
| :--- | :--- | :--- | :--- |
| **Modül A (Statik Denge)** | Dış destek olmadan 30 saniye dik denge ve harici 4.0 N darbe sönümleme | **DOĞRULANDI** | Webots R2025a (`tests/test_simulation_balance.py`) |
| **Modül B (Sürüş & Duruş)** | 0.8 m – 1.2 m mesafe katetme, 3 sn hareketsiz duruş ve geri sürüş | **DOĞRULANDI** | Webots R2025a (`tests/test_simulation_driving.py`) |
| **Modül B (Giyilebilir Eldiven)** | ESP32 + MPU-6050 ile ESP-NOW üzerinden kablosuz el jesti kontrolü | **GELİŞTİRİLİYOR** | `esp32/glove/` FreeRTOS firmware |
| **Modül C (Otonom IR Şarj)** | 38 kHz diferansiyel IR nirengisiyle istasyona yaklaşma ve kilitlenme | **TASARIM AŞAMASINDA** | 2x TSOP38438 IR gözleri + Katman 1 temas pedleri |
| **Modül D (Şarj Göstergesi)** | Fiziksel temasla şarj başlangıcı/ayrılış durum LED'i | **DONANIM ŞEMASI HAZIR** | Katman 4 durum LED devresi |
| **Modül E (İlaç Kutusu Taşıma)** | Standart kapalı karton ilaç (Aspirin) kutusunun düşmeden taşınması | **DOĞRULANDI** | Katman 4 dinamik hazne + yerel eksen denetimi |
| **DC Motor Tork/PWM Modeli** | TB6612 PWM, JGB37 Ters-EMK, armatür direnci ve patinaj sınırlandırması | **DOĞRULANDI** | Webots R2025a (`tests/test_simulation_voltage.py`) |
| **Modül 5.2 (Mobil Doğrulama)** | Telefon kamerasıyla süre, eğim ve konum ölçüm arayüzü | **PLANLAMA AŞAMASINDA** | `app/` mobil değerlendirme uygulaması |


---

## İçindekiler

1. [Sistem Ne Yapar?](#1-sistem-ne-yapar)
2. [Sistem Bir Bakışta](#2-sistem-bir-bakışta)
3. [Güncel Sistem Mimarisi](#3-güncel-sistem-mimarisi)
4. [Mekanik ve Şasi Mimarisi](#4-mekanik-ve-şasi-mimarisi)
5. [Donanım ve Elektriksel Mimari](#5-donanım-ve-elektriksel-mimari)
6. [Yazılım ve Kontrol Teorisi](#6-yazılım-ve-kontrol-teorisi)
7. [Fiziksel Güvenlik ve Emniyet Kalkanları](#7-fiziksel-güvenlik-ve-emniyet-kalkanları)
8. [Doğrulama ve Simülasyon Testleri](#8-doğrulama-ve-simülasyon-testleri)
9. [Hızlı Başlangıç ve Kullanım](#9-hızlı-başlangıç-ve-kullanım)
10. [Dizin Yapısı](#10-dizin-yapısı)
11. [Dokümantasyon ve Proje Hafızası](#11-dokümantasyon-ve-proje-hafızası)
12. [Lisans ve Telif Hakkı](#12-lisans-ve-telif-hakkı)

---

## 1. Sistem Ne Yapar?

Sistem, ev veya hastane ortamında yatağa bağımlı bir hastanın ihtiyacı olan ilacı tek bir hareketle güvenle ulaştırır:

1. **Dengeler (Modül A):** Dış destek almadan dik konumda aktif dengesini sağlar; harici 4.0 N itme darbelerinde devrilmeden 1.5 saniyede dik konuma geri döner.
2. **Yönlendirilir (Modül B):** Hastanın veya refakatçinin giydiği kablosuz kontrol eldivenindeki eğim komutlarıyla (İleri, Geri, Sola Dön, Sağa Dön, Dur) 1 metre menzilde sarsıntısız seyreder.
3. **Taşır (Modül E):** Şartname Madde 5.3'e uygun standart kapalı bir karton ilaç (Aspirin / Parol) kutusunu Katman 4 üzerindeki düşme önleyici kaset yuvasında güvenle taşır.
4. **Homing ve Docking (Modül C):** Görev bitiminde 30 cm yarıçaplı yaklaşma bölgesinden 38 kHz diferansiyel IR nirengisiyle şarj istasyonunu bulur, hizalanır ve yanaşır.
5. **Otomatik Şarj (Modül D):** İnsan müdahalesi olmadan 5 saniye içinde fiziksel temas pedleriyle şarjı otomatik başlatır; durum LED'i ile doğrular.
6. **İzlenir (Modül 5.2):** Akıllı telefon kamerasıyla görev süresi, eğim açısı ve kat edilen mesafe harici arayüz üzerinden doğrulanır.

---

## 2. Sistem Bir Bakışta

| Alt Sistem | Uygulama | Görevi |
| :--- | :--- | :--- |
| **Gövde & Şasi** | 135 × 95 mm Altın Oran, 4 katmanlı 5 mm Beyaz Dekota, 4x M3 tij | Darbe emici, rijit kutu kafes ve dahili dikey kablo bacası |
| **Sensör Füzyonu** | GY-521 MPU-6050 (X=0, Y=0 merkezinde), 100 Hz Tamamlayıcı Filtre | %98 Jiroskop integrali + %2 İvmeölçer ile kaymasız eğim kestirimi |
| **Kontrol Mimarisi** | Çift Döngülü Kaskad PID, Base-Omega İleri Besleme, Clamping Anti-Windup | Düşmeyi engelleyen iç döngü (100 Hz) ve konum çıpalayan dış döngü |
| **Tahrik Sistemi** | 2x JGB37-520 12V DC Redüktörlü Motor, Manyetik Enkoder, 85 mm Tekerlek | Yüksek torklu diferansiyel sürüş ve milimetrik odometri |
| **Sürücü & Güç** | TB6612FNG MOSFET (Katman 1'de 3 cm hatla EMI izolasyonu) + XL4015 5A | 12V motor ve 5V lojik besleme hatları |
| **Batarya Paketi** | 3S1P 18650 Li-ion (11.1V, 2000 mAh) + 3S 10A BMS + 3A Cam Sigorta | 2 saat kesintisiz çalışma ve aşırı akım emniyeti |
| **Kablosuz İletişim** | ESP-NOW 2.4 GHz Doğrudan MAC Katmanı (1-2 ms gecikme) | Router/Wi-Fi gerektirmeyen güvenilir eldiven-robot linki |
| **Giyilebilir Eldiven** | ESP32 + MPU-6050 sensörlü esnek eldiven | Operatör el eğimini anlık 5 sürüş komutuna dönüştürür |
| **Dijital İkiz** | Webots R2025a, ayrık dinamik ilaç kutusu, zemin kablo kanalı ve oda | Fiziksel donanıma zarar vermeden simülasyonda doğrulama |

---

## 3. Güncel Sistem Mimarisi

Sistem dört ana katmandan oluşur. Üst katmanlar stratejik referans üretir, alt katmanlar bu referansı refleks düzeyinde eyleme dönüştürür:

```text
[Giyilebilir Eldiven / ESP-NOW] ──► Hedef Hız (v_cmd) & Dönüş
                                           │
                                           ▼
[Dış Döngü: Konum & Hız PI-D (Anti-Windup)] ──► Hedef Eğim Açısı (pitch_target)
                                                               │
[MPU-6050 Sensör Füzyonu (%98 Gyro + %2 Accel)] ─────────────►│
                                                               ▼
                                              [Hata: pitch_est - pitch_target]
                                                               │
                                                               ▼
                                              [İç Döngü: Denge PID (100 Hz)] ──► u_balance
                                                                                       │
[Taban Hız İleri Beslemesi: base_omega = -v / R] ──────────────────────────────────────┼──► u_unsat
                                                                                       │
[Diferansiyel Dönüş Komutu: +/- turn_speed] ───────────────────────────────────────────┘
                                           │
                                           ▼
[Motor Telafisi: Stiction Deadband (+0.50 rad/s) & Eşik Filtresi]
                                           │
                                           ▼
[TB6612FNG Sürücü ➔ 2x JGB37 DC Redüktörlü Motor]
```

### Katmanların Sorumlulukları

1. **Giyilebilir Arayüz (Eldiven):** Operatörün el hareketlerini ölçer, filtreler ve ESP-NOW üzerinden 5 komuttan birini 20 Hz hızında yayınlar.
2. **Dış Döngü (Navigasyon & Konum):** Hedef hız komutundan gövde yatış açısı referansı üretir; durulduğunda anında koordinat çıpası atarak konumu kilitler.
3. **İç Döngü (Denge Refleksi):** 100 Hz hızında gövde açısını ve açısal hızını denetleyerek robotun yerçekimine yenilmesini engeller.
4. **Fiziksel Eyleme (Motor & Şasi):** Taban hız ileri beslemesi ve statik sürtünme telafisi uygulanarak motorlara tork komutu iletilir.

---

## 4. Mekanik ve Şasi Mimarisi

Şasi ölçüleri, hammadde verimliliği ve ağırlık merkezi dengesi dikkate alınarak **135 mm Genişlik × 95 mm Derinlik (Altın Oran)** standardında tasarlanmıştır:

- **Doğal Tampon Koruma:** 85 mm çapındaki tekerlekler, 95 mm şasi derinliğinin içinde kalır; şasi tekerleklerin önünden ve arkasından 5'er mm taşarak engellere karşı doğal tampon görevi görür.
- **Motor İçi Kablo Koridoru:** 135 mm plaka genişliği sayesinde tabandaki iki motor arasında 35 mm açık kablo geçiş koridoru kalır.
- **Dikey Kablo Bacası (Conduit Riser):** Tüm katmanlarda birebir aynı koordinatta (`X=67.5 mm`, `Y=22 mm`) açılan 32 × 12 mm şaft sayesinde hiçbir kablo dışarı sarkmaz; tekerleklere dolanma riski sıfırdır.
- **Kesintisiz M3 Tij (Gijon) Kafesi:** Titreşimle gevşeyen pirinç distanslar yerine, tabandan tavana uzanan 4 adet paslanmaz M3 tij, pul ve somunlarla sıkıştırılarak monolitik kutu kafes (box-truss) yapısı elde edilmiştir.

### 4 Katmanın İşlev Dağılımı

1. **Katman 1 (Taban / Base Deck):** JGB37 motor L-braketleri, TB6612FNG motor sürücüsü, şarj temas pedleri ve kablo bacası.
2. **Katman 2 (Güç / Power Deck):** 3S 18650 Li-ion batarya paketi (Ağırlık merkezi CoM = 10 cm hizası), XL4015 5A regülatör ve kablo bacası.
3. **Katman 3 (Beyin / Brain Deck):** ESP32 DevKit V1, MPU-6050 (tam merkez X=0, Y=0), 2x TSOP38438 IR gözleri ve kablo bacası.
4. **Katman 4 (Tavan / Top Deck):** 80 × 26 mm Aspirin kaset yuvası, KCD11 rocker switch, 3A cam sigorta yuvası ve durum LED'i.

---

## 5. Donanım ve Elektriksel Mimari

### Güç Dağıtım Hattı

- **12V Ham Batarya Hattı:** Batarya (+) ➔ Katman 4 Sigorta (3A) ➔ KCD11 Switch ➔ Katman 1 TB6612 `VM` pini & Katman 2 XL4015 `IN(+)`.
- **5V Lojik Hattı:** XL4015 5.0V çıkışı ➔ Katman 3 ESP32 `VIN` pini, TB6612 `VCC` pini ve Manyetik Enkoder beslemeleri.
- **3.3V Sensör Hattı:** ESP32 dahili 3.3V LDO ➔ MPU-6050 `VCC` ve TSOP38438 `VCC`.

### EMI İzolasyon Kararı

Motor sürücü (TB6612FNG) Katman 1 tabanına yerleştirilmiştir. Bu sayede motorlara giden yüksek akımlı (12V, 2-3A) 20 kHz PWM hatları sadece **3 cm** boyunda kalmış; ESP32 ve MPU-6050 üzerindeki elektromanyetik parazit (EMI) sıfırlanmıştır.

---

## 6. Yazılım ve Kontrol Mimarisi

Sistem mimarisi, Webots simülasyonunu kirletmemek ve gerçek zamanlı algoritmaları bağımsız tutmak için `self_balancing_robot` çekirdek paketi üzerine kurulmuştur:

- [config.py](self_balancing_robot/config.py): Robot fiziksel parametreleri, kazançlar ve emniyet sınırları.
- [sensor_fusion.py](self_balancing_robot/sensor_fusion.py): Gürültü enjeksiyonu ve %98 Gyro + %2 İvmeölçer Tamamlayıcı Filtresi.
- [cascade_controller.py](self_balancing_robot/cascade_controller.py): Kaskad PID, Base-Omega İleri Beslemesi, Clamping Anti-Windup ve Motor Deadband Telafisi.
- [dc_motor_model.py](self_balancing_robot/dc_motor_model.py): JGB37 DC Motor armatür direnci, Ters-EMK, tork katsayısı ve patinaj sınırlaması.
- [telemetry.py](self_balancing_robot/telemetry.py): CSV telemetri kayıt yöneticisi.
- [balance_controller.py](webots/controllers/balance_controller/balance_controller.py): Webots hız sürümlü referans kontrolcü adaptörü.
- [voltage_controller.py](webots/controllers/voltage_controller/voltage_controller.py): Webots ESP32 PWM / Gerilim (Tork) tabanlı fiziksel kontrolcü adaptörü.

### Temel Kontrol İlkeleri

1. **İç Döngü (100 Hz):** Gövde eğim hatasından denge torku üretir (`KP_PITCH = 140.0`, `KD_PITCH = 3.60`).
2. **Dış Döngü (100 Hz):** Sürüş komutu yokken robot durduğu koordinata çıpa atar (`KP_POS = 0.20`, `KI_POS = 0.015`, `MAX_POS_INT = 0.030 rad`). Sürüş anında hız hatasından hedef eğim açısı üretir (`KP_VEL = 0.045`, `MAX_PITCH_TARGET = 0.025 rad` ~ 1.43°).
3. **Clamping Anti-Windup:** İntegral doyumu sınırlandırılmıştır (`MAX_POS_INT = 0.030 rad` ~ 1.7°); motor doyuma ulaştığında integral hesabı dondurulur.
4. **Base-Omega Feedforward:** Gaz bırakıldığında tekerleklerin motor-freni yaparak gövdeyi öne fırlatmasını engellemek için anlık dönme hızı (`-v / R`) ileri besleme olarak eklenir.

---

## 7. Fiziksel Güvenlik ve Emniyet Kalkanları

1. **Devrilme Emniyet Kesicisi (Fall Protection):** Gövde açısı +/- 45 dereceyi aştığında motor PWM sinyalleri derhal sıfırlanır; motorların ve dişlilerin yanması engellenir.
2. **Donanımsal 3A Cam Sigorta:** Motor sıkışması veya kısa devre anında bataryayı korumak için 3A hızlı atan cam sigorta hattı keser.
3. **Dinamik İlaç Kutusu Kaseti:** 80 × 26 mm ölçülerinde 4 kenarlı dekota bariyer sayesinde ani fren veya dürtmelerde ilaç kutusu hazneden fırlamaz.
4. **Stiction Deadband Filtresi:** 0.60 rad/s altındaki mikro titreşim komutları motor ölü bölgesinde süzülerek duruş anında rezonans titremesi önlenir.

---

## 8. Doğrulama ve Simülasyon Testleri

Simülasyon dünyası ve kontrolcü performansı, şartname isterlerinin tamamını kapsayan **2 adet bütünleşik ve sadeleştirilmiş test paketiyle** doğrulanır:

```bash
# 1. Modül A (30s Denge & 4.0N Bozucu) ve Modül E (İlaç Güvenliği) Testi
python tests/test_simulation_balance.py

# 2. Modül B (1m Mesafe, 3s Duruş, Geri Sürüş) ve Modül E Testi
python tests/test_simulation_driving.py

# 3. Gerçekçi DC Motor Modeli ve TB6612 Gerilim/PWM (Tork) Testi
python tests/test_simulation_voltage.py
```

### Test Doğrulama Metrikleri

| Test Paketi | Kapsanan Şartname İsterleri | Başarı Kriteri | Ölçülen Sonuç | Durum |
| :--- | :--- | :--- | :--- | :--- |
| **Modül A & E (Denge & Bozucu)** | • Kesintisiz 30 saniye statik denge<br>• +4.0 N ve -4.0 N itme darbeleri<br>• Aspirin kutusu yerel eksen takibi | • time >= 29.8s, step >= 2990<br>• Statik eğim < 0.20°<br>• Darbe sonrası toparlanma < 2.0°<br>• Kutu hazneden düşmez | • Ortalama statik eğim: **0.070°**<br>• Darbe anı azami eğim: **6.98°**<br>• 1. Toparlanma: **0.60°**, 2.: **0.71°**<br>• Süre: **30.00 saniye** | **BAŞARILI** |
| **Modül B & E (Sürüş & Duruş)** | • 0.8 m – 1.2 m mesafe katetme<br>• Hedefte 3 sn hareketsiz duruş<br>• Geri sürüş (negatif hız)<br>• Manevralarda ilaç kutusu güvenliği | • time >= 17.3s, step >= 1730<br>• Net mesafe: 0.80 - 1.20 m<br>• Duruş fazı hızı < 0.02 m/s<br>• Geri sürüş hızı < -0.05 m/s | • Net mesafe: **+1.025 m**<br>• 3 sn duruş ort. hız: **0.0090 m/s**<br>• Geri sürüş ort. hız: **-0.086 m/s**<br>• Süre: **17.50 saniye** | **BAŞARILI** |
| **DC Motor PWM & Tork Testi** | • TB6612 gerilim/PWM komutları<br>• JGB37 Ters-EMK ve sürtünme telafisi<br>• 30s denge ve 1.5N darbe toparlanma | • time >= 29.8s, step >= 2990<br>• Statik ortalama eğim < 0.35°<br>• Patinajsız toparlanma<br>• Kutu hazneden düşmez | • Ortalama statik eğim: **0.117°**<br>• Darbe sonrası toparlanma: **-0.01°**<br>• Patinaj/kayma: **Önlendi**<br>• Süre: **30.00 saniye** | **BAŞARILI** |

---

## 9. Hızlı Başlangıç ve Kullanım

### Gereksinimler ve Kurulum

1. **Python 3.10 - 3.12:**
   ```bash
   pip install -r requirements.txt
   ```
2. **Webots R2025a:**
   Simülasyon ortamı için [Cyberbotics Webots R2025a](https://cyberbotics.com/) kurulu olmalıdır.
   Standart dışı bir konuma kurulduysa `WEBOTS_EXECUTABLE` ortam değişkeni tanımlanabilir:
   ```bash
   # Windows PowerShell
   $env:WEBOTS_EXECUTABLE = "C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
   ```

### Webots Simülasyonunu Açma

```bash
webots webots/worlds/cdtp_balance_world.wbt
```

### Klavye ile Teleoperasyon Kontrolleri

Simülasyon penceresine tıkladıktan sonra aşağıdaki tuşlarla robot kontrol edilir:
- **W / Yukarı Ok:** İleri sürüş (güvenli nominal hız)
- **S / Aşağı Ok:** Geri sürüş (güvenli nominal hız)
- **A / Sol Ok:** Sola yumuşak dönüş (~34°/s)
- **D / Sağ Ok:** Sağa yumuşak dönüş (~34°/s)
- **P Tuşu:** Robota 4.0 N harici bozucu itme kuvveti uygula
- **Boşluk (Space):** Dinamik fren yap ve durduğun noktaya konum kilidi at
- **R Tuşu:** Robotu ve ilaç kutusunu başlangıç konumuna ve dik dengeye sıfırla

---

## 10. Dizin Yapısı

```text
c:\YTU\self-balancing-robot\
├── docs/                      # Dokümantasyon ve şartnameler
│   ├── specs/                 # Şartname, akademik takvim ve tanıtım PDF'leri
│   ├── reports/               # FR-1558 yönetim planı ve proje raporları
│   ├── cad/                   # Lazer kesim DXF ve teknik SVG şablonları
│   └── assets/                # Mimari şemalar, renderlar ve flowchartlar
├── esp32/                     # Gömülü yazılımlar (ESP32)
│   ├── robot/                 # Denge robotu FreeRTOS/PID firmware'i
│   └── glove/                 # Kontrol eldiveni ESP-NOW firmware'i
├── app/                       # Şartname 5.2 Mobil / Kamera değerlendirme uygulaması
├── webots/                    # Webots R2025a simülasyon ortamı
│   ├── controllers/           # Modüler kontrolcü mimarisi (config, fusion, cascade, telemetry)
│   └── worlds/                # Simülasyon dünyası (12x12m oda, zemin kanalı, aspirin kutusu)
├── analysis/                  # Telemetri, kontrol teorisi ve analiz grafikleri
├── tests/                     # Bütünleşik simülasyon doğrulama testleri
├── logs/                      # Telemetri ve test günlükleri (CSV)
├── .editorconfig              # Kod biçimlendirme standartları
├── .gitattributes             # Git satır sonu kuralları
├── .gitignore                 # Versiyon kontrol kuralları
├── CHANGELOG.md               # Sürüm ve değişiklik geçmişi
├── AGENTS.md                  # Teknik hafıza, mimari kararlar ve proje arşivi
└── README.md                  # Proje ana dokümantasyonu
```

---

## 11. Dokümantasyon ve Proje Hafızası

- **Proje Teknik Hafızası & Karar Günlüğü:** [AGENTS.md](AGENTS.md)
- **Sürüm Değişiklik Geçmişi:** [CHANGELOG.md](CHANGELOG.md)
- **Resmi Şartname Belgesi:** [YTU_EEF_CDTP_Ev_Ici_Ilac_Tasima_Denge_Robotu_TR_v14.pdf](docs/specs/YTU_EEF_CDTP_Ev_Ici_Ilac_Tasima_Denge_Robotu_TR_v14.pdf)
- **Lazer Kesim Şablonu:** [pleksi_katmanlari.dxf](docs/cad/pleksi_katmanlari.dxf) ve [pleksi_kesim_sablonu.svg](docs/cad/pleksi_kesim_sablonu.svg)

---

## 12. Lisans ve Telif Hakkı

Bu proje [MIT Lisansı](LICENSE) kapsamında lisanslanmıştır. Yıldız Teknik Üniversitesi Elektrik-Elektronik Fakültesi ÇDTP bitirme projesi kapsamında geliştirilmiştir.
