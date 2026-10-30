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
[![Tests](https://img.shields.io/badge/Tests-2%20passed-brightgreen.svg)](tests/)

Bu proje; Yıldız Teknik Üniversitesi Elektrik-Elektronik Fakültesi Çok Disiplinli Tasarım Projesi (ÇDTP) kapsamında geliştirilen, **ters sarkaç (Inverted Pendulum) prensibiyle iki tekerleği üzerinde aktif dengelenen, giyilebilir eldiven arayüzüyle yönlendirilen ve ev içinde standart bir ilaç kutusunu otonom olarak taşıyıp şarj istasyonuna yanaşan mobil sağlık asistanı robotudur**.

> **English Overview:** Autonomous Two-Wheeled Self-Balancing Indoor Patient Support and Medicine Carrier Robot developed as a senior multidisciplinary design project at Yildiz Technical University. It features an active inverted pendulum stabilization architecture, wearable gesture-control glove teleoperation via low-latency ESP-NOW, an autonomous 38 kHz infrared homing dock system, physical payload security (standard blister medicine box), and a digital twin validated in Webots R2025a with cascade dual-loop PID, base-omega feedforward, and clamping anti-windup.

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

- [config.py](file:///c:/YTU/self-balancing-robot/self_balancing_robot/config.py): Robot fiziksel parametreleri, kazançlar ve emniyet sınırları.
- [sensor_fusion.py](file:///c:/YTU/self-balancing-robot/self_balancing_robot/sensor_fusion.py): Gürültü enjeksiyonu ve %98 Gyro + %2 İvmeölçer Tamamlayıcı Filtresi.
- [cascade_controller.py](file:///c:/YTU/self-balancing-robot/self_balancing_robot/cascade_controller.py): Kaskad PID, Base-Omega İleri Beslemesi, Clamping Anti-Windup ve Motor Deadband Telafisi.
- [telemetry.py](file:///c:/YTU/self-balancing-robot/self_balancing_robot/telemetry.py): CSV telemetri kayıt yöneticisi.
- [balance_controller.py](file:///c:/YTU/self-balancing-robot/webots/controllers/balance_controller/balance_controller.py): Webots API'sini bağlayan ince adaptör (shim).

### Temel Kontrol İlkeleri

1. **İç Döngü (100 Hz):** Gövde eğim hatasından denge torku üretir (`KP_PITCH = 85.0`, `KD_PITCH = 2.40`).
2. **Dış Döngü (20 Hz):** Sürüş komutu yokken robot durduğu koordinata çıpa atar (`KP_POS = 0.15`, `KI_POS = 0.02`).
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

Simülasyon dünyası ve kontrolcü performansı iki bütünleşik test paketiyle doğrulanır:

```bash
# 1. Statik Denge, Dış Dürtme ve Yük Güvenliği Testi (Modül A & E)
python tests/test_simulation_balance.py

# 2. Sürüş, Dinamik Frenleme ve 4 mm Zemin Engeli Aşma Testi (Modül B & E)
python tests/test_simulation_driving.py
```

### Test Doğrulama Metrikleri

| Test | Koşul | Başarı Kriteri | Sonuç |
| :--- | :--- | :--- | :--- |
| **Modül A (Statik Denge)** | 5 saniye bekleme | Ortalama eğim < 0.20°, sıfır kaçış | **BAŞARILI** (0.031°) |
| **Bozucu Dürtme** | +4.0 N ve -4.0 N itme | < 1.5 saniyede dik dengeye dönüş | **BAŞARILI** (1.75°) |
| **Modül B (Sürüş)** | W / S / Space sürüş | Sürüş hızı > 0.05 m/s, dinamik fren < 0.20 m/s | **BAŞARILI** |
| **Zemin Engeli** | 4 mm zemin kablo kanalı | Devrilmeden tırmanıp aşma (eğim < 25°) | **BAŞARILI** (1.77°) |
| **Modül E (İlaç Taşıma)** | Dinamik sürüş & dürtme | Aspirin kutusunun düşmeden taşınması | **BAŞARILI** |

---

## 9. Hızlı Başlangıç ve Kullanım

### Webots Simülasyonunu Açma

```bash
webots webots/worlds/cdtp_balance_world.wbt
```

### Klavye ile Teleoperasyon Kontrolleri

Simülasyon penceresine tıkladıktan sonra aşağıdaki tuşlarla robot kontrol edilir:
- **W / Yukarı Ok:** İleri sürüş (+0.30 m/s)
- **S / Aşağı Ok:** Geri sürüş (-0.30 m/s)
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

- **Proje Teknik Hafızası & Karar Günlüğü:** [AGENTS.md](file:///c:/YTU/self-balancing-robot/AGENTS.md)
- **Sürüm Değişiklik Geçmişi:** [CHANGELOG.md](file:///c:/YTU/self-balancing-robot/CHANGELOG.md)
- **Resmi Şartname Belgesi:** [YTU_EEF_CDTP_Ev_Ici_Ilac_Tasima_Denge_Robotu_TR_v14.pdf](file:///c:/YTU/self-balancing-robot/docs/specs/YTU_EEF_CDTP_Ev_Ici_Ilac_Tasima_Denge_Robotu_TR_v14.pdf)
- **Lazer Kesim Şablonu:** [pleksi_katmanlari.dxf](file:///c:/YTU/self-balancing-robot/docs/cad/pleksi_katmanlari.dxf) ve [pleksi_kesim_sablonu.svg](file:///c:/YTU/self-balancing-robot/docs/cad/pleksi_kesim_sablonu.svg)

---

## 12. Lisans ve Telif Hakkı

Bu proje [MIT Lisansı](LICENSE) kapsamında lisanslanmıştır. Yıldız Teknik Üniversitesi Elektrik-Elektronik Fakültesi ÇDTP bitirme projesi kapsamında geliştirilmiştir.
