# İki Tekerlekli Ev İçi Hasta Destek ve İlaç Taşıma Denge Robotu

## Yıldız Teknik Üniversitesi — Elektrik-Elektronik Fakültesi

Çok Disiplinli Tasarım Projesi (ÇDTP)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python 3.10-3.12](https://img.shields.io/badge/Python-3.10--3.12-blue.svg)](https://www.python.org/)
[![Hardware](https://img.shields.io/badge/Hardware-ESP32%20DevKit%20V1-002B49.svg)](https://www.espressif.com/)
[![Simulation](https://img.shields.io/badge/Simulation-Webots%20R2025a-red.svg)](https://cyberbotics.com/)
[![Sensors](https://img.shields.io/badge/Sensors-MPU6050%206--DoF-00AEEF.svg)](https://invensense.tdk.com/)
[![Control](https://img.shields.io/badge/Control-Cascade%20Dual--Loop%20PID-orange.svg)](self_balancing_robot/pid_controller.py)
[![Comms](https://img.shields.io/badge/Comms-ESP--NOW%202.4GHz-brightgreen.svg)](https://www.espressif.com/en/solutions/low-power-solutions/esp-now)
[![Tests](https://img.shields.io/badge/Tests-26%20passed-brightgreen.svg)](tests/)

Bu proje; Yıldız Teknik Üniversitesi Elektrik-Elektronik Fakültesi Çok Disiplinli Tasarım Projesi (ÇDTP) kapsamında geliştirilen, **ters sarkaç (Inverted Pendulum) prensibiyle iki tekerleği üzerinde aktif dengelenen, giyilebilir eldiven arayüzüyle yönlendirilen ve ev içinde standart bir ilaç kutusunu otonom olarak taşıyıp şarj istasyonuna yanaşan mobil sağlık asistanı robotudur**.

> **English Overview:** Autonomous Two-Wheeled Self-Balancing Indoor Patient Support and Medicine Carrier Robot developed as a senior multidisciplinary design project at Yildiz Technical University. It features an active inverted pendulum stabilization architecture, wearable gesture-control glove teleoperation via low-latency ESP-NOW, an autonomous 38 kHz infrared homing dock system, physical payload security (standard blister medicine box), and a digital twin validated in Webots R2025a with cascade dual-loop PID, back-EMF feedforward, and clamping anti-windup.

---

## Proje ve Şartname Geliştirme Durumu

| Şartname Modülü | Kapsam | Durum | Doğrulama Ortamı |
| :--- | :--- | :--- | :--- |
| **Modül A (Statik Denge)** | Dış destek olmadan 30 saniye dik denge ve harici 1.5 N darbe sönümleme | **DOĞRULANDI** | Webots R2025a (`tests/test_simulation_pid.py`) |
| **Modül B (Sürüş & Duruş)** | 0.8 m – 1.2 m mesafe katetme, 3 sn hareketsiz duruş ve geri sürüş | **DOĞRULANDI** | Webots R2025a (`tests/test_simulation_pid_driving.py`) |
| **Modül B (Giyilebilir Eldiven)** | ESP32 + MPU-6050 ile ESP-NOW üzerinden kablosuz el jesti kontrolü | **GELİŞTİRİLİYOR** | `esp32/glove/` FreeRTOS firmware |
| **Modül C (Otonom IR Yanaşma)** | 38 kHz IR verici feneri ve 2x TSOP38438 alıcı nirengisiyle istasyona yaklaşma | **DOĞRULANDI** | Webots R2025a (`tests/test_simulation_ir_homing.py`) |
| **Modül D (Şarj Göstergesi)** | Fiziksel temasla şarj başlangıcı/ayrılış durum LED'i | **DONANIM ŞEMASI HAZIR** | Katman 4 durum LED devresi |
| **Modül E (İlaç Kutusu Taşıma)** | Standart kapalı karton ilaç (Aspirin) kutusunun düşmeden taşınması | **DOĞRULANDI** | Katman 4 dinamik hazne + yerel eksen denetimi |
| **DC Motor Tork/PWM Modeli** | TB6612 PWM, JGB37 Ters-EMK, armatür direnci ve fiziksel temasla kayma ölçümü | **DOĞRULANDI** | Webots R2025a (`tests/test_simulation_pid.py`) |
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
| **Kontrol Mimarisi** | Çift Döngülü Kaskad PID, Ters EMK İleri Besleme, Enkoderli PI Dönüş | Düşmeyi engelleyen iç döngü (100 Hz) ve konum çıpalayan dış döngü |
| **Tahrik Sistemi** | 2x JGB37-520 12V DC Redüktörlü Motor, Manyetik Enkoder, 85 mm Tekerlek | Yüksek torklu diferansiyel sürüş ve milimetrik odometri |
| **Sürücü & Güç** | TB6612FNG MOSFET (Katman 1'de 3 cm hatla EMI izolasyonu) + XL4015 5A | 12V motor ve 5V lojik besleme hatları |
| **Batarya Paketi** | 3S1P 18650 Li-ion (11.1V, 2000 mAh) + 3S 10A BMS + 3A Cam Sigorta | 2 saat kesintisiz çalışma ve aşırı akım emniyeti |
| **Kablosuz İletişim** | ESP-NOW 2.4 GHz Doğrudan MAC Katmanı (1-2 ms gecikme) | Router/Wi-Fi gerektirmeyen güvenilir eldiven-robot linki |
| **Giyilebilir Eldiven** | ESP32 + MPU-6050 sensörlü esnek eldiven | Operatör el eğimini anlık 5 sürüş komutuna dönüştürür |
| **Dijital İkiz** | Webots R2025a, ayrık dinamik ilaç kutusu, düz zemin ve oda | Fiziksel donanıma zarar vermeden simülasyonda doğrulama |

---

## 3. Güncel Sistem Mimarisi

Sistem dört ana katmandan oluşur. Üst katmanlar stratejik referans üretir, alt katmanlar bu referansı refleks düzeyinde eyleme dönüştürür:

```text
[Klavye / planlanan ESP-NOW komutları]
                  |
[Hız hedefi / duruşta konum PI ve hız sönümleme]
                  |
             [Hedef eğim] <--- [IMU tamamlayıcı filtre]
                  |
         [Açı PD: gerilim komutu]
                  |
[Sürtünme telafisi + ters EMK ileri beslemesi]
                  |
[Enkoderli PI dönüş kontrolü + denge öncelikli gerilim sınırı]
                  |
        [PWM nicemlemesi: 8 bit]
                  |
     [DC motor modeli: gerilim -> tork]
                  |
        [Webots fizik ve zemin teması]
```

### Katmanların Sorumlulukları

1. **Giyilebilir Arayüz (Eldiven):** Operatörün el hareketlerini ölçer, filtreler ve ESP-NOW üzerinden 5 komuttan birini 20 Hz hızında yayınlar.
2. **Dış Döngü (Navigasyon & Konum):** Hedef hız komutundan gövde yatış açısı referansı üretir; durulduğunda anında koordinat çıpası atarak konumu kilitler.
3. **İç Döngü (Denge Refleksi):** 100 Hz hızında gövde açısını ve açısal hızını denetleyerek robotun yerçekimine yenilmesini engeller.
4. **Fiziksel Eyleme (Motor & Şasi):** Ters EMK ileri beslemesi ve sürtünme telafisi uygulanarak motorlara tork komutu iletilir.

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

Tek simülasyon modeli gerilim/PWM komutundan motor torku üretir. Çekirdek paket Webots'tan bağımsızdır:

- [config.py](self_balancing_robot/config.py): SSOT merkezi konfigürasyon (`CONFIG` / `RobotConfig`); mekanik, kontrol, motor, sensör füzyonu ve IR yanaşma parametreleri.
- [pid_controller.py](self_balancing_robot/pid_controller.py): Kaskad denge/sürüş kontrolü ve enkoder geri beslemeli PI dönüş kontrolü.
- [dc_motor_model.py](self_balancing_robot/dc_motor_model.py): JGB37 motor direnci, ters EMK, redüktör ve sürtünme modeli.
- [ir_homing.py](self_balancing_robot/ir_homing.py): Diferansiyel IR alıcı-verici nirengisi, olduğu yerde radar tarama (scan) ve huni kılavuza otonom yanaşma kontrolcüsü.
- [sensor_fusion.py](self_balancing_robot/sensor_fusion.py): Saf deterministik tamamlayıcı filtre.
- [telemetry.py](self_balancing_robot/telemetry.py): Gerçek ve kestirilmiş eğim, hız, konum ve gerilim telemetrisi.
- [webots_main.py](webots/controllers/webots_main/webots_main.py): Webots dijital ikiz adaptörü; sensörleri okur, simülasyon gürültüsünü ekler ve motorlara fiziksel tork uygular.

İç döngü 100 Hz açı PD kontrolüyle gerilim üretir. Sürüşte hız hatası hedef eğimi belirler; duruşta konum PI ve hız sönümleme kullanılır. Yüzde 85 ters EMK ileri beslemesi, sürtünme telafisi ve PWM nicemlemesi uygulanır. Dönüş hızı enkoderlerle takip edilir; gerilim doyumunda dengeye öncelik verilir.

Mevcut kazançlar korunur; yeni ayar yalnız doğrudan Webots koşularıyla yapılır. Katsayılar ve optimizasyonun kapsamı [optimizasyon kılavuzunda](docs/pid_optimization.md), motor modelinin varsayımları [gerilim modeli belgesinde](docs/voltage_model.md) açıklanır.

---

## 7. Fiziksel Güvenlik ve Emniyet Kalkanları

1. **Devrilme Emniyet Kesicisi (Fall Protection):** Gövde açısı +/- 45 dereceyi aştığında motor PWM sinyalleri derhal sıfırlanır; motorların ve dişlilerin yanması engellenir.
2. **Donanımsal 3A Cam Sigorta:** Motor sıkışması veya kısa devre anında bataryayı korumak için 3A hızlı atan cam sigorta hattı keser.
3. **Dinamik İlaç Kutusu Kaseti:** 80 × 26 mm ölçülerinde 4 kenarlı dekota bariyer sayesinde ani fren veya dürtmelerde ilaç kutusu hazneden fırlamaz.
4. **Stiction Deadband Filtresi:** Düşük gerilim ve düşük motor hızındaki ölü bölge modellenir; kontrolcü sürtünme telafisi uygular. Bu yaklaşık model tam statik sürtünme çözümü değildir.

---

## 8. Doğrulama ve Simülasyon Testleri

```text
python tests/test_simulation_pid.py
python tests/test_simulation_pid_driving.py
python tests/test_simulation_pid_turning.py
python tests/test_simulation_ir_homing.py
python -m unittest discover -s tests
```

| Test | Kontrol edilen davranış |
|---|---|
| Denge | 30 saniye denge, 1.5 N darbe, ortalama statik eğim ve kutu güvenliği |
| Sürüş | 0.8–1.2 m fiziksel mesafe, frenlemeden sonraki geçiş tamamlanınca üç saniye duruş, geri sürüş ve kutu güvenliği |
| Dönüş | İki gürültü tohumunda yerinde sağ/sol dönüş, tuş bırakınca duruş, eğim ve kayma |
| IR Yanaşma | 30 cm yaklaşma bölgesinden diferansiyel IR nirengisi ve radar tarama ile huni kılavuza otonom giriş, duruş ve durum LED'i |
| Birim testleri | SSOT konfigürasyon tutarlılığı, motor fiziği, IR nirengi, Webots puanlama ve dönüş doyum koruması |

Şartname mesafesi fizik motorundan ölçülür. Testlerin geçmesi gerçek donanımı, eşzamanlı sürüş/dönüşü veya tüm şartname modüllerini doğrulamaz. İleri sürüş, frenleme ve dönüş için farklı senaryolar kullanılır.

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
   Değer olarak Webots çalıştırılabilir dosyasının tam yolunu kullanın.

### Webots Simülasyonunu Açma

```bash
webots webots/worlds/webots_self_balancing_robot.wbt
```

### Klavye ile Teleoperasyon Kontrolleri

Simülasyon penceresine tıkladıktan sonra aşağıdaki tuşlarla robot kontrol edilir:
- **W / Yukarı Ok:** İleri sürüş (güvenli nominal hız)
- **S / Aşağı Ok:** Geri sürüş (güvenli nominal hız)
- **A / Sol Ok:** Sola yumuşak dönüş (~34°/s)
- **D / Sağ Ok:** Sağa yumuşak dönüş (~34°/s)
- **H Tuşu:** Otonom 38 kHz IR alıcı-verici istasyona yanaşma modu (`IRHomingController`)
- **P Tuşu:** Robota 4.0 N harici bozucu itme kuvveti uygula
- **Boşluk (Space):** Dinamik fren yap ve durduğun noktaya konum kilidi at
- **R Tuşu:** Robotu ve ilaç kutusunu başlangıç konumuna ve dik dengeye sıfırla

---

## 10. Dizin Yapısı

```text
self_balancing_robot/
├── .github/workflows/ci.yml              # Python 3.10-3.12 test ve ruff matrisi
├── analysis/                             # Kontrol, telemetri ve Webots optimizasyon araçları
│   ├── README.md                         # Analiz ve doğrulama araçları kılavuzu
│   ├── comparison/                       # Sistem dinamiği ve Webots doğrulama grafik motoru
│   │   └── plot_tuned_system.py          # 300 DPI akademik Webots doğrulama grafikleri üreticisi
│   ├── controllers/pid/                  # Gerilim tabanlı kaskad PID optimizasyon araçları
│   │   ├── optimize_pid_controller.py    # Doğrudan Webots Optuna TPE 6-kazanç araması
│   │   ├── simulation_runner.py          # İzole Webots fizik koşucusu ve puanlayıcısı
│   │   ├── pid_candidate_params.json     # Tam hassasiyetli referans aday kazançları
│   │   └── pid_optuna_results.json       # Çok tohumlu doğrulama ve karşılaştırma özeti
│   └── figures/python/                   # 300 DPI Webots doğrulama figürleri (python_fig1 - python_fig5)
├── app/                                  # Şartname 5.2 Mobil / Kamera değerlendirme uygulaması
├── docs/                                 # Dokümantasyon ve şartnameler
│   ├── specs/                            # Şartname, akademik takvim ve tanıtım PDF'leri
│   ├── reports/                          # FR-1558 yönetim planı ve proje raporları
│   ├── cad/                              # Lazer kesim DXF ve teknik SVG şablonları
│   ├── assets/                           # Mimari şemalar, renderlar ve flowchartlar
│   ├── voltage_model.md                  # DC motor ve TB6612 gerilim/tork modeli varsayımları
│   └── pid_optimization.md               # Doğrudan Webots Optuna PID optimizasyon kılavuzu
├── esp32/                                # Gömülü yazılımlar (ESP32)
│   ├── robot/                            # Denge robotu FreeRTOS/PID firmware'i
│   └── glove/                            # Kontrol eldiveni ESP-NOW firmware'i
├── logs/                                 # Telemetri ve test günlükleri (CSV)
├── self_balancing_robot/                 # Bağımsız çekirdek robot yazılım paketi
│   ├── __init__.py                       # Paket dışa aktarımları
│   ├── config.py                         # SSOT merkezi konfigürasyon (CONFIG / RobotConfig)
│   ├── dc_motor_model.py                 # JGB37 direnç, ters EMK, redüktör ve sürtünme modeli
│   ├── ir_homing.py                      # Diferansiyel IR nirengi ve istasyona yanaşma kontrolcüsü
│   ├── pid_controller.py                 # Gerilim tabanlı kaskad denge ve PI dönüş kontrolcüsü
│   ├── sensor_fusion.py                  # Saf deterministik tamamlayıcı filtre
│   └── telemetry.py                      # Zaman serisi CSV telemetri kaydedicisi
├── tests/                                # Pytest birim testleri ve Webots doğrulama testleri
├── webots/                               # Webots R2025a dijital ikiz ortamı
│   ├── worlds/webots_self_balancing_robot.wbt  # Dijital ikiz simülasyon dünyası
│   └── controllers/webots_main/          # Üretim kodunu çalıştıran Webots kontrolcü adaptörü
├── pyproject.toml                        # Paket ve pytest yapılandırması
├── requirements.txt                      # Python çalışma, analiz ve test bağımlılıkları
├── pyrightconfig.json                    # Pyright tip ve modül yolu yapılandırması
├── ruff.toml                             # Ruff lint yapılandırması
├── CITATION.cff                          # Akademik atıf üstverisi
├── CHANGELOG.md                          # Sürüm ve değişiklik geçmişi
├── AGENTS.md                             # Teknik hafıza, mimari kararlar ve proje arşivi
├── README.md                             # Proje ana dokümantasyonu
└── LICENSE                               # MIT
```

---

## 11. Dokümantasyon ve Proje Hafızası

- **Proje Teknik Hafızası & Karar Günlüğü:** [AGENTS.md](AGENTS.md)
- **Analiz ve Doğrulama Araçları:** [analysis/README.md](analysis/README.md)
- **Sürüm Değişiklik Geçmişi:** [CHANGELOG.md](CHANGELOG.md)
- **Resmi Şartname Belgesi:** [YTU_EEF_CDTP_Ev_Ici_Ilac_Tasima_Denge_Robotu_TR_v14.pdf](docs/specs/YTU_EEF_CDTP_Ev_Ici_Ilac_Tasima_Denge_Robotu_TR_v14.pdf)
- **Lazer Kesim Şablonu:** [pleksi_katmanlari.dxf](docs/cad/pleksi_katmanlari.dxf) ve [pleksi_kesim_sablonu.svg](docs/cad/pleksi_kesim_sablonu.svg)

---

## 12. Lisans ve Telif Hakkı

Bu proje [MIT Lisansı](LICENSE) kapsamında lisanslanmıştır. Yıldız Teknik Üniversitesi Elektrik-Elektronik Fakültesi ÇDTP bitirme projesi kapsamında geliştirilmiştir.
