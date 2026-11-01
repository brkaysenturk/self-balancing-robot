# YTU ÇDTP — İki Tekerlekli Ev İçi Hasta Destek ve İlaç Taşıma Robotu

## Kapsamlı Proje Hafızası, Mimari Kararlar ve Gelişim Dokümanı (AGENTS.md)

> Bu doküman; projenin en başından bugüne kadar geçirdiği tüm tasarım fazlarını, mekanik, elektronik ve kontrol teorisi kararlarını, "Neden böyle yaptık?" arşivini ve gelecekte açılacak yeni yapay zeka sohbetlerinin bağlamı hiç kaybetmemesi için gereken tam teknik hafızayı içerir.

---

## 1. Kesin Biçimlendirme ve İletişim Kuralları (STRICT)

- **ASLA LATEX VEYA DOLAR İŞARETİ (`$`, `$$`) KULLANILMAYACAK:**
  Kullanıcı arayüzü LaTeX/MathJax render etmez; ham kod gibi görünür.
  Tüm matematik, açılar, birimler ve formüller düz metin (plain text) olarak yazılacaktır:
  Örnek: `45 derece`, `5.0 saniye`, `30 FPS`, `50 Hz`, `Kp = 0.16`, `v_max = A * w`, `+/- 45 derece`, `tau = 45 ms`.
- **DOKÜMANTASYONA VE COMMITLERE ANLIK KONUŞMA SAYILARI YAZILMAYACAK:**
- **DOSYA BAĞLANTILARI:** Kod ve tasarım dosyalarına her zaman GitHub markdown link formatında (`[dosya_adi](file:///c:/YTU/self-balancing-robot/...)`) tıklanabilir referans verilecektir.

---

## 2. Proje Tanımı ve Şartname İsterleri (YTÜ EEF ÇDTP)

- **Resmi Şartname Belgesi:** [YTU_EEF_CDTP_Ev_Ici_Ilac_Tasima_Denge_Robotu_TR_v14.pdf](file:///c:/YTU/self-balancing-robot/docs/specs/YTU_EEF_CDTP_Ev_Ici_Ilac_Tasima_Denge_Robotu_TR_v14.pdf)
- **Ana Hedef:** Kendi dengesini aktif olarak sağlayan iki tekerlekli robot (Inverted Pendulum); giyilebilir eldiven arayüzüyle yönlendirilerek standart bir ilaç (Aspirin vb.) kutusunu ev içinde taşır ve ardından insan müdahalesi olmadan otonom biçimde şarj istasyonuna yerleşip şarjı başlatır.

### Şartname Modülleri ve Başarı Metrikleri

1. **Modül A (Statik Denge):** Robotun dış destek almadan en az 30 saniye boyunca dik ve dengede kalması.
2. **Modül B (Robot Yönlendirmesi):** Giyilebilir eldiven üzerinden verilen kablosuz komutlarla (ileri, geri, sola dön, sağa dön, dur) 0.8 m – 1.2 m mesafe katetmesi.
3. **Modül C (Otonom Şarj):** 30 cm yarıçaplı 90 derece yaklaşma bölgesinden itibaren IR nirengisiyle istasyonu bulması, yanaşıp hizalanması ve 5 saniye içinde fiziksel temas pedleriyle şarjı otomatik başlatması.
4. **Modül D (Şarj Göstergesi):** Donanımsal durum LED'inin şarj başladığında yanması, ayrıldığında 2 saniye içinde sönmesi.
5. **Modül E (Bütünleşik Görev & İlaç Taşıma):** Standart kapalı bir ilaç (Aspirin / Parol) kutusunu düşürmeden tüm görevi tamamlaması.
6. **Modül 5.2 (Mobil Kamera Değerlendirme Uygulaması):** Telefon kamerasıyla robotun konumunu, eğimini ve süresini ölçen harici izleme/doğrulama arayüzü.

---

## 3. Mekanik Mimari & Tasarım Kararları

### 3.1. Şasi Ölçüleri: 135 mm Genişlik × 95 mm Derinlik (Altın Oran)

- **Neden 135 × 95 mm?**
  - **Tekerlek Koruması (Doğal Tampon):** Tekerlek çapı 85 mm, şasi derinliği 95 mm'dir. Şasi, tekerleklerin önünden ve arkasından 5'er mm taşarak engellere karşı tampon görevi görür.
  - **Motor İçi Kablo Koridoru:** 135 mm plaka genişliği sayesinde tabandaki iki JGB37 motor arasında 35 mm açık kablo koridoru kalır.
  - **İlaç Kutusu Oranı:** Katman 4'teki 80 × 26 mm kaset yuvası plakanın ortasında dengeli durur; sağında ve solunda 27 mm, ön/arkasında 35 mm et payı bırakır.
  - **Hammadde Verimi:** 4 katmanın tamamı standart 20 × 30 cm Dekota tabakasına (27 × 19 cm yerleşimle) sıfır israfla sığar; 35 × 50 cm tabakadan ise 8-9 plaka çıkar.

### 3.2. Malzeme Seçimi: 5 mm Beyaz Dekota (Forex) & 3 mm Pleksi

- **Neden 5 mm Dekota?**
  - **Kırılmazlık:** Robot ilk PID denemelerinde devrildiğinde pleksi gibi çatlamaz, darbeyi emer.
  - **Evde Kolay İşleme:** Matkap gerekmeden tornavida veya maket bıçağıyla pürüzsüzce delinir ve kesilir.
  - **Kablo Gizleme:** Şeffaf pleksi iç kabloları gösterip dağınık dururken, mat beyaz dekota kabloları gizler ve medikal cihaz estetiği sunar.

### 3.3. Tij (Gijon) vs. Distans (Standoff) Kararı

- **Problem:** Modüler pirinç/naylon distanslar motor titreşimlerinde vida dişlerinden gevşer ve katmanlar üst üste eklendikçe gövdede esnemeye yol açar. Bu mekanik esneme MPU-6050 sensörüne yalancı açısal ivme basarak PID döngüsünü kararsızlaştırır.
- **Çözüm:** Taban katmanından en üst kata kadar uzanan **4 adet kesintisiz M3 paslanmaz çelik tij (gijon)** kullanıldı. Her katman altından ve üstünden M3 pul ve somunla sıkıştırılarak monolitik kutu kafes (box-truss) yapısı elde edildi.

### 3.4. Dikey Kablo Bacası (Conduit Riser) Mimarisi

- Tüm katmanlarda birebir aynı koordinatta (`X=67.5 mm`, `Y=22 mm`) **32 mm × 12 mm** dikey kablo bacası açıldı.
- Kablolar şasinin dışından sarkmaz; asansör bacası gibi içeriden yukarı çıkar. Tekerleklere dolanma veya dış engellere takılma riski sıfırdır.

### 3.5. 4 Katmanın İşlev Dağılımı

1. **Katman 1 (Taban / Base Deck):** JGB37 Motor L-braketleri, **TB6612FNG Motor Sürücüsü**, Şarj temas pedleri (2x 12×5 mm), Dikey kablo bacası.
2. **Katman 2 (Güç / Power Deck):** 3S 18650 Li-ion batarya paketi (Ağırlık merkezi CoM = 10 cm hizası), XL4015 5A regülatör, velcro kayış yarıkları, Dikey kablo bacası.
3. **Katman 3 (Kontrol / Brain Deck):** ESP32 DevKit V1, **MPU-6050 (Tam merkez X=0, Y=0)**, Ön kenarda 85 mm aralıklı 2x TSOP38438 IR gözleri, Dikey kablo bacası.
4. **Katman 4 (Tavan / Top Deck):** **80 × 26 mm Aspirin/Parol kaset yuvası**, KCD11 Rocker Switch (19×13 mm), 3A Panel Tipi Cam Sigorta Yuvası (Ø12.2 mm), Durum/Şarj LED'i (Ø5.0 mm).

---

## 4. Donanım & Güç Mimarisi (Power Budget)

### 4.1. Güç Dağıtım Hattı

- **Ana Batarya Hattı (12V Ham Güç):**
  - Batarya (+) ➔ Katman 4 Sigorta (3A) ➔ KCD11 Switch ➔ Katman 1 TB6612 `VM` pini & Katman 2 XL4015 `IN(+)`.
  - Kablo: 0.50 mm² veya 22 AWG çok damarlı montaj teli.
  - Batarya soketi: Kablolu XT60 Erkek (10 cm silikon kablolu).
- **5V Lojik Hattı (XL4015 Step-Down Çıkışı):**
  - XL4015 5.0V çıkışı ➔ Katman 3 ESP32 `VIN` pini, TB6612 `VCC` pini ve Manyetik Enkoder beslemeleri.
- **3.3V Sensör Hattı (ESP32 Dahili LDO):**
  - ESP32 3.3V ➔ MPU-6050 `VCC`, TSOP38438 `VCC`.

### 4.2. TB6612 Taban Katmanı Kararı (EMI İzolasyonu)

- Motor sürücü (TB6612) Katman 3'ten Katman 1'e indirildi.
- **Kazanım:** Motorlara giden yüksek akımlı (12V, 2-3A) 20 kHz PWM hatları sadece **3 cm** boyunda kaldı. Yüksek akım hatları üst kata çıkmadığı için ESP32 ve MPU-6050 üzerindeki elektromanyetik parazit (EMI) sıfırlandı.

### 4.3. Batarya Tüketim Profili (3S 18650 Li-ion, 2000 mAh)

- Bekleme Akımı (Motorlar kapalı, ESP32 aktif): ~0.12 A
- Normal Denge Akımı (Ortalama seyir): ~0.85 A – 1.15 A (Ortalama 1.0 A)
- Dinamik Toparlanma / Ani İvme (Tepe akım): ~2.4 A – 3.2 A
- Kilitlenme (Stall eşiği): ~4.5 A – 5.0 A (3A sigorta ile korunur)
- **Çalışma Süresi:** 2000 mAh / 1.0 A = **Yaklaşık 1.5 – 2.0 saat kesintisiz çalışma.**

---

## 5. Kontrol & Gömülü Yazılım Mimarisi (Firmware)

### 5.1. Dinamik Model (Inverted Pendulum on Wheels)

- **Durum Değişkenleri:**
  - `theta`: Gövde eğim açısı (derece / radyan)
  - `theta_dot`: Gövde açısal hızı (derece/saniye)
  - `x`: Robot tekerlek konumu (metre, enkoderlerden entegre)
  - `v`: Robot tekerlek çizgisel hızı (metre/saniye)

### 5.2. Çift Döngülü Kaskad PID Kontrolü

- **İç Döngü (Denge & Açı Kontrolü - 100 Hz / 10 ms):**
  - MPU-6050 tamamlayıcı filtre (Complementary Filter: %98 Gyro + %2 Accel) veya DMP çıktısı okunur.
  - `u_denge = Kp_theta * theta + Kd_theta * theta_dot`
- **Dış Döngü (Hız & Konum Kontrolü - 100 Hz / 10 ms):**
  - Enkoder pulse verileri okunur; robotun sürüklenmesini önlemek ve eldivenden gelen hız komutunu takip etmek için açı referansına (`theta_ref`) küçük ofset basılır:
  - `theta_ref = Kp_v * (v_hedef - v) + Ki_v * e_v_int`

### 5.3. Kablosuz Haberleşme (ESP-NOW)

- Harici telsiz modülü (NRF24L01 / Bluetooth) yerine ESP32'nin yerel **ESP-NOW protokolü** kullanılır.
- 2.4 GHz doğrudan MAC adresi tabanlı paket iletimi.
- Gecikme süresi: **1 – 2 milisaniye** (Wi-Fi yönlendirici/router gerektirmez).
- Paket İçeriği (Eldiven ➔ Robot): 5 komut (İleri, Geri, Sol, Sağ, Dur) + Kalibrasyon bayrağı.

---

## 6. Dizin Ağacı ve Dosya Yerleşimi

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
│   └── figures/python/                   # 300 DPI Webots doğrulama figürleri (python_fig1 - python_fig4)
├── app/                                  # Şartname 5.2 Mobil / Kamera Değerlendirme Uygulaması
├── docs/                                 # Proje dokümantasyonu ve şartnameler
│   ├── specs/                            # Resmi şartname, takvim ve tanıtım PDF'leri
│   ├── reports/                          # FR-1558 yönetim planı ve proje raporları
│   ├── cad/                              # Lazer kesim DXF ve teknik SVG şablonları (135x95 mm)
│   ├── assets/                           # Flowchart, sistem mimarisi ve görsel renderlar
│   ├── voltage_model.md                  # DC motor ve TB6612 gerilim/tork modeli varsayımları
│   └── pid_optimization.md               # Doğrudan Webots Optuna PID optimizasyon kılavuzu
├── esp32/                                # Gömülü yazılımlar (ESP32)
│   ├── robot/                            # İki tekerlekli robot FreeRTOS/PID kontrol firmware'i
│   └── glove/                            # Giyilebilir kontrol eldiveni ESP-NOW firmware'i
├── logs/                                 # Telemetri CSV ve test günlükleri
├── self_balancing_robot/                 # Bağımsız çekirdek robot yazılım paketi
│   ├── __init__.py                       # Paket dışa aktarımları
│   ├── config.py                         # SSOT merkezi konfigürasyon (CONFIG / RobotConfig)
│   ├── dc_motor_model.py                 # JGB37 armatür direnci, ters EMK, redüktör ve sürtünme modeli
│   ├── ir_homing.py                      # 38 kHz IR verici ve 2x TSOP38438 diferansiyel yanaşma kontrolcüsü
│   ├── pid_controller.py                 # Gerilim tabanlı kaskad denge kontrolü ve PI dönüş kontrolcüsü
│   ├── sensor_fusion.py                  # Saf deterministik tamamlayıcı filtre (%98 Gyro + %2 Accel)
│   └── telemetry.py                      # Zaman serisi CSV telemetri kaydedicisi
├── tests/                                # Pytest / unittest birim ve Webots doğrulama testleri
├── webots/                               # Webots R2025a dijital ikiz ortamı
│   ├── worlds/webots_self_balancing_robot.wbt  # Dijital ikiz simülasyon dünyası
│   └── controllers/webots_main/          # Üretim kodunu çalıştıran Webots kontrolcü adaptörü
├── pyproject.toml                        # Paket ve pytest yapılandırması
├── requirements.txt                      # Python çalışma, analiz ve test bağımlılıkları
├── pyrightconfig.json                    # Pyright tip ve modül yolu yapılandırması
├── ruff.toml                             # Ruff lint yapılandırması
├── CITATION.cff                          # Akademik atıf üstverisi
├── CHANGELOG.md                          # Sürüm ve değişiklik geçmişi
├── AGENTS.md                             # Teknik hafıza ve mimari kararlar arşivi
├── README.md                             # Proje genel tanıtımı ve hızlı başlangıç kılavuzu
└── LICENSE                               # MIT
```

---

## 7. Webots Simülasyonu ve Gerilim Kontrolü

- **Simülasyon Dünyası:** [webots_self_balancing_robot.wbt](file:///c:/YTU/self-balancing-robot/webots/worlds/webots_self_balancing_robot.wbt)
- **Webots Adaptörü:** [webots_main.py](file:///c:/YTU/self-balancing-robot/webots/controllers/webots_main/webots_main.py)
- **Çekirdek Paket (`self_balancing_robot`):**
  - Robot & Kontrolör Konfigürasyonu (SSOT): [config.py](file:///c:/YTU/self-balancing-robot/self_balancing_robot/config.py) (`CONFIG`, `RobotConfig`)
  - JGB37 & TB6612 Fiziksel Motor Modeli: [dc_motor_model.py](file:///c:/YTU/self-balancing-robot/self_balancing_robot/dc_motor_model.py) (`DCMotorModel`)
  - Gerilim Tabanlı Kaskad PID & PI Dönüş Kontrolcüsü: [pid_controller.py](file:///c:/YTU/self-balancing-robot/self_balancing_robot/pid_controller.py) (`PIDController`)
  - Deterministik Tamamlayıcı Filtre: [sensor_fusion.py](file:///c:/YTU/self-balancing-robot/self_balancing_robot/sensor_fusion.py) (`SensorFusion`)
  - Zaman Serisi Telemetri Kaydedici: [telemetry.py](file:///c:/YTU/self-balancing-robot/self_balancing_robot/telemetry.py) (`TelemetryLogger`)

### 7.1. Fiziksel ve Kinematik Modelleme

- **Koordinat Sistemi:** Webots ENU (+X Doğu/Sağ, +Y Kuzey/İleri, +Z Yukarı).
- **Şasi Ebatları:** 135 mm × 95 mm × 5 mm 4 katmanlı Dekota plaka.
- **Tekerlekler:** 85 mm çap (yarıçap R = 0.0425 m), 170 mm iz genişliği, kauçuk zemin sürtünmesi (`coulombFriction 1.8`).
- **Taşıyıcı Kolonlar:** 4 adet M3 paslanmaz çelik tij (Ø3.2 mm, 122 mm boy).
- **Kütle ve Ağırlık Merkezi:** Toplam kütle 0.86 kg, CoM mil ekseninden 55 mm yukarıda (yerden yaklaşık 97.5 mm).
- **Bileşenler:** Taban katmanında TB6612FNG ve JGB37 motorlar, Katman 2'de 3S 18650 Li-ion batarya, Katman 3'te ESP32 ve MPU-6050, Katman 4'te Aspirin ilaç kutusu kaseti, switch, sigorta ve durum LED'i.

### 7.2. Kontrol ve Optimizasyon

- **İç ve Dış Döngü (100 Hz / 10 ms):** İç açı döngüsü 100 Hz PD ile gerilim üretir (`KP_VOLT = 60.7944`, `KD_VOLT = 1.2985`). Sürüşte hız hatası hedef eğime dönüştürülür (`KP_VEL_DRIVE = 0.2486`); duruşta konum PI (`KP_POS = 0.1000`, `KI_POS = 0.0104`) ve hız sönümleme (`KV_VEL = 0.2952`) kullanılır.
- **Motor Modeli ve İleri Besleme:** Gerilim komutu; sürtünme telafisi ve tekerlek başına yüzde 85 ters EMK ileri beslemesinden sonra 8 bit PWM üzerinden [dc_motor_model.py](file:///c:/YTU/self-balancing-robot/self_balancing_robot/dc_motor_model.py) modeline uygulanır. Motor modeli armatür direnci, ters EMK, redüktör ve sürtünmeden fiziksel tork hesaplar; kaymayı Webots zemin teması çözer.
- **Denge Öncelikli PI Dönüş Kontrolü:** Sağ/sol dönüş, iki enkoderin yarı hız farkından PI geri beslemesiyle (`KP_TURN = 0.20`, `KI_TURN = 1.0`) kontrol edilir. Gerilim doyumunda dengeye mutlak öncelik verilir. `TURN_SPEED = 1.2 rad/s` mevcut geometride yaklaşık 34 derece/saniye gövde dönüş hedefidir.
- **Doğrudan Webots Optimizasyonu:** Yeniden ayar yalnız doğrudan Webots ölçümleriyle yapılır: [optimize_pid_controller.py](file:///c:/YTU/self-balancing-robot/analysis/controllers/pid/optimize_pid_controller.py) Optuna adaylarını, [simulation_runner.py](file:///c:/YTU/self-balancing-robot/analysis/controllers/pid/simulation_runner.py) fizik koşularını ve puanlamayı yönetir. Mevcut kazançlar korunur; arama kaynak değerlerini otomatik değiştirmez.
- **Donanım Kalibrasyon Sınırları:** Donanım hazır olduğunda motor/enkoder, batarya yük gerilimi, sürücü kayıpları, IMU ofseti ve kütle/ağırlık merkezi ölçülerek model ve donanım ayarı güncellenececek. Öncesinde model sabitlerini ölçülmüş veri saymayın.
- **Kayıtlar ve Belgeler:** Tam katsayılar ve son test özeti `analysis/controllers/pid/` altında ([pid_candidate_params.json](file:///c:/YTU/self-balancing-robot/analysis/controllers/pid/pid_candidate_params.json), [pid_optuna_results.json](file:///c:/YTU/self-balancing-robot/analysis/controllers/pid/pid_optuna_results.json)) korunur. Model varsayımları [voltage_model.md](file:///c:/YTU/self-balancing-robot/docs/voltage_model.md) ve optimizasyon yöntemi [pid_optimization.md](file:///c:/YTU/self-balancing-robot/docs/pid_optimization.md) belgelerindedir.
- **Teleoperasyon Tuşları:** W/S ileri/geri, A/D dönüş, Space frenleme, P dış dürtme, R fizik ve kontrolcü sıfırlamasıdır. İtme sonrası eski konuma dönmek zorunlu değildir.

### 7.3. Doğrulama Testleri

- **Statik Denge ve Bozucu Testi ([test_simulation_pid.py](file:///c:/YTU/self-balancing-robot/tests/test_simulation_pid.py)):** 30 saniye denge, 1.5 N / 80 ms itme, ilaç kutusu güvenliği ve hızın sönmesi.
- **Sürüş ve Duruş Testi ([test_simulation_pid_driving.py](file:///c:/YTU/self-balancing-robot/tests/test_simulation_pid_driving.py)):** 0.8–1.2 m fiziksel mesafe, geçiş sonrası üç saniye duruş, geri sürüş ve ilaç kutusu güvenliği.
- **Dönüş Testi ([test_simulation_pid_turning.py](file:///c:/YTU/self-balancing-robot/tests/test_simulation_pid_turning.py)):** İki gürültü tohumunda (42, 142) yerinde iki yönlü dönüş, duruş, kutu güvenliği ve tekerlek kayması.
- **IR Alıcı-Verici İstasyona Yanaşma Testi ([test_simulation_ir_homing.py](file:///c:/YTU/self-balancing-robot/tests/test_simulation_ir_homing.py)):** 30 cm yaklaşma konisi içinden 2x TSOP38438 alıcı ve 38 kHz IR verici fener nirengisiyle hizalanıp istasyon önünde otonom duruş.
- **Birim Testleri:** [test_config_consistency.py](file:///c:/YTU/self-balancing-robot/tests/test_config_consistency.py), [test_dc_motor_model.py](file:///c:/YTU/self-balancing-robot/tests/test_dc_motor_model.py), [test_sensor_fusion.py](file:///c:/YTU/self-balancing-robot/tests/test_sensor_fusion.py), [test_pid_controller.py](file:///c:/YTU/self-balancing-robot/tests/test_pid_controller.py), [test_ir_homing.py](file:///c:/YTU/self-balancing-robot/tests/test_ir_homing.py) ve [test_pid_optimizer.py](file:///c:/YTU/self-balancing-robot/tests/test_pid_optimizer.py).
- **Telemetri ve Kutu Denetimi:** Eğim telemetrisinde `pitch_deg` gerçek fizik açısı, `pitch_est_deg` tamamlayıcı filtre kestirimidir. Gerilim komut sütunları volt birimindedir (`cmd_left_volts`, `cmd_right_volts`). Kutu güvenliği robotun yerel ekseninde denetlenir. Eşzamanlı sürüş/dönüş ve gerçek donanım bu testlerle doğrulanmış sayılmaz.
