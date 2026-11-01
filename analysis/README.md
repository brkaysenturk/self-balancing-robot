# Analysis Tools & Verification Suite (Analiz ve Doğrulama Araçları)

Bu dizin; İki Tekerlekli Denge Robotu projesinin kaskad PID kontrol dinamiklerini, DC motor/TB6612 tork modelini, tamamlayıcı filtre kestirim doğruluğunu ve Webots R2025a fizik motoru telemetri kayıtlarını incelemek, optimize etmek ve 300 DPI akademik grafiklere dönüştürmek için geliştirilen mühendislik araçlarını modüler bir hiyerarşide içerir.

---

## Dizin Hiyerarşisi

```text
analysis/
├── README.md                          # Analiz ve optimizasyon araçları kılavuzu
├── comparison/                        # Sistem dinamiği ve Webots doğrulama grafik motoru
│   └── plot_tuned_system.py           # 300 DPI akademik Webots doğrulama grafikleri üreticisi
├── controllers/                       # Kontrolcü optimizasyon ve tarama araçları
│   └── pid/                           # Gerilim tabanlı kaskad PID kontrolcü araçları
│       ├── optimize_pid_controller.py # Doğrudan Webots Optuna TPE 6-kazanç optimizasyonu
│       ├── simulation_runner.py       # İzole Webots fizik koşucusu ve çoklu senaryo puanlayıcısı
│       ├── pid_candidate_params.json  # Tam hassasiyetli referans aday kazançları
│       └── pid_optuna_results.json    # Çok tohumlu doğrulama ve karşılaştırma sonuç özeti
└── figures/                           # Tüm akademik analiz ve doğrulama grafikleri
    └── python/                        # Webots ve alt sistem doğrulama figürleri (python_fig1 - python_fig5)
```

---

## Üretilen Akademik Figürler (`analysis/figures/python/`)

1. **`python_fig1_static_balance_and_push_recovery.png`:** Modül A & E kapsamında 30 saniye statik denge, t = 5.0 s (+1.5 N) ve t = 17.0 s (-1.5 N) harici dürtme darbeleri altında gövde eğim açısı, açısal hız, çizgisel hız sönümlemesi ve TB6612 motor gerilim yanıtı.
2. **`python_fig2_driving_and_position_hold.png`:** Modül B & E kapsamında 0.80 m – 1.20 m hedef bölgeye ileri sürüş, 3 saniye (13–16 s) hareketsiz konum tutma, geri sürüş hız takibi ve kaskad eğim-gerilim eforu.
3. **`python_fig3_differential_turning_and_slip.png`:** Modül B kapsamında enkoder geri beslemeli denge öncelikli PI dönüş kontrolcüsünün sola/sağa dönüş hızı (yaw rate) takibi, diferansiyel tekerlek gerilimleri, dönüş esnasındaki eğim kararlılığı ve zemin kayma hızı (slip speed).
4. **`python_fig4_sensor_fusion_and_motor_characteristics.png`:** MPU-6050 tamamlayıcı filtrenin (%98 Gyro + %2 Accel) gerçek eğim açısına karşı kestirim hatası dağılımı ve JGB37-520 (30:1) DC motorun tork-hız eğrileri ile kalkış ölü bölgesi (deadband) telafisi.
5. **`python_fig5_ir_homing_docking_response.png`:** Modül C & E kapsamında Katman 3 ön kenardaki 85 mm aralıklı 2x TSOP38438 IR alıcısının 38 kHz IR verici fenerine karşı diferansiyel sinyal karakteristiği ve 30 cm yaklaşma konisinden istasyona otonom hizalanıp durma yanıtı.

---

## Hızlı Başlangıç Komutları

### 1. Webots Doğrulama ve Akademik Grafik Üretimi

```bash
# Tüm Webots senaryolarını koşturup 5 adet 300 DPI PNG figürünü üretir
python analysis/comparison/plot_tuned_system.py
```

### 2. Kaskad PID Kontrolcü Optuna TPE Optimizasyonu

```bash
# Doğrudan Webots üzerinde 100 denemelik çok-senaryolu (denge, dürtme, sürüş) arama
python analysis/controllers/pid/optimize_pid_controller.py --trials 100
```

### 3. Bütünleşik Simülasyon ve Birim Doğrulama Testleri

```bash
# Hızlı birim ve SSOT tutarlılık testleri (CONFIG, motor modeli, tamamlayıcı filtre, PI dönüş, IR yanaşma, puanlayıcı)
python -m unittest discover -s tests -v

# Modül A & E: 30 saniye statik denge, 1.5 N dürtme ve ilaç kutusu güvenliği
python tests/test_simulation_pid.py

# Modül B & E: 0.8 - 1.2 m sürüş, 3 saniye duruş, geri sürüş ve ilaç kutusu güvenliği
python tests/test_simulation_pid_driving.py

# Modül B & E: İki gürültü tohumunda yerinde sağ/sol dönüş ve duruş doğrulaması
python tests/test_simulation_pid_turning.py

# Modül C & E: 38 kHz IR alıcı-verici nirengisiyle 30 cm koniden istasyona otonom yanaşma
python tests/test_simulation_ir_homing.py
```
