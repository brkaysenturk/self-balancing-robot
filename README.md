# İki Tekerlekli Ev İçi Hasta Destek ve İlaç Taşıma Denge Robotu

Bu proje; Yıldız Teknik Üniversitesi Elektrik-Elektronik Fakültesi Çok Disiplinli Tasarım Projesi (ÇDTP) kapsamında geliştirilen **aktif dengelenen iki tekerlekli otonom mobil sağlık asistanı robotudur**.

---

## 📌 Proje Özeti ve Yetenekleri

Robot, ters sarkaç (Inverted Pendulum) prensibiyle iki tekerleği üzerinde aktif dinamik denge kurarak aşağıdaki görevleri icra eder:

1. **Aktif Statik ve Dinamik Denge (Modül A):** Dış destek almadan en az 30 saniye dik konumda dengede kalma ve harici itme bozuntularını (disturbances) toparlama.
2. **Giyilebilir Eldivenle Teleoperasyon (Modül B):** Operatörün el hareketlerinden türetilen 5 komutla (İleri, Geri, Sola Dön, Sağa Dön, Dur) kablosuz ESP-NOW protokolü üzerinden 1 metre mesafe katetme.
3. **Standart Görev Yükü (Modül E):** Şartname Madde 5.3'e uygun standart kapalı bir karton ilaç (Aspirin 500 mg / Parol) kutusunu üst tavanındaki düşme önleyici kaset cebinde güvenle taşıma.
4. **Otonom Şarj ve Docking (Modül C & D):** 30 cm yarıçaplı yaklaşma bölgesinden 38 kHz kızılötesi (IR) nirengisiyle şarj istasyonunu tespit etme, yönelme ve insan müdahalesi olmadan temas pedleriyle otomatik şarj başlatma.
5. **Kamera Tabanlı Değerlendirme (Modül 5.2):** Akıllı telefon kamerası ve kablosuz telemetriyle robotun görev süresini, kat ettiği mesafeyi ve eğim açısını canlı takip edip raporlama.

---

## 🛠 Donanım Mimarisi & Şasi Özellikleri

- **Şasi Boyutları:** 135 mm (Genişlik) × 95 mm (Derinlik) × 180 mm (Yükseklik)
- **Toplam Kütle:** ~910 – 920 gram (Ağırlık merkezi yerden ~10 cm yükseklikte)
- **Tekerlek İz Genişliği:** 17.0 cm (85 mm silikon tekerlekler, yerleşik şasi tampon korumasıyla)
- **Gövde Mimarisi:** 4 katmanlı monolitik kafes (4x kesintisiz paslanmaz M3 tij, çift somun + geniş pul sıkıştırmalı 5 mm Beyaz Dekota / 3 mm Döküm Pleksi)
- **Dikey Kablo Bacası:** Tüm katmanlarda aynı hizada (32×12 mm) açılan iç şaft; sıfır dış kablo sarkması
- **Tahrik Sistemi:** 2x JGB37-520 12V 330 RPM Çelik Redüktörlü ve Manyetik Hall Enkoderli DC Motor + TB6612FNG Çift Yönlü MOSFET Sürücü (Doğrudan tabanda, 3 cm güç hattıyla sıfır EMI)
- **Ana Denetleyici:** ESP32 DevKit V1 (240 MHz Dual-Core, dahili Wi-Fi & ESP-NOW)
- **Sensör Füzyonu:** GY-521 MPU-6050 6-Eksen IMU (Katman 3 tam geometrik merkezinde) + 2x TSOP38438 38 kHz IR Alıcı (85 mm diferansiyel nirengi aralığı)
- **Güç Sistemi:** 3S1P 18650 Li-ion Batarya (11.1V nominal, 12.6V tam şarj, 2000 mAh) + 3S 10A/15A BMS + XL4015 5A Regülatör + 3A Hızlı Cam Sigorta + KCD11 Switch + XT60 Güç Soketi

---

## 📂 Dizin Yapısı

```text
c:\YTU\CDTP\
├── docs/                      # Dokümantasyon ve şartnameler
│   ├── specs/                 # Şartname, akademik takvim ve tanıtım dosyaları
│   ├── reports/               # FR-1558 yönetim planı ve proje raporları
│   ├── cad/                   # Lazer kesim DXF ve teknik SVG şablonları
│   └── assets/                # Sistem mimarisi şemaları, renderlar ve flowchartlar
├── esp32/                     # Gömülü yazılımlar (ESP32)
│   ├── robot/                 # Denge robotu ESP32 firmware'i (PID/LQR)
│   └── glove/                 # Kontrol eldiveni ESP32 firmware'i (ESP-NOW)
├── app/                       # Şartname 5.2 Mobil / Kamera test değerlendirme uygulaması
├── simulation/                # Denge ve kontrol dinamiği simülatörü
├── analysis/                  # Telemetri, kontrol teorisi ve analiz grafikleri
├── tests/                     # Donanım doğrulama test scriptleri
├── logs/                      # Telemetri ve çalışma kayıtları
├── .editorconfig              # Kod biçimlendirme kuralları
├── .gitattributes             # Git satır sonu kuralları
├── .gitignore                 # Git versiyon kontrolü kuralları
├── CHANGELOG.md               # Sürüm ve değişiklik geçmişi
├── AGENTS.md                  # Teknik hafıza, mimari kararlar ve kurallar arşivi
└── README.md                  # Proje ana dokümanı
```

---

## 📐 Hızlı Erişim & Tasarım Dosyaları

- **Lazer Kesim / Dekota CAD Dosyası:** [pleksi_katmanlari.dxf](file:///c:/YTU/CDTP/docs/cad/pleksi_katmanlari.dxf)
- **1:1 Vektörel Teknik Çizim Şablonu:** [pleksi_kesim_sablonu.svg](file:///c:/YTU/CDTP/docs/cad/pleksi_kesim_sablonu.svg)
- **Resmi ÇDTP Şartnamesi:** [YTU_EEF_CDTP_Ev_Ici_Ilac_Tasima_Denge_Robotu_TR_v14.pdf](file:///c:/YTU/CDTP/docs/specs/YTU_EEF_CDTP_Ev_Ici_Ilac_Tasima_Denge_Robotu_TR_v14.pdf)
- **Sistem Mimarisi Çizimi:** [sistem_mimarisi.svg](file:///c:/YTU/CDTP/docs/assets/sistem_mimarisi.svg)
- **İş Akışı (Flowchart):** [flowchart.svg](file:///c:/YTU/CDTP/docs/assets/flowchart.svg)
- **Detaylı Karar ve Hafıza Dokümanı:** [AGENTS.md](file:///c:/YTU/CDTP/AGENTS.md)

