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
- **Dış Döngü (Hız & Konum Kontrolü - 20 Hz / 50 ms):**
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
c:\YTU\self-balancing-robot\
├── .vscode/               # VSCode ayarları ve dosya format kuralları
├── docs/                  # Proje dokümantasyonu ve şartnameler
│   ├── specs/             # Resmi şartname, takvim ve tanıtım PDF'leri
│   ├── reports/           # FR-1558 yönetim planı ve proje raporları
│   ├── cad/               # Lazer kesim DXF ve teknik SVG şablonları (135x95 mm)
│   └── assets/            # Flowchart, sistem mimarisi ve görsel renderlar
├── self_balancing_robot/  # Bağımsız çekirdek robot yazılım paketi
│   ├── __init__.py        # Paket dışa aktarımları
│   ├── config.py          # Robot parametreleri, PID kazançları ve eşikler
│   ├── sensor_fusion.py   # Tamamlayıcı filtre ve sensör gürültüsü
│   ├── cascade_controller.py # Kaskad PID, anti-windup ve sürtünme telafisi
│   └── telemetry.py       # Telemetri veri kaydedicisi
├── esp32/                 # Gömülü yazılımlar (ESP32)
│   ├── robot/             # İki tekerlekli robot FreeRTOS/PID kontrol firmware'i
│   └── glove/             # Giyilebilir kontrol eldiveni ESP-NOW firmware'i
├── app/                   # Şartname 5.2 Mobil / Kamera Değerlendirme Uygulaması
├── webots/                # Webots R2025a simülasyon dünyası ve kontrolcüleri
│   ├── controllers/       # İnce kontrolcü adaptörleri (self_balancing_robot paketini yükler)
│   └── worlds/            # Webots dünya dosyaları (12x12 m oda, arena, engeller)
├── analysis/              # Telemetri, kontrol analizi ve grafikler
├── tests/                 # Sensör, motor, enkoder ve bağlantı doğrulama testleri
├── logs/                  # Telemetri ve test günlükleri
├── .editorconfig          # Kod biçimlendirme kuralları
├── .gitattributes         # Git satır sonu ve dosya türü kuralları
├── .gitignore             # Git versiyon kontrolü kuralları
├── CHANGELOG.md           # Sürüm ve değişiklik geçmişi
├── AGENTS.md              # Teknik hafıza ve mimari kararlar arşivi
└── README.md              # Proje genel tanıtımı ve hızlı başlangıç kılavuzu
```

---

## 7. Webots Simülasyon Dünyası ve Kontrolcü Mimarisi

- **Dünya Dosyası:** [cdtp_balance_world.wbt](file:///c:/YTU/self-balancing-robot/webots/worlds/cdtp_balance_world.wbt)
- **Çekirdek Paket (`self_balancing_robot`):**
  - Robot & Kontrolör Parametreleri: [config.py](file:///c:/YTU/self-balancing-robot/self_balancing_robot/config.py)
  - MPU-6050 Tamamlayıcı Filtre & Sensör Füzyonu: [sensor_fusion.py](file:///c:/YTU/self-balancing-robot/self_balancing_robot/sensor_fusion.py)
  - Çift Döngülü Kaskad PID & Motor Telafisi: [cascade_controller.py](file:///c:/YTU/self-balancing-robot/self_balancing_robot/cascade_controller.py)
  - Zaman Serisi Telemetri Kaydedici: [telemetry.py](file:///c:/YTU/self-balancing-robot/self_balancing_robot/telemetry.py)
- **Webots Adaptörü:**
  - Ana Koordinatör: [balance_controller.py](file:///c:/YTU/self-balancing-robot/webots/controllers/balance_controller/balance_controller.py)
- **Doğrulama Testleri:** [test_simulation_balance.py](file:///c:/YTU/self-balancing-robot/tests/test_simulation_balance.py) (Statik Denge, Dış Dürtme ve İlaç Kutusu Taşıma), [test_simulation_driving.py](file:///c:/YTU/self-balancing-robot/tests/test_simulation_driving.py) (Sürüş, Frenleme, Engel Aşma ve İlaç Kutusu Güvenliği)

### 7.1. Fiziksel ve Kinematik Modelleme

- **Koordinat Sistemi:** Webots ENU (+X Doğu/Sağ, +Y Kuzey/İleri, +Z Yukarı).
- **Şasi Ebatları:** 135 mm × 95 mm × 5 mm 4 katmanlı Dekota plaka.
- **Tekerlekler:** 85 mm çap (yarıçap R = 0.0425 m), 170 mm iz genişliği, kauçuk zemin sürtünmesi (coulombFriction 1.8).
- **Taşıyıcı Kolonlar:** 4 adet M3 paslanmaz çelik tij (Ø3.2 mm, 122 mm boy).
- **Kütle ve Ağırlık Merkezi:** Toplam kütle 0.86 kg, CoM mil ekseninden 55 mm yukarıda (yerden yaklaşık 97.5 mm).
- **Bileşenler:** Taban katmanında TB6612FNG ve JGB37 motorlar, Katman 2'de 3S 18650 Li-ion batarya, Katman 3'te ESP32 ve MPU-6050, Katman 4'te Aspirin ilaç kutusu kaseti, switch, sigorta ve durum LED'i.

### 7.2. Kaskad Kontrol Kazançları ve Dinamik Davranış

- **Geometrik Oryantasyon Düzeltmesi (Webots Cylinder Ekseni):**
  - Webots Cylinder varsayılan olarak yerel Z eksenindedir.
  - Tekerlekler ve motor gövdeleri Y ekseni etrafında 90 derece döndürülerek (`0 1 0 1.57079632679`) tekerlek mili X eksenine hizalandı ve kauçuk tekerlekler yere dik olarak basar hale getirildi.
  - Taşıyıcı M3 tij kolonları Z ekseninde bırakılarak (`0 0 1 0`) 4 katmanı dikey olarak birbirine bağladı.
- **İç Döngü (Eğim Açısı PID - 100 Hz / dt = 10 ms):**
  - Eğim Açısı: IMU rpy[0] (Roll around X)
  - Eğim Hızı Filtresi: 50 Hz ayrık titremeyi (discrete chatter) engellemek için EMA filtresi (`filtered_rate = 0.70 * filtered_rate + 0.30 * gyro_vals[0]`) uygulandı.
  - `KP_PITCH = 140.0` (Optuna ile optimize edildi)
  - `KD_PITCH = 3.60`
- **Dış Döngü (Anti-Windup Clamped Konum & Hız PI-D - 100 Hz / dt = 10 ms):**
  - **Döngü Frekansı:** Dış döngü de iç döngü ile eşzamanlı olarak her 10 ms'de (100 Hz) hesaplanır.
  - **Clamping Anti-Windup (Koşullu Entegrasyon):**
    - Motor çıkışı azami hız sınırına (`MAX_VELOCITY = 30.0 rad/s`) ulaştığında veya hedef eğim açısı sınıra dayandığında integral hesabı dondurulur (freeze).
    - Konum integrali `MAX_POS_INT = 0.030 rad` (~1.7 derece) sınırıyla sıkı biçimde clamp edilir.
    - Sürüş komutlarında (W/S) integral sıfırlanır ve hız hatası üzerinden hedef eğim açısı (`pitch_target = - clamp(KP_VEL * vel_err, -MAX_PITCH_TARGET, MAX_PITCH_TARGET)`) üretilir.
  - `KP_VEL = 0.045` (Hız hatası hedef eğim kazancı)
  - `KP_POS = 0.20` (Konum hatası kazancı, rad/m)
  - `KI_POS = 0.015` (Konum integrali kazancı, rad/m-s)
  - `MAX_PITCH_TARGET = 0.025 rad` (~1.43 derece azami hedef eğim açısı)
- **Taban Hız İleri Beslemesi (Base Omega Feedforward & Dinamik Frenleme):**
  - **Sürüşten Duruşa Geçişte Öne Devrilme Kök Nedeni:** Robot W tuşuyla ileri giderken tekerleklerin doğrusal hızı (`v > 0`, `omega < 0`) oluşur. W bırakıldığında gövdeyi arkaya yatırmak için motor komutu doğrudan yavaşlatılırsa, tekerlekler yere göre motor-freni yaparak gövdeyi eylemsizlikle daha da öne fırlatır ve robot öne devrilir.
  - **Çözüm (Base Omega Feedforward):** Motor kontrol komutuna robotun anlık çizgisel hızına karşılık gelen dönme hızı (`base_omega = - v / R`) ileri besleme olarak eklendi (`u = base_omega + u_balance`).
  - Sürüş komutu bittiğinde robot ani bir pozisyon çıpası atmak yerine, hız hatasından (`target_velocity - v`) hafif bir frenleme eğimi üretir. Hız 0.05 m/s altına indiğinde robot durduğu yere yumuşakça çıpa atar.
  - W / Yukarı Ok: İleri sürüş (güvenli nominal hız)
  - S / Aşağı Ok: Geri sürüş (güvenli nominal hız)
  - A / Sol Ok: Sola diferansiyel dönüş
  - D / Sağ Ok: Sağa diferansiyel dönüş
  - Boşluk (Space): Frenleme ve anında durduğu yere konum kilidi atma
  - R Tuşu: Robotu başlangıç koordinatlarına ve sıfır açıya sıfırlama (Devrilmiş olsa bile klavye emniyet bloğundan önce okunduğu için fizik sıfırlanabilir).

### 7.3. Gerçekçi Simülasyon Geliştirmeleri ve Fiziksel Dayanıklılık

1. **Dış Dürtme / Bozucu Kuvvet Dayanıklılığı (Disturbance Rejection - P Tuşu):**
   - Webots Supervisor API (`robot_node.addForceWithOffset([0.0, push_force_y, 0.0], [0.0, 0.0, 0.08], True)`) ile robota anlık 4.0 N itme kuvveti uygulanabilir.
   - P tuşuna basıldığında yönü değişen (bir ileri, bir geri) 4.0 N impuls uygulanır; kaskad kontrolcü gerçekte yaklaşık 6.9 derece tepe eğim altında sendelemeyi söndürür ve devrilmeden dik dengeye geri döner.
   - Doğrulama Testi: [test_simulation_balance.py](file:///c:/YTU/self-balancing-robot/tests/test_simulation_balance.py)
2. **Ayrık Dinamik İlaç Kutusu ve Yerel Eksen (Body Frame) Doğrulama (Modül E):**
   - Katman 4 üzerine 80 × 26 mm dekota kaset yuvası sınırları ve çerçevesi eklendi.
   - 25 gram kütleye sahip bağımsız bir Aspirin ilaç kutusu `Solid` nesnesi olarak modellendi; temas sürtünmesi (`ContactProperties: medicine_box - default, coulombFriction 0.5`) tanımlandı.
   - [balance_controller.py](file:///c:/YTU/self-balancing-robot/webots/controllers/balance_controller/balance_controller.py) içinde her adımda robot rotasyon matrisi (`R_inv = R^T`) kullanılarak kutunun bağıl konumu robotun yerel gövde eksenine (`dx_loc, dy_loc, dz_loc`) dönüştürülür. Robot eğilse dahi yanıltıcı hata üretilmez; kutu kaset sınırlarını aşarsa (`abs(dx_loc) > 0.040 m`, `abs(dy_loc) > 0.020 m`, `dz_loc < 0.100 m`) sistem `[HATA] Ilac kutusu hazneden dustu!` basar.
   - Doğrulama Testleri: [test_simulation_driving.py](file:///c:/YTU/self-balancing-robot/tests/test_simulation_driving.py), [test_simulation_balance.py](file:///c:/YTU/self-balancing-robot/tests/test_simulation_balance.py) 
3. **Telemetri ve Gerçek Açı (Ground Truth vs. Kestirim):**
   - Telemetri dosyasında (`simulation_telemetry.csv`) birincil `pitch_deg` sütununda Webots fizik motorunun gerçek gövde eğim açısı (Ground Truth Roll around X) kaydedilir.
   - Tamamlayıcı filtrenin (%98 Gyro + %2 Accel) ürettiği kestirim değeri `pitch_est_deg` sütununda ayrı olarak saklanır; böylece sensör gecikmesi ve dinamik ivmelenme hataları analiz edilebilir.
4. **JGB37 DC Motor Ölü Bölgesi (Deadband) & Sürtünme Telafisi:**
   - 0.60 rad/s altındaki torklarda motorun statik sürtünmeden dolayı dönmediği stiction ölü bölgesi modellendi.
   - Kontrolcüye stiction feedforward ofseti (`compensate_deadband`, comp = 0.50 rad/s) eklenerek motorun eşik altı komut gecikmesi sıfırlandı.
   - Doğrulama Testi: [test_simulation_driving.py](file:///c:/YTU/self-balancing-robot/tests/test_simulation_driving.py)
5. **Zemin Modeli:**
   - Standart düz zemin üzerinde pürüzsüz temas yüzeyi tanımlanmıştır. Zemin kablo kanalı engeli sadeleştirilerek kaldırılmıştır.
