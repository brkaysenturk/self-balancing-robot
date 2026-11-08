# ESP32 Gerçek Zamanlı Denge Robotu Firmware'i

Bu dizin, Yıldız Teknik Üniversitesi ÇDTP İki Tekerlekli Hasta Destek ve İlaç Taşıma Denge Robotunun **ESP32 DevKit V1** üzerinde çalışan, FreeRTOS tabanlı gerçek zamanlı gömülü kontrol yazılımını içerir.

Simülasyonda ([Webots R2025a](file:///c:/YTU/self-balancing-robot/webots/worlds/webots_self_balancing_robot.wbt)) doğrulanan kaskad PID, tamamlayıcı filtre, motor ters EMK ileri beslemesi ve 38 kHz IR yanaşma algoritmaları bu yazılımda birebir C/C++ donanım koduna dönüştürülmüştür.

---

## 1. Mimari ve FreeRTOS Çift Çekirdek Dağılımı

ESP32'nin çift çekirdekli (Dual-Core Xtensa LX6) mimarisi tam kapasiteyle kullanılır:

```text
       ESP32 CORE 1 (En Yüksek Öncelik - 100 Hz / 10 ms Katı Deterministik)
  +-------------------------------------------------------------------------+
  |  1. MPU-6050 I2C Burst Okuma (400 kHz, < 0.4 ms)                        |
  |  2. Tamamlayıcı Filtre (%98 Gyro + %2 Accel) ve Jiroskop EMA Filtresi   |
  |  3. Enkoder Odometrisi (Hız ve Konum Kestirimi)                         |
  |  4. 36.0 Derece Devrilme E-Stop Denetimi                                |
  |  5. Kaskad Gerilim/PWM PID (Dış Konum Döngüsü + İç Açı Döngüsü)         |
  |  6. Ters EMK İleri Beslemesi (%85) ve Kalkış Sürtünme Telafisi           |
  |  7. TB6612FNG 20 kHz Donanımsal LEDC PWM Çıkışı                         |
  +-------------------------------------------------------------------------+

       ESP32 CORE 0 (Düşük Öncelik - 20 Hz / 50 ms İletişim ve Telemetri)
  +-------------------------------------------------------------------------+
  |  1. USB Seri Port Klavye Dinleyici (W/S/A/D/Space/H/R/E)                 |
  |  2. ESP-NOW Kablosuz Eldiven Paket Alıcısı (1-2 ms Gecikme)             |
  |  3. Katman 4 Durum / Şarj LED'i Yönetimi                                 |
  |  4. Seri Telemetri Yazımı (Metin veya Arduino Serial Plotter CSV)        |
  +-------------------------------------------------------------------------+
```

---

## 2. Dizin Yapısı ve Modüller

```text
esp32/robot/
├── include/
│   ├── robot_config.h        # SSOT: Pinler, PID kazançları, mekanik ve süre sabitleri
│   ├── types.h               # Durum makineleri, komut, telemetri ve sensör yapıları
│   ├── sensor_fusion.h       # Deterministik tamamlayıcı filtre başlığı
│   ├── mpu6050_driver.h      # 400 kHz donanımsal I2C MPU-6050 sürücüsü
│   ├── motor_driver.h        # TB6612FNG 20 kHz LEDC PWM motor sürücüsü
│   ├── encoder_reader.h      # JGB37 manyetik enkoder kesme ve odometri başlığı
│   ├── pid_controller.h      # Kaskad PID, anti-windup ve PI dönüş kontrolcüsü
│   ├── ir_homing.h           # Modül C: 38 kHz TSOP2238 IR istasyon yanaşma başlığı
│   └── robot_comm.h          # Seri port klavye ve ESP-NOW birleşik komut köprüsü
├── src/
│   ├── sensor_fusion.cpp     # Tamamlayıcı filtre ve açısal hız EMA hesabı
│   ├── mpu6050_driver.cpp    # 14-byte burst read ve jiroskop kalibrasyonu
│   ├── motor_driver.cpp      # TB6612FNG yön ve LEDC PWM üretimi
│   ├── encoder_reader.cpp    # IRAM kesmeleri ve tekerlek hız entegrasyonu
│   ├── pid_controller.cpp    # Gerilim kaskad PID, ölü bölge ve ters EMK formülleri
│   ├── ir_homing.cpp         # Diferansiyel IR nirengisi ve kenetlenme mantığı
│   ├── robot_comm.cpp        # Komut rampalama, zaman aşımı ve telemetri formatlayıcı
│   └── main.cpp              # FreeRTOS görevleri ve setup() başlatıcısı
├── platformio.ini            # PlatformIO derleme ve yükleme yapılandırması
├── robot.ino                 # Arduino IDE doğrudan derleme giriş dosyası
└── README.md                 # Bu kılavuz ve test dokümanı
```

---

## 3. Donanım ve Pin Bağlantı Haritası (ESP32 38-Pin Vidalı Shield)

| Modül / Parça | ESP32 Pini | Pin Türü | İşlevi |
| :--- | :--- | :--- | :--- |
| **MPU-6050 SDA** | GPIO 21 | I2C Data | 400 kHz Fast-Mode I2C veri hattı |
| **MPU-6050 SCL** | GPIO 22 | I2C Clock | 400 kHz Fast-Mode I2C saat hattı |
| **TB6612 STBY** | GPIO 4 | Çıkış | Sürücü aktif (HIGH) / Acil kapatma (LOW) |
| **Sol Motor PWM** | GPIO 25 | LEDC PWM | Sol tekerlek 20 kHz donanımsal PWM (Kanal 0) |
| **Sol Motor AIN1**| GPIO 26 | Çıkış | Sol motor yön pini 1 |
| **Sol Motor AIN2**| GPIO 27 | Çıkış | Sol motor yön pini 2 |
| **Sağ Motor PWM** | GPIO 14 | LEDC PWM | Sağ tekerlek 20 kHz donanımsal PWM (Kanal 1) |
| **Sağ Motor BIN1**| GPIO 16 | Çıkış | Sağ motor yön pini 1 (RX2) |
| **Sağ Motor BIN2**| GPIO 17 | Çıkış | Sağ motor yön pini 2 (TX2) |
| **Sol Enkoder A** | GPIO 34 | Giriş / Kesme | Sol tekerlek Faz A (Yükselen kenar kesmesi) |
| **Sol Enkoder B** | GPIO 35 | Giriş | Sol tekerlek Faz B (Yön tayini) |
| **Sağ Enkoder A** | GPIO 36 | Giriş / Kesme | Sağ tekerlek Faz A (VP pini, kesme) |
| **Sağ Enkoder B** | GPIO 39 | Giriş | Sağ tekerlek Faz B (VN pini) |
| **Sol TSOP2238**  | GPIO 32 | Giriş (Pullup)| Sol 38 kHz IR alıcı (Aktif LOW) |
| **Sağ TSOP2238**  | GPIO 33 | Giriş (Pullup)| Sağ 38 kHz IR alıcı (Aktif LOW) |
| **Tampon Butonu** | GPIO 13 | Giriş (Pullup)| İstasyon dayanağı temas mikroswitchi |
| **Durum LED'i**   | GPIO 2 | Çıkış | Katman 4 şarj / durum gösterge LED'i |

---

## 4. Şasisiz Masaüstü (Bench) Test Kılavuzu

Fiziksel gövde henüz toplanmamış olsa dahi sistemin yüzde 80'i masada doğrulanabilir:

### Test 1: MPU-6050 ve Eğim Filtresi Doğrulaması
1. ESP32'yi USB ile bilgisayara bağlayın.
2. MPU-6050'yi breadboard üzerinde GPIO 21 (SDA) ve GPIO 22 (SCL) pinlerine bağlayın.
3. Seri port monitörünü (115200 baud) açın.
4. Başlangıçta robotun jiroskop ofsetini kalibre ettiğini görün (`GYRO_CALIBRATION_SAMPLES: 300`).
5. Sensörü elinizle öne ve arkaya eğdiğinizde seri ekrandaki `Egim: +X.XX deg` değerinin gecikmesiz ve gürültüsüz aktığını doğrulayın.
6. `T` tuşuna basarak Arduino Serial Plotter formatına geçip eğim grafiğini görsel olarak izleyin.

### Test 2: 36.0 Derece Devrilme E-Stop Testi
1. MPU-6050'yi elinizle 36 dereceden fazla yatırın.
2. Seri ekranda modun derhal `ESTOP_FALL` durumuna geçtiğini ve durum LED'inin hızlıca (5 Hz) yanıp söndüğünü görün.
3. `R` tuşuna basarak sistemi normale sıfırlayın.

### Test 3: Bilgisayar Klavye Komut Köprüsü Testi
Eldivene ihtiyaç duymadan USB seri terminal üzerinden aşağıdaki tuşlarla sürüşü doğrulayın:
- `W`: İleri sürüş emri (`+0.20 m/s`). Hedef eğimin öne doğru küçük bir açı ürettiğini (`Hedef: -0.05 rad`) gözlemleyin.
- `S`: Geri sürüş emri (`-0.20 m/s`).
- `A` / `D`: Sola / sağa dönüş emri (`+/- 1.20 rad/s`).
- `Boşluk (Space)`: Dinamik fren ve durulan noktada konum kilitleme.
- `H`: Otonom 38 kHz IR istasyon yanaşma modunu tetikleme.

### Test 4: Motor ve Enkoder Yön Doğrulaması
1. TB6612 motor sürücüsünü ve motorları bağlayın.
2. 12V 3A adaptör ile TB6612 `VM` hattını besleyin.
3. MPU-6050'yi öne eğdiğinizde her iki tekerleğin de ileriye doğru dönerek düşmeyi engellemeye çalıştığını gözünüzle teyit edin.
4. Tekerlekleri elinizle ileri çevirdiğinizde enkoder `Pos` değerinin pozitif yönde arttığını teyit edin.

---

## 5. Derleme ve Yükleme

### Yöntem A: PlatformIO (Önerilen)
```bash
cd esp32/robot
pio run -t upload
pio device monitor -b 115200
```

### Yöntem B: Arduino IDE
1. Arduino IDE ile `esp32/robot/robot.ino` dosyasını açın.
2. Araçlar menüsünden Kart: **DOIT ESP32 DEVKIT V1** seçin.
3. Yükle butonuna basın ve Seri Port Ekranını 115200 baud ile açın.

