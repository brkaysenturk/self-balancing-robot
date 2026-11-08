#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

/**
 * ================================================================================
 * IKI TEKERLEKLI ILAC TASIMA DENGE ROBOTU
 * MERKEZI KONFIGURASYON VE DONANIM HARITASI (SINGLE SOURCE OF TRUTH - SSOT)
 * ================================================================================
 * Bu modul; Python/Webots tarafindaki self_balancing_robot/config.py dosyasiyla
 * birebir esdeger olup, tum fiziksel parametreleri, PID kazanclarini, zamanlama
 * periyotlarini ve ESP32 DevKit V1 (38-pin shield) pin atamalarini tek bir
 * merkezde toplar.
 * ================================================================================
 */

// ================================================================================
// 1. MEKANIK VE GEOMETRIK PARAMETRELER
// ================================================================================
#define WHEEL_RADIUS_M          0.0425f     // 85 mm tekerlek capi / 2 = 0.0425 m
#define TRACK_WIDTH_M           0.1900f     // 150 mm sasi + 35 mm tekerlek + 2x 2.5 mm aciklik (m)
#define WHEEL_CIRCUMFERENCE_M   0.267035f   // 2 * PI * R (m)
#define FALL_LIMIT_DEG          36.0f       // E-Stop acil durum devrilme kesici esigi (derece)
#define DRIVE_SPEED_MPS         0.20f       // Guvenli nominal ileri/geri surus hizi (m/s)
#define TURN_SPEED_RADPS        1.20f       // Nominal diferansiyel donus hiz farki (~31 derece/saniye)

// ================================================================================
// 2. KASKAD PID VE PI DONUS KONTROLCU KAZANCLARI (WEBOTS OPTUNA TPE ILE OPTIMIZE)
// ================================================================================
// Ic Dongu: Govde Acisi PD Gerilim Kontrolu (100 Hz)
#define KP_VOLT                 62.7795f    // Aci kazanci (V/rad) ~1.10 V/derece
#define KD_VOLT                 1.0329f     // Acisal hiz sonumleme kazanci (V/(rad/s))

// Dis Dongu: Hiz ve Konum Kaskad Eğim Hedefi Uretici (100 Hz)
#define KP_POS                  0.2730f     // Durusta konumdan hedef egim kazanci (rad/m)
#define KI_POS                  0.0051f     // Durusta konum integral kazanci (rad/(m*s))
#define KV_VEL                  0.2892f     // Durusta hiz sonumleme kazanci (rad/(m/s))
#define KP_VEL_DRIVE            0.2333f     // Surus esnasinda hiz hatasi egim kazanci (rad/(m/s))

// Sinirlayicilar ve Anti-Windup Esikleri
#define MAX_PITCH_TARGET_RAD    0.0250f     // Dis dongu azami hedef egim acisi (~1.43 derece)
#define MAX_POS_INT             0.0300f     // Clamping konum integral doyumu (m*s)
#define HOLD_CMD_THRESHOLD_MPS  0.0100f     // Konum kilitleme komut hiz esigi (m/s)
#define HOLD_VEL_THRESHOLD_MPS  0.0200f     // Konum kilitleme gercek hiz esigi (m/s)
#define ODOMETRY_VEL_ALPHA      0.1000f     // Tekerlek hiz kestirimi EMA filtre katsayisi
#define STICTION_TANH_GAIN      3.0000f     // Kalkis surtunme telafisi tanh egimi
#define BACK_EMF_FF_RATIO       0.8500f     // Tekerlek basina ters EMK ileri besleme orani (%85)

// Enkoder Geri Beslemeli Diferansiyel PI Donus Kontrolu (Denge Oncelikli)
#define KP_TURN                 0.4500f     // Donus hiz hatasi kazanci (V/(rad/s))
#define KI_TURN                 2.5000f     // Donus integral kazanci (V/rad)
#define MAX_TURN_VOLTS          3.0000f     // Donus icin ayrilan tavan gerilim (V)
#define TURN_INTEGRAL_LIMIT     2.2000f     // Donus anti-windup tavan gerilimi (V)
#define TURN_FRICTION_TANH_GAIN 6.0000f     // Donus Coulomb surtunme tanh katsayisi
#define TURN_DAMPING_KE_RATIO   0.1500f     // Donus viskoz/EMK sonumleme orani

// Komut Rampasi ve Yumusatma
#define CMD_VEL_ALPHA           0.0400f     // Dogrusal hiz komut rampasi EMA katsayisi
#define CMD_TURN_ALPHA          0.1000f     // Donus hizi komut rampasi EMA katsayisi

// ================================================================================
// 3. JGB37 DC MOTOR VE TB6612FNG ELEKTROMEKANIK PARAMETRELERI
// ================================================================================
#define V_BATTERY_NOMINAL       12.0f       // 3S 18650 Li-ion nominal gerilim (V)
#define R_ARMATURE              4.20f       // JGB37 motor sargi direnci (Ohm)
#define DRIVER_RESISTANCE       0.50f       // TB6612FNG MOSFET iletim ic direnci (Ohm)
#define GEAR_RATIO              30.0f       // 30:1 metal redüktör oranı
#define KT_MOTOR                0.012f      // Motor tork sabiti (N*m/A)
#define KE_MOTOR                0.012f      // Motor ters EMK sabiti (V/(rad/s))
#define GEAR_EFFICIENCY         0.85f       // Reduktor mekanik verimi (%85)
#define KT_WHEEL                (KT_MOTOR * GEAR_RATIO * GEAR_EFFICIENCY) // 0.306 N*m/A
#define KE_WHEEL                (KE_MOTOR * GEAR_RATIO)                   // 0.360 V/(rad/s)
#define COULOMB_FRICTION_NM     0.035f      // Mil statik surtunme torku (N*m)
#define VISCOUS_FRICTION_NMS    0.0015f     // Mil viskoz surtunme katsayisi (N*m*s/rad)
#define DEADBAND_VOLTS          0.60f       // Motor baslama gerilim olu bolgesi (V)
#define MAX_PWM_VALUE           255         // 8-bit tam olcek PWM degeri

// ================================================================================
// 4. MPU-6050 SENSOR FUZYONU VE FILTRE PARAMETRELERI
// ================================================================================
#define FUSION_ALPHA            0.9800f     // Tamamlayici filtre jiroskop agirligi (%98 Gyro, %2 Accel)
#define RATE_EMA_ALPHA          0.3000f     // Jiroskop acisal hiz EMA alcak geciren filtre katsayisi
#define MIN_ACCEL_Z_G           0.0100f     // atan2 tekillik emniyet esigi
#define GYRO_CALIBRATION_SAMPLES 300        // Baslangic jiroskop ofset kalibrasyon ornek sayisi

// ================================================================================
// 5. MODUL C: OTONOM 38 KHZ IR YANASMA (HOMING) PARAMETRELERI
// ================================================================================
#define HOMING_CRUISE_SPEED_MPS 0.16f       // Uzak mesafe yaklasma hizi (m/s)
#define HOMING_CREEP_SPEED_MPS  0.08f       // Yakin mesafe (<30 cm) hassas yanasma hizi (m/s)
#define HOMING_SEARCH_TURN_RATE 0.75f       // IR isigi gorulmediginde tarama donus hizi (rad/s)
#define HOMING_KP_IR_TURN       2.20f       // Diferansiyel IR sinyali donus kazanci (rad/s)
#define HOMING_MAX_TURN_RATE    0.90f       // Yanasma sirasinda azami donus siniri (rad/s)
#define HOMING_DOCK_TIMEOUT_MS  12000       // Otonom yanasma azami guvenlik zaman asimi (ms)

// ================================================================================
// 6. DONANIM VE PIN TANIMLARI (ESP32 DevKit V1 - 38-Pin Vidalı Terminal Shield)
// ================================================================================
// I2C Veri Yolu (MPU-6050 - Katman 3 Geometrik Merkez X=0, Y=0)
#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             22
#define I2C_CLOCK_SPEED_HZ      400000      // 400 kHz Fast-Mode I2C

// TB6612FNG Motor Surucu (Katman 1 Taban - 3 cm Kisa Hatlar)
#define PIN_TB6612_STBY         4           // Surucu Standby (HIGH = Aktif, LOW = Torku Kes)
#define PIN_MOTOR_L_PWM         25          // Sol Motor LEDC PWM (Kanal 0)
#define PIN_MOTOR_L_IN1         26          // Sol Motor Yon Girişi 1
#define PIN_MOTOR_L_IN2         27          // Sol Motor Yon Girişi 2
#define PIN_MOTOR_R_PWM         14          // Sag Motor LEDC PWM (Kanal 1)
#define PIN_MOTOR_R_IN1         16          // Sag Motor Yon Girişi 1 (RX2 pini)
#define PIN_MOTOR_R_IN2         17          // Sag Motor Yon Girişi 2 (TX2 pini)
#define PWM_FREQUENCY_HZ        20000       // 20 kHz Isitilmez Ultrasonik PWM
#define PWM_TIMER_BITS          8           // 8-bit cozunurluk (0 - 255)
#define PWM_CHANNEL_L           0
#define PWM_CHANNEL_R           1

// JGB37 Manyetik Enkoder Fazlari
#define PIN_ENC_L_A             34          // Sol Enkoder Faz A (Kesme pini)
#define PIN_ENC_L_B             35          // Sol Enkoder Faz B
#define PIN_ENC_R_A             36          // Sag Enkoder Faz A (VP pini, Kesme)
#define PIN_ENC_R_B             39          // Sag Enkoder Faz B (VN pini)
#define ENCODER_PPR             330.0f      // Reduktor cikis milinde tur basina pulse (11 * 30:1)

// Algilayicilar ve Gostergeler
#define PIN_TSOP_LEFT           32          // Sol TSOP2238 IR Alicisi (Aktif LOW)
#define PIN_TSOP_RIGHT          33          // Sag TSOP2238 IR Alicisi (Aktif LOW)
#define PIN_DOCK_BUMPER         13          // Fiziksel Tampon / Istasyon Dayanak Butonu
#define PIN_STATUS_LED          2           // Katman 4 Durum / Sarj LED'i (Dahili LED uyumlu)

// ================================================================================
// 7. GERCEK ZAMANLI ZAMANLAMA VE FREERTOS AYARLARI
// ================================================================================
#define CONTROL_LOOP_HZ         100         // 100 Hz Kontrol Dongusu (dt = 10 ms)
#define CONTROL_PERIOD_MS       (1000 / CONTROL_LOOP_HZ)
#define CONTROL_DT_SEC          (1.0f / (float)CONTROL_LOOP_HZ)
#define CONTROL_TASK_CORE       1           // Core 1: Katı Deterministik Kontrol Task'ı
#define CONTROL_TASK_PRIORITY   (configMAX_PRIORITIES - 1)

#define TELEOP_LOOP_HZ          20          // 20 Hz Telemetri & Iletisim Dongusu (dt = 50 ms)
#define TELEOP_PERIOD_MS        (1000 / TELEOP_LOOP_HZ)
#define TELEOP_TASK_CORE        0           // Core 0: Wi-Fi, ESP-NOW ve Seri Iletisim Task'ı
#define TELEOP_TASK_PRIORITY    1

#define SERIAL_BAUD_RATE        115200      // USB Seri Port Hizlandirilmis Hizi

#endif // ROBOT_CONFIG_H

