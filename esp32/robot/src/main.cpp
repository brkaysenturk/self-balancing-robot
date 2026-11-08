#include <Arduino.h>
#include "robot_config.h"
#include "types.h"
#include "sensor_fusion.h"
#include "mpu6050_driver.h"
#include "motor_driver.h"
#include "encoder_reader.h"
#include "pid_controller.h"
#include "ir_homing.h"
#include "robot_comm.h"

/**
 * ================================================================================
 * IKI TEKERLEKLI HASTA DESTEK VE ILAC TASIMA DENGE ROBOTU
 * ESP32 GERCEK ZAMANLI FREERTOS FIRMWARE ANA GIRIS NOKTASI (MAIN)
 * ================================================================================
 * Cift Cekirdek Gorev Dagilimi:
 * - CORE 1 (100 Hz / 10 ms Katı Deterministik Real-Time):
 *     1. MPU-6050 Burst I2C Okuma
 *     2. %98 Gyro + %2 Accel Tamamlayici Filtre Sensor Fuzyonu
 *     3. Manyetik Enkoder Odometri Hesabi
 *     4. 36.0 Derece Devrilme E-Stop Denetimi
 *     5. Kaskad Gerilim/PWM PID, Anti-Windup ve Ters EMK Ileri Beslemesi
 *     6. TB6612FNG 20 kHz Donanimsal LEDC PWM Surus
 *
 * - CORE 0 (20 Hz / 50 ms Iletisim & Telemetri):
 *     1. USB Seri Port Klavye Komutlari (W/S/A/D/Space/H/R/E)
 *     2. ESP-NOW Kablosuz Eldiven Paket Alimi
 *     3. Durum LED'i Denetimi ve Seri Telemetri Yazimi
 * ================================================================================
 */

// Sistem Nesneleri
SensorFusion          fusion;
MPU6050Driver         imu;
MotorDriver           motor_driver;
EncoderReader         encoder_reader;
PIDController         pid_controller;
IRHomingController    ir_homing;
RobotComm             robot_comm;

// Global Sistem Durum Değişkenleri
volatile RobotMode_t  g_current_mode = MODE_DISARMED;
IMUState_t            g_imu_state;
OdometryState_t       g_odo_state;
ControllerOutput_t    g_ctrl_output;
TeleopCommand_t       g_teleop_cmd;

// FreeRTOS Task Kolları
TaskHandle_t xControlTaskHandle = NULL;
TaskHandle_t xTeleopTaskHandle  = NULL;

// --------------------------------------------------------------------------------
// GÖREV 1 (CORE 1): 100 Hz Katı Deterministik Kontrol ve Denge Döngüsü
// --------------------------------------------------------------------------------
void controlTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_PERIOD_MS); // 10 ms

    Serial.printf("[FREERTOS] 100 Hz Kontrol Task'i Core %d uzerinde baslatildi.\n", xPortGetCoreID());

    while (true) {
        // Tam 10 ms (100 Hz) periyodik zamanlama
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        const float dt = CONTROL_DT_SEC; // 0.01 s

        // 1. MPU-6050 I2C Burst Okuma (< 0.4 ms)
        if (imu.readSensor(&g_imu_state)) {
            // 2. Tamamlayıcı Filtre & Jiroskop EMA Türev Filtresi
            fusion.update(g_imu_state.raw_gx_rads,
                          g_imu_state.raw_ay_mps2,
                          g_imu_state.raw_az_mps2,
                          dt);

            g_imu_state.pitch_rad = fusion.getPitchRad();
            g_imu_state.pitch_deg = fusion.getPitchDeg();
            g_imu_state.pitch_rate_rads = fusion.getFilteredRateRadS();
        }

        // 3. Manyetik Enkoder Odometri Güncellemesi
        encoder_reader.updateOdometry(dt, &g_odo_state);

        // 4. Sıfırlama Talebi (Klavye 'R' veya Eldiven Reset)
        if (robot_comm.isResetRequested()) {
            fusion.reset();
            pid_controller.reset();
            encoder_reader.reset();
            ir_homing.stopHoming();
            motor_driver.enable();
            g_current_mode = MODE_BALANCING;
            robot_comm.clearResetRequest();
            Serial.println("[RESET] Robot konumu ve tum integraller sifirlandi.");
            continue;
        }

        // 5. Acil Devrilme E-Stop Denetimi (|θ| > 36.0 Derece)
        if (fabsf(g_imu_state.pitch_deg) > FALL_LIMIT_DEG || robot_comm.isEstopToggled()) {
            motor_driver.emergencyStop();
            g_current_mode = MODE_ESTOP;
            continue;
        }

        // 6. Otonom 38 kHz IR İstasyon Yanaşma Talebi (Klavye 'H' veya Eldiven)
        if (robot_comm.isHomingRequested()) {
            ir_homing.startHoming();
            robot_comm.clearHomingRequest();
        }

        float desired_v = 0.0f;
        float desired_turn = 0.0f;
        float target_v = 0.0f;
        float target_turn = 0.0f;

        // 7. Mod Belirleme ve Hız/Dönüş Referansı Üretimi
        if (ir_homing.isHomingActive()) {
            // Otonom Homing Modu
            ir_homing.update(&desired_v, &desired_turn);
            target_v = desired_v;
            target_turn = desired_turn;

            if (ir_homing.isDocked()) {
                g_current_mode = MODE_DOCKED;
            } else {
                g_current_mode = MODE_HOMING;
            }
        } else {
            // Manuel Teleoperasyon Modu (Klavye veya Eldiven)
            robot_comm.getSmoothedCommand(dt, &target_v, &target_turn, &g_teleop_cmd);
            desired_v = g_teleop_cmd.desired_velocity;
            desired_turn = g_teleop_cmd.desired_turn;

            if (fabsf(target_v) > 0.02f) {
                g_current_mode = MODE_DRIVING;
            } else if (fabsf(target_turn) > 0.05f) {
                g_current_mode = MODE_TURNING;
            } else {
                g_current_mode = MODE_BALANCING;
            }
        }

        // 8. Kaskad PID, Anti-Windup Clamping, Sürtünme ve Ters EMK İleri Beslemesi
        pid_controller.compute(g_imu_state.pitch_rad,
                              g_imu_state.pitch_rate_rads,
                              target_v,
                              desired_v,
                              target_turn,
                              g_odo_state,
                              dt,
                              &g_ctrl_output);

        // 9. TB6612FNG 20 kHz Donanımsal PWM Motor Sürüşü
        motor_driver.setPWM(g_ctrl_output.pwm_left, g_ctrl_output.pwm_right);
    }
}

// --------------------------------------------------------------------------------
// GÖREV 2 (CORE 0): 20 Hz İletişim, Telemetri ve Gösterge Döngüsü
// --------------------------------------------------------------------------------
void teleopTask(void* parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TELEOP_PERIOD_MS); // 50 ms
    uint32_t led_timer_ms = 0;
    bool led_state = false;

    Serial.printf("[FREERTOS] 20 Hz Teleop/Iletisim Task'i Core %d uzerinde baslatildi.\n", xPortGetCoreID());

    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 1. Seri Port ve ESP-NOW Komutlarını Oku
        robot_comm.update();

        // 2. Telemetriyi USB Seri Portuna Yazdır
        robot_comm.printTelemetry(g_imu_state, g_odo_state, g_ctrl_output, g_current_mode);

        // 3. Katman 4 Durum / Şarj LED'i Davranışı
        uint32_t now = millis();
        if (g_current_mode == MODE_DOCKED) {
            // Şarj İstasyonuna Kenetlendi: LED Sabit Yanar (Modül D)
            digitalWrite(PIN_STATUS_LED, HIGH);
        } else if (g_current_mode == MODE_ESTOP) {
            // Devrilme / Acil Stop: Hızlı Yanıp Söner (100 ms)
            if (now - led_timer_ms > 100) {
                led_timer_ms = now;
                led_state = !led_state;
                digitalWrite(PIN_STATUS_LED, led_state ? HIGH : LOW);
            }
        } else {
            // Normal Denge / Sürüş: Yavaş Kalp Atışı (1 Hz / 500 ms)
            if (now - led_timer_ms > 500) {
                led_timer_ms = now;
                led_state = !led_state;
                digitalWrite(PIN_STATUS_LED, led_state ? HIGH : LOW);
            }
        }
    }
}

// --------------------------------------------------------------------------------
// ARDUINO SETUP
// --------------------------------------------------------------------------------
void setup() {
    // 1. Teleoperasyon ve USB Seri Port Başlatma
    robot_comm.begin(SERIAL_BAUD_RATE);
    delay(100);

    // 2. TB6612FNG Motor Sürücü Başlatma (Güvenlik için başlangıçta pasif)
    motor_driver.begin();

    // 3. Manyetik Enkoder Kesmelerini Başlatma
    encoder_reader.begin();

    // 4. MPU-6050 Donanımsal I2C ve Sensör Başlatma
    if (!imu.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_SPEED_HZ)) {
        Serial.println("[HATA] MPU-6050 I2C baslatilamadi! Kablolari kontrol edin.");
        while (true) {
            digitalWrite(PIN_STATUS_LED, HIGH);
            delay(100);
            digitalWrite(PIN_STATUS_LED, LOW);
            delay(100);
        }
    }

    // 5. Jiroskop Ofset Kalibrasyonu (Robot masa üzerinde dik tutulmalı)
    imu.calibrateGyro(GYRO_CALIBRATION_SAMPLES);

    // 6. IR Homing ve Tampon Switch Başlatma
    ir_homing.begin();

    // 7. Kontrolcü ve Filtre Durumlarını Sıfırlama
    fusion.reset();
    pid_controller.reset();
    g_current_mode = MODE_BALANCING;

    // 8. FreeRTOS Çoklu Çekirdek Görevlerini Başlatma
    // Kontrol Task'ı -> Core 1 (En yüksek öncelik)
    xTaskCreatePinnedToCore(
        controlTask,
        "ControlTask_100Hz",
        8192,
        NULL,
        CONTROL_TASK_PRIORITY,
        &xControlTaskHandle,
        CONTROL_TASK_CORE
    );

    // Telemetri/İletişim Task'ı -> Core 0
    xTaskCreatePinnedToCore(
        teleopTask,
        "TeleopTask_20Hz",
        4096,
        NULL,
        TELEOP_TASK_PRIORITY,
        &xTeleopTaskHandle,
        TELEOP_TASK_CORE
    );

    Serial.println("\n[SISTEM] Denge Robotu Firmware Basariyla Hazir!");
    Serial.println("[SISTEM] Bilgisayardan test etmek icin W/S/A/D/Space tuslarini kullanabilirsiniz.\n");
}

void loop() {
    // FreeRTOS görevleri arka planda çalıştığı için loop() boş bırakılır
    vTaskDelete(NULL);
}

