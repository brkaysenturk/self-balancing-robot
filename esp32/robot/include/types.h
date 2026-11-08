#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/**
 * ================================================================================
 * SISTEM VERI YAPILARI VE DURUM MAKINELERI
 * ================================================================================
 */

// Robotun Calisma Modlari
typedef enum {
    MODE_DISARMED = 0,      // Motorlar kapali, guvenli bekleme
    MODE_BALANCING,         // Oldugu yerde dik denge ve konum kilitleme (Modul A)
    MODE_DRIVING,           // Eldiven veya klavye ile ileri/geri surus (Modul B)
    MODE_TURNING,           // Oldugu yerde saga/sola donus
    MODE_HOMING,            // Otonom 38 kHz IR istasyona yaklasma (Modul C)
    MODE_DOCKED,            // Istasyona kenetlendi, dik durus ve sarj devrede (Modul D)
    MODE_ESTOP              // Devrilme acil durdurma (Egim > 36 derece)
} RobotMode_t;

// Komut Kaynagi
typedef enum {
    CMD_SOURCE_NONE = 0,
    CMD_SOURCE_SERIAL,      // Bilgisayar USB Klavye (W/S/A/D/Space/H)
    CMD_SOURCE_ESPNOW,      // Giyilebilir Kontrol Eldiveni
    CMD_SOURCE_AUTONOMOUS   // Otonom Homing Kontrolcusu
} CommandSource_t;

// Eldiven veya Bilgisayardan Gelen Sürüş Komut Paketi
typedef struct {
    float desired_velocity;     // Hedef dogrusal cizgisel hiz (m/s)
    float desired_turn;         // Hedef acisal donus hizi (rad/s)
    CommandSource_t source;     // Komutun geldigi kaynak
    uint32_t last_cmd_time_ms;  // Zaman asimi emniyet damgasi
    bool estop_requested;       // Kullanici acil stop talebi
    bool homing_requested;      // Otonom yanaşma tetikleyicisi
    bool reset_requested;       // Sifirlama talebi
} TeleopCommand_t;

// MPU-6050 ve Sensor Fuzyonu Ciktisi
typedef struct {
    float pitch_rad;            // Filtrelenmis govde egim acisi (radyan)
    float pitch_deg;            // Filtrelenmis govde egim acisi (derece)
    float pitch_rate_rads;      // EMA filtrelenmis acisal hiz (rad/s)
    float raw_gx_rads;          // Ham jiroskop X acisal hizi (rad/s)
    float raw_ay_mps2;          // Ham ivmeolcer Y ivmesi (m/s^2)
    float raw_az_mps2;          // Ham ivmeolcer Z ivmesi (m/s^2)
    bool imu_healthy;           // I2C haberlesme saglik bayragi
} IMUState_t;

// Tekerlek ve Odometri Durumu
typedef struct {
    int32_t enc_left_ticks;     // Sol enkoder toplam pulse
    int32_t enc_right_ticks;    // Sag enkoder toplam pulse
    float omega_left_rads;      // Sol tekerlek acisal hizi (rad/s)
    float omega_right_rads;     // Sag tekerlek acisal hizi (rad/s)
    float linear_velocity_mps;  // Robot govde cizgisel hizi (m/s)
    float filtered_vel_mps;     // EMA filtrelenmis cizgisel hiz (m/s)
    float linear_position_m;    // Kat edilen toplam mesafe (m)
    float turn_omega_rads;      // Diferansiyel donus hizi (rad/s)
} OdometryState_t;

// Kaskad PID Cikis Durumu
typedef struct {
    float pitch_target_rad;     // Dis dongu hedef yatma acisi (rad)
    float u_balance_volts;      // Ic dongu denge gerilimi (V)
    float u_turn_volts;         // Donus gerilimi (V)
    float v_left_volts;         // Sol motor nihai gerilimi (V)
    float v_right_volts;        // Sag motor nihai gerilimi (V)
    int16_t pwm_left;           // Sol motor 8-bit PWM (-255 ... +255)
    int16_t pwm_right;          // Sag motor 8-bit PWM (-255 ... +255)
    bool is_holding_position;   // Durus modunda konum kilitli mi?
} ControllerOutput_t;

// Otonom IR Istasyon Yanasma Durumu
typedef struct {
    bool left_ir_detected;      // Sol TSOP2238 aktif mi? (LOW = Isik var)
    bool right_ir_detected;     // Sag TSOP2238 aktif mi?
    bool bumper_pressed;        // Fiziksel tampon butonu temas etti mi?
    uint32_t homing_start_ms;   // Yanasma baslama zamani
    bool dock_completed;        // Yanasip durma tamamlandi mi?
} HomingState_t;

#endif // ROBOT_TYPES_H

