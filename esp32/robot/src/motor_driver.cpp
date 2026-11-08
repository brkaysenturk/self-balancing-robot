#include "motor_driver.h"

MotorDriver::MotorDriver() : enabled_(false) {
}

void MotorDriver::begin() {
    // Yon pinleri cıkıs olarak ayarlanır
    pinMode(PIN_TB6612_STBY, OUTPUT);
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN1, OUTPUT);
    pinMode(PIN_MOTOR_R_IN2, OUTPUT);

    // Baslangicta surucu guvenlik icin pasif (STBY = LOW)
    digitalWrite(PIN_TB6612_STBY, LOW);
    digitalWrite(PIN_MOTOR_L_IN1, LOW);
    digitalWrite(PIN_MOTOR_L_IN2, LOW);
    digitalWrite(PIN_MOTOR_R_IN1, LOW);
    digitalWrite(PIN_MOTOR_R_IN2, LOW);

    // ESP32 LEDC Donanımsal PWM Ayarları (20 kHz, 8-bit çözünürlük)
    ledcSetup(PWM_CHANNEL_L, PWM_FREQUENCY_HZ, PWM_TIMER_BITS);
    ledcAttachPin(PIN_MOTOR_L_PWM, PWM_CHANNEL_L);
    ledcWrite(PWM_CHANNEL_L, 0);

    ledcSetup(PWM_CHANNEL_R, PWM_FREQUENCY_HZ, PWM_TIMER_BITS);
    ledcAttachPin(PIN_MOTOR_R_PWM, PWM_CHANNEL_R);
    ledcWrite(PWM_CHANNEL_R, 0);

    enable();
    Serial.println("[TB6612] Motor surucu baslatildi (20 kHz PWM, STBY aktif)");
}

void MotorDriver::enable() {
    digitalWrite(PIN_TB6612_STBY, HIGH);
    enabled_ = true;
}

void MotorDriver::emergencyStop() {
    digitalWrite(PIN_TB6612_STBY, LOW);
    ledcWrite(PWM_CHANNEL_L, 0);
    ledcWrite(PWM_CHANNEL_R, 0);
    digitalWrite(PIN_MOTOR_L_IN1, LOW);
    digitalWrite(PIN_MOTOR_L_IN2, LOW);
    digitalWrite(PIN_MOTOR_R_IN1, LOW);
    digitalWrite(PIN_MOTOR_R_IN2, LOW);
    enabled_ = false;
}

void MotorDriver::coast() {
    ledcWrite(PWM_CHANNEL_L, 0);
    ledcWrite(PWM_CHANNEL_R, 0);
    digitalWrite(PIN_MOTOR_L_IN1, LOW);
    digitalWrite(PIN_MOTOR_L_IN2, LOW);
    digitalWrite(PIN_MOTOR_R_IN1, LOW);
    digitalWrite(PIN_MOTOR_R_IN2, LOW);
}

void MotorDriver::brake() {
    ledcWrite(PWM_CHANNEL_L, 255);
    ledcWrite(PWM_CHANNEL_R, 255);
    digitalWrite(PIN_MOTOR_L_IN1, HIGH);
    digitalWrite(PIN_MOTOR_L_IN2, HIGH);
    digitalWrite(PIN_MOTOR_R_IN1, HIGH);
    digitalWrite(PIN_MOTOR_R_IN2, HIGH);
}

void MotorDriver::setPWM(int16_t pwm_left, int16_t pwm_right) {
    if (!enabled_) {
        enable();
    }
    setMotorL(pwm_left);
    setMotorR(pwm_right);
}

void MotorDriver::setMotorL(int16_t speed) {
    if (speed > MAX_PWM_VALUE) speed = MAX_PWM_VALUE;
    if (speed < -MAX_PWM_VALUE) speed = -MAX_PWM_VALUE;

    if (speed > 0) {
        // İleri
        digitalWrite(PIN_MOTOR_L_IN1, HIGH);
        digitalWrite(PIN_MOTOR_L_IN2, LOW);
        ledcWrite(PWM_CHANNEL_L, (uint32_t)speed);
    } else if (speed < 0) {
        // Geri
        digitalWrite(PIN_MOTOR_L_IN1, LOW);
        digitalWrite(PIN_MOTOR_L_IN2, HIGH);
        ledcWrite(PWM_CHANNEL_L, (uint32_t)(-speed));
    } else {
        // Dinamik Fren
        digitalWrite(PIN_MOTOR_L_IN1, HIGH);
        digitalWrite(PIN_MOTOR_L_IN2, HIGH);
        ledcWrite(PWM_CHANNEL_L, 0);
    }
}

void MotorDriver::setMotorR(int16_t speed) {
    if (speed > MAX_PWM_VALUE) speed = MAX_PWM_VALUE;
    if (speed < -MAX_PWM_VALUE) speed = -MAX_PWM_VALUE;

    if (speed > 0) {
        // İleri
        digitalWrite(PIN_MOTOR_R_IN1, HIGH);
        digitalWrite(PIN_MOTOR_R_IN2, LOW);
        ledcWrite(PWM_CHANNEL_R, (uint32_t)speed);
    } else if (speed < 0) {
        // Geri
        digitalWrite(PIN_MOTOR_R_IN1, LOW);
        digitalWrite(PIN_MOTOR_R_IN2, HIGH);
        ledcWrite(PWM_CHANNEL_R, (uint32_t)(-speed));
    } else {
        // Dinamik Fren
        digitalWrite(PIN_MOTOR_R_IN1, HIGH);
        digitalWrite(PIN_MOTOR_R_IN2, HIGH);
        ledcWrite(PWM_CHANNEL_R, 0);
    }
}

