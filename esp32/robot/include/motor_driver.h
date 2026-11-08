#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include "robot_config.h"

/**
 * ================================================================================
 * TB6612FNG MOTOR SURUCUSU (LEDC DONANIMSAL PWM)
 * ================================================================================
 * ESP32 donanımsal LEDC zamanlayıcısını kullanarak 20 kHz ultrasonik PWM uretir.
 * Motorlarda duyulabilir vınlama sesini tamamen keser ve kayıpları asgariye indirir.
 * ================================================================================
 */

class MotorDriver {
public:
    MotorDriver();

    // GPIO ve LEDC PWM kanallarini baslatir
    void begin();

    // Motor PWM ve yon kontrolu (-255 ... +255)
    // Pozitif: Ileri, Negatif: Geri, 0: Dinamik Fren
    void setPWM(int16_t pwm_left, int16_t pwm_right);

    // Motorlari bosta serbest birak (Coast)
    void coast();

    // Motor terminallerini kisa devre ederek kilitli fren yap (Brake)
    void brake();

    // Acil durum durdurmasi: STBY pinini LOW yaparak surucuyu 1 mikrosaniyede kapatir
    void emergencyStop();

    // Surucuyu uyandir (STBY = HIGH)
    void enable();

    bool isEnabled() const { return enabled_; }

private:
    bool enabled_;
    void setMotorL(int16_t speed);
    void setMotorR(int16_t speed);
};

#endif // MOTOR_DRIVER_H

