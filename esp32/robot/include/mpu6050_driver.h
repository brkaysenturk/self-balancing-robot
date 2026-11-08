#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#include <Arduino.h>
#include <Wire.h>
#include "robot_config.h"
#include "types.h"

/**
 * ================================================================================
 * MPU-6050 DONANIMSAL I2C SURUCUSU
 * ================================================================================
 * Harici kutuphane bagimliligi olmadan 400 kHz Fast-Mode I2C veri yolu uzerinden
 * 14-byte paket okumasi (Burst Read) yapan, 42 Hz dahili DLPF filtreli ve otomatik
 * jiroskop ofset kalibrasyonlu yuksek hizli surucudur.
 * ================================================================================
 */

class MPU6050Driver {
public:
    MPU6050Driver(uint8_t i2c_addr = 0x68);

    // I2C veri yolunu ve MPU-6050 registerlarini baslatir
    bool begin(int sda_pin = PIN_I2C_SDA, int scl_pin = PIN_I2C_SCL, uint32_t frequency = I2C_CLOCK_SPEED_HZ);

    // Duragan halde jiroskop ofsetini kalibre eder
    void calibrateGyro(uint16_t samples = GYRO_CALIBRATION_SAMPLES);

    // 100 Hz cevrimde tek bir I2C isleminde tum eksenleri okur (< 0.4 ms)
    bool readSensor(IMUState_t* imu_state);

    // Ham jiroskop ofset degerleri
    float getGyroXBias() const { return gyro_x_bias_; }

private:
    uint8_t addr_;
    float gyro_x_bias_;
    float gyro_y_bias_;
    float gyro_z_bias_;
    bool is_calibrated_;

    bool writeRegister(uint8_t reg, uint8_t data);
    bool readRegisters(uint8_t reg, uint8_t* buffer, size_t length);
};

#endif // MPU6050_DRIVER_H

