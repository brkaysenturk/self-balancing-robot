#include "mpu6050_driver.h"

// MPU-6050 Dahili Register Haritasi
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_ACCEL_XOUT_H    0x3B
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_WHO_AM_I        0x75

#define DEG_TO_RAD_F            0.0174532925f
#define GRAVITY_MSS             9.80665f

MPU6050Driver::MPU6050Driver(uint8_t i2c_addr)
    : addr_(i2c_addr),
      gyro_x_bias_(0.0f),
      gyro_y_bias_(0.0f),
      gyro_z_bias_(0.0f),
      is_calibrated_(false) {
}

bool MPU6050Driver::begin(int sda_pin, int scl_pin, uint32_t frequency) {
    Wire.begin(sda_pin, scl_pin, frequency);
    delay(50);

    // 1. Cihaz ID kontrolu
    uint8_t who_am_i = 0;
    if (!readRegisters(MPU_REG_WHO_AM_I, &who_am_i, 1) || who_am_i != 0x68) {
        Serial.printf("[MPU6050] Hata! Cihaz bulunamadi veya ID gecersiz: 0x%02X (Beklenen: 0x68)\n", who_am_i);
        return false;
    }

    // 2. Uyku modundan cikar (Internal 8MHz osilator veya gyro PLL)
    writeRegister(MPU_REG_PWR_MGMT_1, 0x01); // PLL with X axis gyroscope reference
    delay(10);

    // 3. Dahili Alcak Geciren Filtre (DLPF) Ayari: 42 Hz Bant Genisligi
    // 100 Hz ornekleme icin Nyquist frekansinin (50 Hz) altinda sekerlemeyi (aliasing) engeller
    writeRegister(MPU_REG_CONFIG, 0x03);

    // 4. Jiroskop Olcek Araligi: +/- 250 deg/s (Hassasiyet: 131.0 LSB / (deg/s))
    writeRegister(MPU_REG_GYRO_CONFIG, 0x00);

    // 5. Ivmeolcer Olcek Araligi: +/- 2g (Hassasiyet: 16384.0 LSB / g)
    writeRegister(MPU_REG_ACCEL_CONFIG, 0x00);

    Serial.println("[MPU6050] Basariyla baslatildi (400 kHz, 42 Hz DLPF, +/-250 deg/s, +/-2g)");
    return true;
}

void MPU6050Driver::calibrateGyro(uint16_t samples) {
    Serial.printf("[MPU6050] Jiroskop ofset kalibrasyonu basliyor (%d ornek, robotu sabit tutun)...\n", samples);

    float sum_gx = 0.0f;
    float sum_gy = 0.0f;
    float sum_gz = 0.0f;
    uint8_t buffer[14];

    for (uint16_t i = 0; i < samples; ++i) {
        if (readRegisters(MPU_REG_ACCEL_XOUT_H, buffer, 14)) {
            int16_t raw_gx = (int16_t)((buffer[8] << 8) | buffer[9]);
            int16_t raw_gy = (int16_t)((buffer[10] << 8) | buffer[11]);
            int16_t raw_gz = (int16_t)((buffer[12] << 8) | buffer[13]);

            sum_gx += (float)raw_gx;
            sum_gy += (float)raw_gy;
            sum_gz += (float)raw_gz;
        }
        delay(4);
    }

    gyro_x_bias_ = (sum_gx / (float)samples) / 131.0f * DEG_TO_RAD_F;
    gyro_y_bias_ = (sum_gy / (float)samples) / 131.0f * DEG_TO_RAD_F;
    gyro_z_bias_ = (sum_gz / (float)samples) / 131.0f * DEG_TO_RAD_F;
    is_calibrated_ = true;

    Serial.printf("[MPU6050] Kalibrasyon tamamlandi! X-Ofset: %.4f rad/s (%.2f deg/s)\n",
                  gyro_x_bias_, gyro_x_bias_ * 57.2957795f);
}

bool MPU6050Driver::readSensor(IMUState_t* imu_state) {
    if (!imu_state) return false;

    uint8_t buffer[14];
    if (!readRegisters(MPU_REG_ACCEL_XOUT_H, buffer, 14)) {
        imu_state->imu_healthy = false;
        return false;
    }

    // int16_t donusumleri (Big-endian)
    int16_t raw_ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    int16_t raw_ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    int16_t raw_az = (int16_t)((buffer[4] << 8) | buffer[5]);
    // buffer[6..7] Sicaklik
    int16_t raw_gx = (int16_t)((buffer[8] << 8) | buffer[9]);

    // Ivmeolcer donusumu: +/- 2g olcek (16384 LSB/g) -> m/s^2
    imu_state->raw_ay_mps2 = ((float)raw_ay / 16384.0f) * GRAVITY_MSS;
    imu_state->raw_az_mps2 = ((float)raw_az / 16384.0f) * GRAVITY_MSS;

    // Jiroskop donusumu: +/- 250 deg/s (131 LSB / deg/s) -> rad/s
    float gx_rads = ((float)raw_gx / 131.0f) * DEG_TO_RAD_F;
    imu_state->raw_gx_rads = gx_rads - gyro_x_bias_; // Ofset duzeltmesi

    imu_state->imu_healthy = true;
    return true;
}

bool MPU6050Driver::writeRegister(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(data);
    return (Wire.endTransmission() == 0);
}

bool MPU6050Driver::readRegisters(uint8_t reg, uint8_t* buffer, size_t length) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    size_t count = Wire.requestFrom((int)addr_, (int)length);
    if (count != length) {
        return false;
    }
    for (size_t i = 0; i < length; ++i) {
        buffer[i] = Wire.read();
    }
    return true;
}

