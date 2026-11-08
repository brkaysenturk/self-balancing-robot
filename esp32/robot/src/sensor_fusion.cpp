#include "sensor_fusion.h"
#include <math.h>

SensorFusion::SensorFusion(float alpha, float rate_ema_alpha, float min_accel_z_g)
    : alpha_(alpha),
      rate_ema_alpha_(rate_ema_alpha),
      min_accel_z_g_(min_accel_z_g),
      pitch_est_(0.0f),
      filtered_rate_(0.0f),
      initialized_(false) {
}

void SensorFusion::update(float gyro_x_rads, float accel_y_mps2, float accel_z_mps2, float dt) {
    // İvmeölçerden statik eğim açısı kestirimi (atan2(y, z))
    float accel_pitch = 0.0f;
    if (fabsf(accel_z_mps2) > min_accel_z_g_) {
        accel_pitch = atan2f(accel_y_mps2, accel_z_mps2);
    }

    if (!initialized_) {
        pitch_est_ = accel_pitch;
        filtered_rate_ = gyro_x_rads;
        initialized_ = true;
        return;
    }

    // Tamamlayıcı Filtre: %98 Jiroskop İntegrali + %2 İvmeölçer Açısı
    pitch_est_ = alpha_ * (pitch_est_ + gyro_x_rads * dt) + (1.0f - alpha_) * accel_pitch;

    // Jiroskop Açısal Hız Alçak Geçiren Filtre (EMA - 50 Hz Titremeleri Önler)
    filtered_rate_ = (1.0f - rate_ema_alpha_) * filtered_rate_ + rate_ema_alpha_ * gyro_x_rads;
}

void SensorFusion::reset() {
    pitch_est_ = 0.0f;
    filtered_rate_ = 0.0f;
    initialized_ = false;
}

