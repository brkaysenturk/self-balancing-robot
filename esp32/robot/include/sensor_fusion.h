#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "robot_config.h"
#include <stdbool.h>

/**
 * ================================================================================
 * DETERMINISTIK TAMAMLAYICI FILTRE VE SENSOR FUZYONU
 * ================================================================================
 * MPU-6050 jiroskop ve ivmeolcer verilerini birlestirerek gurultuden arindirilmis
 * govde egim acisini (%98 Gyro + %2 Accel) ve EMA filtreli acisal hizi deterministik
 * olarak kestirir. Python self_balancing_robot/sensor_fusion.py ile tam esdegerdir.
 * ================================================================================
 */

class SensorFusion {
public:
    SensorFusion(float alpha = FUSION_ALPHA,
                 float rate_ema_alpha = RATE_EMA_ALPHA,
                 float min_accel_z_g = MIN_ACCEL_Z_G);

    // Yeni sensör verisiyle filtreyi gunceller
    void update(float gyro_x_rads, float accel_y_mps2, float accel_z_mps2, float dt);

    // Kestirilen aci ve acisal hiz erisimi
    float getPitchRad() const { return pitch_est_; }
    float getPitchDeg() const { return pitch_est_ * 57.2957795f; }
    float getFilteredRateRadS() const { return filtered_rate_; }
    bool isInitialized() const { return initialized_; }

    // Filtreyi baslangic durumuna sifirla
    void reset();

private:
    float alpha_;
    float rate_ema_alpha_;
    float min_accel_z_g_;
    float pitch_est_;
    float filtered_rate_;
    bool initialized_;
};

#endif // SENSOR_FUSION_H

