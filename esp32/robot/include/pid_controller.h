#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "robot_config.h"
#include "types.h"
#include <stdbool.h>

/**
 * ================================================================================
 * KASKAD GERILIM/PWM PID VE PI DONUS KONTROLCUSU
 * ================================================================================
 * Webots ortaminda Optuna TPE ile 830g gozlenen kutle icin dogrulanmis cift dongulu
 * kaskad kontrol mimarisidir.
 * - Ic Dongu: 100 Hz Govde Acisi PD Denge Denetleyicisi
 * - Dis Dongu: 100 Hz Cizgisel Hiz/Konum Kaskad Egim Hedefi Ureticisi
 * - Geri Beslemeli Donus: Enkoder diferansiyel hiz farkli PI denetimi
 * - Ileri Besleme: %85 Motor Ters EMK ve Kalkis Surtunme Telafisi
 * ================================================================================
 */

class PIDController {
public:
    PIDController();

    // 100 Hz cevrimde gerilim ve PWM ciktilarini hesaplar
    void compute(float pitch_angle_rad,
                 float filtered_rate_rads,
                 float target_velocity_mps,
                 float desired_v_mps,
                 float turn_command_rads,
                 const OdometryState_t& odometry,
                 float dt,
                 ControllerOutput_t* output);

    // Kontrolcu durumunu, integralleri ve konum kilidini sifirla
    void reset();

    bool isHoldingPosition() const { return is_holding_pos_; }
    float getTargetPitch() const { return target_pitch_; }

private:
    float target_pos_;
    float pos_integral_;
    float turn_integral_;
    float target_pitch_;
    bool is_holding_pos_;

    static float clamp(float val, float min_val, float max_val);
};

#endif // PID_CONTROLLER_H

