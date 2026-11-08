#include "pid_controller.h"
#include <math.h>

float PIDController::clamp(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

PIDController::PIDController()
    : target_pos_(0.0f),
      pos_integral_(0.0f),
      turn_integral_(0.0f),
      target_pitch_(0.0f),
      is_holding_pos_(true) {
}

void PIDController::reset() {
    target_pos_ = 0.0f;
    pos_integral_ = 0.0f;
    turn_integral_ = 0.0f;
    target_pitch_ = 0.0f;
    is_holding_pos_ = true;
}

void PIDController::compute(float pitch_angle_rad,
                            float filtered_rate_rads,
                            float target_velocity_mps,
                            float desired_v_mps,
                            float turn_command_rads,
                            const OdometryState_t& odometry,
                            float dt,
                            ControllerOutput_t* output) {
    if (!output) return;

    // -------------------------------------------------------------------------
    // 1. DIS DONGU: Kaskad Konum / Hiz Hedef Egim Acisi Uretimi
    // -------------------------------------------------------------------------
    if (fabsf(desired_v_mps) < HOLD_CMD_THRESHOLD_MPS &&
        fabsf(target_velocity_mps) < HOLD_VEL_THRESHOLD_MPS &&
        fabsf(odometry.filtered_vel_mps) < HOLD_VEL_THRESHOLD_MPS) {
        
        // Durus modu: Robot oldugu noktaya koordinat cipasi atarak konumunu kilitler
        if (!is_holding_pos_) {
            is_holding_pos_ = true;
            target_pos_ = odometry.linear_position_m;
            pos_integral_ = 0.0f;
        }

        float pos_err = odometry.linear_position_m - target_pos_;
        pos_integral_ = clamp(pos_integral_ + pos_err * dt, -MAX_POS_INT, MAX_POS_INT);

        float raw_target = (KP_POS * pos_err) + (KI_POS * pos_integral_) + (KV_VEL * odometry.filtered_vel_mps);
        target_pitch_ = clamp(raw_target, -MAX_PITCH_TARGET_RAD, MAX_PITCH_TARGET_RAD);
    } else {
        // Surus modu: Hiz hatasindan dinamik govde yatma acisi uretilir
        is_holding_pos_ = false;
        target_pos_ = odometry.linear_position_m;
        pos_integral_ = 0.0f;

        float vel_err = target_velocity_mps - odometry.filtered_vel_mps;
        target_pitch_ = -clamp(KP_VEL_DRIVE * vel_err, -MAX_PITCH_TARGET_RAD, MAX_PITCH_TARGET_RAD);
    }

    // -------------------------------------------------------------------------
    // 2. IC DONGU: Gerilim PD Denge Kontrolu (100 Hz Refleks)
    // -------------------------------------------------------------------------
    float pitch_error = pitch_angle_rad - target_pitch_;
    float u_balance = (KP_VOLT * pitch_error) + (KD_VOLT * filtered_rate_rads);
    float u_volts = u_balance;

    // -------------------------------------------------------------------------
    // 3. KALKIS VE STATIK SURTUNME TELAFISI (SMOOTH STICTION COMPENSATION)
    // -------------------------------------------------------------------------
    u_volts += DEADBAND_VOLTS * tanhf(STICTION_TANH_GAIN * u_volts);

    // -------------------------------------------------------------------------
    // 4. ENKODER GERI BESLEMELI PI DONUS DENETIMI (DENGE ONCELIKLI)
    // -------------------------------------------------------------------------
    const float total_resistance = R_ARMATURE + DRIVER_RESISTANCE;
    float headroom = fminf(MAX_TURN_VOLTS, fmaxf(0.0f, V_BATTERY_NOMINAL - fabsf(u_volts)));
    float target_turn = clamp(turn_command_rads, -TURN_SPEED_RADPS, TURN_SPEED_RADPS);

    float friction_ff = COULOMB_FRICTION_NM * total_resistance / KT_WHEEL;
    float damping_ff = (TURN_DAMPING_KE_RATIO * KE_WHEEL) +
                       (VISCOUS_FRICTION_NMS * total_resistance / KT_WHEEL);

    float turn_volts = KP_TURN * (target_turn - odometry.turn_omega_rads) +
                       friction_ff * tanhf(TURN_FRICTION_TANH_GAIN * target_turn) +
                       damping_ff * target_turn;

    // Denge doyumuna gore sartli anti-windup integrali
    if (fabsf(target_turn) < 0.001f) {
        turn_integral_ = 0.0f;
    } else {
        float error = target_turn - odometry.turn_omega_rads;
        float trial_integral = clamp(turn_integral_ + KI_TURN * error * dt,
                                     -TURN_INTEGRAL_LIMIT,
                                     TURN_INTEGRAL_LIMIT);
        if (fabsf(turn_volts + trial_integral) <= headroom || error * (turn_volts + trial_integral) < 0.0f) {
            turn_integral_ = trial_integral;
        }
    }

    turn_volts += turn_integral_;
    headroom = fminf(MAX_TURN_VOLTS, fmaxf(0.0f, V_BATTERY_NOMINAL - fabsf(u_volts)));
    turn_volts = clamp(turn_volts, -headroom, headroom);

    // Diferansiyel sol/sağ gerilim paylasimi
    float v_left = clamp(u_volts - turn_volts, -V_BATTERY_NOMINAL, V_BATTERY_NOMINAL);
    float v_right = clamp(u_volts + turn_volts, -V_BATTERY_NOMINAL, V_BATTERY_NOMINAL);

    // -------------------------------------------------------------------------
    // 5. TEKERLEK BASINA %85 TERS EMK ILERI BESLEMESI (BACK-EMF FEEDFORWARD)
    // -------------------------------------------------------------------------
    v_left = clamp(v_left + BACK_EMF_FF_RATIO * KE_WHEEL * odometry.omega_left_rads,
                   -V_BATTERY_NOMINAL, V_BATTERY_NOMINAL);
    v_right = clamp(v_right + BACK_EMF_FF_RATIO * KE_WHEEL * odometry.omega_right_rads,
                    -V_BATTERY_NOMINAL, V_BATTERY_NOMINAL);

    // -------------------------------------------------------------------------
    // 6. GERILIM -> 8-BIT PWM DONUSUMU (-255 ... +255)
    // -------------------------------------------------------------------------
    float duty_l = clamp(v_left / V_BATTERY_NOMINAL, -1.0f, 1.0f);
    float duty_r = clamp(v_right / V_BATTERY_NOMINAL, -1.0f, 1.0f);
    int16_t pwm_l = (int16_t)roundf(duty_l * (float)MAX_PWM_VALUE);
    int16_t pwm_r = (int16_t)roundf(duty_r * (float)MAX_PWM_VALUE);

    // Cikti yapisini doldur
    output->pitch_target_rad = target_pitch_;
    output->u_balance_volts = u_balance;
    output->u_turn_volts = turn_volts;
    output->v_left_volts = v_left;
    output->v_right_volts = v_right;
    output->pwm_left = pwm_l;
    output->pwm_right = pwm_r;
    output->is_holding_position = is_holding_pos_;
}

