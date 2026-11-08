#include "encoder_reader.h"

// Statik degiskenlerin tanimi
volatile int32_t EncoderReader::count_left_ = 0;
volatile int32_t EncoderReader::count_right_ = 0;

void IRAM_ATTR EncoderReader::isrLeftA() {
    // Faz A yukselen kenar: Faz B durumuna gore yon belirlenir
    if (digitalRead(PIN_ENC_L_B) == HIGH) {
        count_left_++;
    } else {
        count_left_--;
    }
}

void IRAM_ATTR EncoderReader::isrRightA() {
    // Sag motor fiziksel montajda sol motorun ayna simetrigidir
    if (digitalRead(PIN_ENC_R_B) == HIGH) {
        count_right_--;
    } else {
        count_right_++;
    }
}

EncoderReader::EncoderReader()
    : prev_count_left_(0),
      prev_count_right_(0),
      filtered_vel_(0.0f),
      total_position_m_(0.0f),
      mux_(portMUX_INITIALIZER_UNLOCKED) {
}

void EncoderReader::begin() {
    pinMode(PIN_ENC_L_A, INPUT);
    pinMode(PIN_ENC_L_B, INPUT);
    pinMode(PIN_ENC_R_A, INPUT);
    pinMode(PIN_ENC_R_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrLeftA, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrRightA, RISING);

    reset();
    Serial.println("[ENCODER] Manyetik enkoder kesmeleri baslatildi (PPR: 330)");
}

void EncoderReader::reset() {
    portENTER_CRITICAL(&mux_);
    count_left_ = 0;
    count_right_ = 0;
    portEXIT_CRITICAL(&mux_);

    prev_count_left_ = 0;
    prev_count_right_ = 0;
    filtered_vel_ = 0.0f;
    total_position_m_ = 0.0f;
}

int32_t EncoderReader::getTicksL() const {
    int32_t val;
    portENTER_CRITICAL((portMUX_TYPE*)&mux_);
    val = count_left_;
    portEXIT_CRITICAL((portMUX_TYPE*)&mux_);
    return val;
}

int32_t EncoderReader::getTicksR() const {
    int32_t val;
    portENTER_CRITICAL((portMUX_TYPE*)&mux_);
    val = count_right_;
    portEXIT_CRITICAL((portMUX_TYPE*)&mux_);
    return val;
}

void EncoderReader::updateOdometry(float dt, OdometryState_t* odo_state) {
    if (!odo_state || dt <= 0.0001f) return;

    int32_t current_l, current_r;
    portENTER_CRITICAL(&mux_);
    current_l = count_left_;
    current_r = count_right_;
    portEXIT_CRITICAL(&mux_);

    int32_t delta_l = current_l - prev_count_left_;
    int32_t delta_r = current_r - prev_count_right_;
    prev_count_left_ = current_l;
    prev_count_right_ = current_r;

    // Tekerlek acisal hizlari: omega = (delta_ticks / PPR) * 2 * PI / dt (rad/s)
    const float ticks_to_rad = (2.0f * 3.14159265f) / ENCODER_PPR;
    float omega_l = ((float)delta_l * ticks_to_rad) / dt;
    float omega_r = ((float)delta_r * ticks_to_rad) / dt;

    // Cizgisel tekerlek hizlari: v = omega * R
    float v_l = omega_l * WHEEL_RADIUS_M;
    float v_r = omega_r * WHEEL_RADIUS_M;
    float raw_vel = (v_l + v_r) / 2.0f;

    // Toplam kat edilen mesafe integrali
    total_position_m_ += raw_vel * dt;

    // Cizgisel hiz EMA filtresi
    filtered_vel_ = (1.0f - ODOMETRY_VEL_ALPHA) * filtered_vel_ + ODOMETRY_VEL_ALPHA * raw_vel;

    // Odometri durum yapisini doldur
    odo_state->enc_left_ticks = current_l;
    odo_state->enc_right_ticks = current_r;
    odo_state->omega_left_rads = omega_l;
    odo_state->omega_right_rads = omega_r;
    odo_state->linear_velocity_mps = raw_vel;
    odo_state->filtered_vel_mps = filtered_vel_;
    odo_state->linear_position_m = total_position_m_;
    odo_state->turn_omega_rads = (omega_r - omega_l) / 2.0f;
}

