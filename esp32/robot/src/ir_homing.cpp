#include "ir_homing.h"

IRHomingController::IRHomingController()
    : state_(HOMING_STATE_INACTIVE),
      start_time_ms_(0),
      last_seen_ir_ms_(0),
      search_direction_(1) {
}

void IRHomingController::begin() {
    // TSOP2238 aktif-LOW dijital alicilar
    pinMode(PIN_TSOP_LEFT, INPUT_PULLUP);
    pinMode(PIN_TSOP_RIGHT, INPUT_PULLUP);

    // Fiziksel tampon temas butonu (Normalde acik, temas aninda GND'ye ceker)
    pinMode(PIN_DOCK_BUMPER, INPUT_PULLUP);

    // Durum ve sarj gosterge LED'i
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);

    state_ = HOMING_STATE_INACTIVE;
    Serial.println("[IR_HOMING] 38 kHz TSOP2238 alici ve tampon girisleri baslatildi");
}

void IRHomingController::startHoming() {
    state_ = HOMING_STATE_SEARCHING;
    start_time_ms_ = millis();
    last_seen_ir_ms_ = millis();
    search_direction_ = 1; // Varsayilan saga tarama
    digitalWrite(PIN_STATUS_LED, LOW);
    Serial.println("[IR_HOMING] Otonom istasyon yanasma modu AKTIF!");
}

void IRHomingController::stopHoming() {
    state_ = HOMING_STATE_INACTIVE;
    digitalWrite(PIN_STATUS_LED, LOW);
    Serial.println("[IR_HOMING] Otonom yanasma durduruldu.");
}

bool IRHomingController::update(float* desired_v, float* desired_turn) {
    if (!desired_v || !desired_turn || state_ == HOMING_STATE_INACTIVE) {
        return false;
    }

    uint32_t now = millis();

    // 1. Emniyet Zaman Asimi Denetimi
    if (now - start_time_ms_ > HOMING_DOCK_TIMEOUT_MS) {
        Serial.println("[IR_HOMING] Emniyet zaman asimi (12s doldu)! Yanasma iptal edildi.");
        stopHoming();
        *desired_v = 0.0f;
        *desired_turn = 0.0f;
        return false;
    }

    // 2. Tampon Temas Denetimi (Istasyon dayanak temas bolgesi)
    // LOW = Buton basildi / temas gerceklesti
    if (digitalRead(PIN_DOCK_BUMPER) == LOW || state_ == HOMING_STATE_DOCKED) {
        state_ = HOMING_STATE_DOCKED;
        *desired_v = 0.0f;
        *desired_turn = 0.0f;
        digitalWrite(PIN_STATUS_LED, HIGH); // Modul D: Sarj basladi LED'i yanar
        return true;
    }

    // 3. TSOP2238 Sensor Okumalari (Aktif LOW: LOW = 38 kHz IR Isik Algilandi)
    bool left_active = (digitalRead(PIN_TSOP_LEFT) == LOW);
    bool right_active = (digitalRead(PIN_TSOP_RIGHT) == LOW);

    if (left_active || right_active) {
        last_seen_ir_ms_ = now;
        state_ = HOMING_STATE_APPROACHING;

        // Her iki goz de goruyorsa: Duz ileri yaklas
        if (left_active && right_active) {
            *desired_v = HOMING_CREEP_SPEED_MPS; // 0.08 m/s hassas yanaşma
            *desired_turn = 0.0f;
        }
        // Sadece sol goz goruyorsa: Sola dogru yonel
        else if (left_active && !right_active) {
            *desired_v = HOMING_CREEP_SPEED_MPS;
            *desired_turn = -HOMING_MAX_TURN_RATE * 0.70f;
            search_direction_ = -1; // Sinyal kaybolursa sola aramaya devam et
        }
        // Sadece sag goz goruyorsa: Saga dogru yonel
        else if (!left_active && right_active) {
            *desired_v = HOMING_CREEP_SPEED_MPS;
            *desired_turn = +HOMING_MAX_TURN_RATE * 0.70f;
            search_direction_ = 1; // Sinyal kaybolursa saga aramaya devam et
        }
    } else {
        // Sinyal yoksa: Oldugu yerde radar tarama modu (In-place search scan)
        state_ = HOMING_STATE_SEARCHING;
        *desired_v = 0.0f;
        *desired_turn = (float)search_direction_ * HOMING_SEARCH_TURN_RATE;
    }

    return true;
}

