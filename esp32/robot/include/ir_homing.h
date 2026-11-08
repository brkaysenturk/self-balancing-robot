#ifndef IR_HOMING_H
#define IR_HOMING_H

#include <Arduino.h>
#include "robot_config.h"
#include "types.h"

/**
 * ================================================================================
 * OTONOM 38 KHZ IR ISTASYON YANASMA DENETLEYICISI (MODUL C)
 * ================================================================================
 * Katman 3 on kenarindaki 85 mm aralikli 2x TSOP2238 IR alicilarindan gelen aktif-LOW
 * burst sinyallerini okur.
 * - Sinyal yoksa: Oldugu yerde 0.75 rad/s radar taramasi (Search Scan)
 * - Sinyal yakalanirsa: Diferansiyel orantili nirengi ile istasyona yonelme
 * - Istasyon tamponuna temas edilirse: Motorlari durdurur ve Sarj LED'ini yakar
 * ================================================================================
 */

typedef enum {
    HOMING_STATE_INACTIVE = 0,
    HOMING_STATE_SEARCHING,     // Istasyon fenerini tarama
    HOMING_STATE_APPROACHING,   // Istasyona dogru kontrollu yaklasma
    HOMING_STATE_DOCKED         // Istasyona kenetlendi, sarj basladi
} HomingInternalState_t;

class IRHomingController {
public:
    IRHomingController();

    // IR GPIO pinlerini ve tampon switch girisini baslatir
    void begin();

    // Otonom yanasma modunu baslat
    void startHoming();

    // Otonom modu iptal et
    void stopHoming();

    // 100 Hz periyotta yanasma mantigini isletir
    // Donus: desired_v (m/s) ve desired_turn (rad/s) komutlarini gunceller
    bool update(float* desired_v, float* desired_turn);

    bool isHomingActive() const { return state_ != HOMING_STATE_INACTIVE; }
    bool isDocked() const { return state_ == HOMING_STATE_DOCKED; }
    HomingInternalState_t getState() const { return state_; }

private:
    HomingInternalState_t state_;
    uint32_t start_time_ms_;
    uint32_t last_seen_ir_ms_;
    int search_direction_;      // +1 Saga tarama, -1 Sola tarama
};

#endif // IR_HOMING_H

