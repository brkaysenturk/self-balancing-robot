#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <Arduino.h>
#include "robot_config.h"
#include "types.h"

/**
 * ================================================================================
 * MANYETIK ENKODER VE ODOMETRI HESAPLAYICI
 * ================================================================================
 * JGB37-520 arkasindaki Hall manyetik enkoderlerin A/B fazlarini IRAM donanımsal
 * kesmeleriyle (Interrupt) sayar. 100 Hz kaskad donguye kaymasiz tekerlek acisal
 * hizlari (rad/s), govde cizgisel hizi (m/s) ve odometri konumu uretir.
 * ================================================================================
 */

class EncoderReader {
public:
    EncoderReader();

    // Enkoder GPIO kesmelerini baslatir
    void begin();

    // 100 Hz periyotta cagrilarak hiz ve konum odometrisini gunceller
    void updateOdometry(float dt, OdometryState_t* odo_state);

    // Konum ve tick sayaclarini sifirla
    void reset();

    // Ham tick degerleri
    int32_t getTicksL() const;
    int32_t getTicksR() const;

    // Kesme fonksiyonlari tarafindan cagirilan statik sayac guncelleyiciler
    static void IRAM_ATTR isrLeftA();
    static void IRAM_ATTR isrRightA();

private:
    static volatile int32_t count_left_;
    static volatile int32_t count_right_;

    int32_t prev_count_left_;
    int32_t prev_count_right_;
    float filtered_vel_;
    float total_position_m_;
    portMUX_TYPE mux_;
};

#endif // ENCODER_READER_H

