#ifndef ROBOT_COMM_H
#define ROBOT_COMM_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "robot_config.h"
#include "types.h"

/**
 * ================================================================================
 * BIRLESIK ROBOT ILETISIM KOPRUSU
 * ================================================================================
 * Robotu iki farkli kaynaktan yonetmeyi saglar:
 * 1. Bilgisayar USB Seri Portu (Ev ve masaustu testleri icin: W/S/A/D/Space/H/R/E)
 * 2. Giyilebilir Eldiven (ESP-NOW protokolu, 1-2 ms dusuk gecikmeli kablosuz paket)
 * Her iki kaynak da ayni standart TeleopCommand_t yapisina beslenir.
 * ================================================================================
 */

// Eldivenden Robota Gonderilen Standart 8-Byte ESP-NOW Paket Yapisi
typedef struct __attribute__((packed)) {
    uint8_t command_id;         // 0: STOP, 1: FWD, 2: REV, 3: LEFT, 4: RIGHT, 5: HOMING
    int8_t  hand_pitch_deg;     // Eldiven eğim açısı (-90 ... +90)
    int8_t  hand_roll_deg;      // Eldiven yuvarlanma açısı (-90 ... +90)
    uint8_t checksum;           // Basit dogrulama bayti
    uint32_t sequence_num;      // Paket sira numarasi
} GlovePacket_t;

class RobotComm {
public:
    RobotComm();

    // Seri portu ve ESP-NOW alicisini baslatir
    bool begin(uint32_t baud_rate = SERIAL_BAUD_RATE);

    // 20 Hz telemetri/iletisim dongusunde komutlari ve seri portu tarar
    void update();

    // 100 Hz gercek zamanli kontrolcuye yumusatilmis komutlari verir
    void getSmoothedCommand(float dt, float* target_v, float* target_turn, TeleopCommand_t* cmd_out);

    // Seri port telemetri ciktisini yazar
    void printTelemetry(const IMUState_t& imu,
                        const OdometryState_t& odo,
                        const ControllerOutput_t& ctrl,
                        RobotMode_t mode);

    // ESP-NOW paket alim kesmesi
    static void onDataRecv(const uint8_t* mac_addr, const uint8_t* data, int data_len);

    bool isHomingRequested() const { return homing_flag_; }
    void clearHomingRequest() { homing_flag_ = false; }
    bool isResetRequested() const { return reset_flag_; }
    void clearResetRequest() { reset_flag_ = false; }
    bool isEstopToggled() const { return estop_flag_; }

private:
    float raw_target_v_;
    float raw_target_turn_;
    float smoothed_v_;
    float smoothed_turn_;
    uint32_t last_cmd_time_ms_;
    bool homing_flag_;
    bool reset_flag_;
    bool estop_flag_;
    bool plotter_mode_;

    static GlovePacket_t latest_glove_pkt_;
    static volatile bool new_glove_pkt_received_;

    void processSerialInput();
    void processGlovePacket();
    void printHelpMenu();
};

#endif // ROBOT_COMM_H

