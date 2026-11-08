#include "robot_comm.h"

GlovePacket_t RobotComm::latest_glove_pkt_;
volatile bool RobotComm::new_glove_pkt_received_ = false;

void RobotComm::onDataRecv(const uint8_t* mac_addr, const uint8_t* data, int data_len) {
    if (data_len == sizeof(GlovePacket_t)) {
        memcpy(&latest_glove_pkt_, data, sizeof(GlovePacket_t));
        new_glove_pkt_received_ = true;
    }
}

RobotComm::RobotComm()
    : raw_target_v_(0.0f),
      raw_target_turn_(0.0f),
      smoothed_v_(0.0f),
      smoothed_turn_(0.0f),
      last_cmd_time_ms_(0),
      homing_flag_(false),
      reset_flag_(false),
      estop_flag_(false),
      plotter_mode_(false) {
}

bool RobotComm::begin(uint32_t baud_rate) {
    Serial.begin(baud_rate);
    delay(20);

    printHelpMenu();

    // ESP-NOW Icin Wi-Fi Istasyon Modu (Router baglantisi gerekmez)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Hata: ESP-NOW baslatilamadi! Yalnizca USB Seri port devrede.");
        return false;
    }

    esp_now_register_recv_cb(onDataRecv);

    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("[ESP-NOW] Basarili! Robot ESP32 MAC Adresi: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.println("[ESP-NOW] Bu MAC adresini eldiven firmware'ine hedef olarak tanimlayin.");

    return true;
}

void RobotComm::printHelpMenu() {
    Serial.println("\n========================================================");
    Serial.println("EV ICI HASTA DESTEK VE ILAC TASIMA DENGE ROBOTU");
    Serial.println("BILGISAYAR TELEOPERASYON VE TEST KONTROL MENUSU");
    Serial.println("========================================================");
    Serial.println("Klavye Kontrol Tuslari (USB Seri Terminal):");
    Serial.println("  [W] : Ileri Surus (+0.20 m/s)");
    Serial.println("  [S] : Geri Surus  (-0.20 m/s)");
    Serial.println("  [A] : Sola Donus  (-1.20 rad/s / ~31 deg/s)");
    Serial.println("  [D] : Saga Donus  (+1.20 rad/s / ~31 deg/s)");
    Serial.println("  [BOSLUK / SPACE] : Dinamik Fren ve Konum Kilitleme");
    Serial.println("  [H] : Otonom 38 kHz IR Istasyon Yanasma Modu (Homing)");
    Serial.println("  [R] : Robotu ve Tum Integralleri Sifirla (Reset)");
    Serial.println("  [E] : E-Stop Acil Durdurma Ac/Kapat");
    Serial.println("  [T] : Telemetri Cikis Formatini Degistir (Okunur / Plotter)");
    Serial.println("========================================================\n");
}

void RobotComm::update() {
    processSerialInput();
    processGlovePacket();

    // Komut Zaman Asimi Emniyeti:
    // 600 ms boyunca hicbir yeni komut gelmezse hedef hizi yumusakca sifira cek
    if (millis() - last_cmd_time_ms_ > 600 && (raw_target_v_ != 0.0f || raw_target_turn_ != 0.0f)) {
        raw_target_v_ = 0.0f;
        raw_target_turn_ = 0.0f;
    }
}

void RobotComm::processSerialInput() {
    while (Serial.available() > 0) {
        char ch = (char)Serial.read();
        last_cmd_time_ms_ = millis();

        switch (ch) {
            case 'w':
            case 'W':
                raw_target_v_ = DRIVE_SPEED_MPS;
                raw_target_turn_ = 0.0f;
                Serial.println("[CMD] Ileri Surus (+0.20 m/s)");
                break;
            case 's':
            case 'S':
                raw_target_v_ = -DRIVE_SPEED_MPS;
                raw_target_turn_ = 0.0f;
                Serial.println("[CMD] Geri Surus (-0.20 m/s)");
                break;
            case 'a':
            case 'A':
                raw_target_turn_ = -TURN_SPEED_RADPS;
                Serial.println("[CMD] Sola Donus (-1.20 rad/s)");
                break;
            case 'd':
            case 'D':
                raw_target_turn_ = TURN_SPEED_RADPS;
                Serial.println("[CMD] Saga Donus (+1.20 rad/s)");
                break;
            case ' ':
                raw_target_v_ = 0.0f;
                raw_target_turn_ = 0.0f;
                Serial.println("[CMD] Dinamik Fren ve Konum Kilidi Aktif");
                break;
            case 'h':
            case 'H':
                homing_flag_ = true;
                Serial.println("[CMD] Otonom 38 kHz IR Yanasma Modu Tetiklendi");
                break;
            case 'r':
            case 'R':
                reset_flag_ = true;
                raw_target_v_ = 0.0f;
                raw_target_turn_ = 0.0f;
                Serial.println("[CMD] Sifirlama Talebi Gonderildi");
                break;
            case 'e':
            case 'E':
                estop_flag_ = !estop_flag_;
                Serial.printf("[CMD] Acil E-Stop Durumu: %s\n", estop_flag_ ? "KILITLI (MOTORLAR KAPALI)" : "ACIK");
                break;
            case 't':
            case 'T':
                plotter_mode_ = !plotter_mode_;
                Serial.printf("[CMD] Telemetri Cikis Modu: %s\n", plotter_mode_ ? "Serial Plotter CSV" : "Metin");
                break;
            default:
                break;
        }
    }
}

void RobotComm::processGlovePacket() {
    if (!new_glove_pkt_received_) return;

    new_glove_pkt_received_ = false;
    last_cmd_time_ms_ = millis();

    switch (latest_glove_pkt_.command_id) {
        case 0: // STOP
            raw_target_v_ = 0.0f;
            raw_target_turn_ = 0.0f;
            break;
        case 1: // FWD
            raw_target_v_ = DRIVE_SPEED_MPS;
            raw_target_turn_ = 0.0f;
            break;
        case 2: // REV
            raw_target_v_ = -DRIVE_SPEED_MPS;
            raw_target_turn_ = 0.0f;
            break;
        case 3: // LEFT
            raw_target_turn_ = -TURN_SPEED_RADPS;
            break;
        case 4: // RIGHT
            raw_target_turn_ = TURN_SPEED_RADPS;
            break;
        case 5: // HOMING
            homing_flag_ = true;
            break;
        default:
            break;
    }
}

void RobotComm::getSmoothedCommand(float dt, float* target_v, float* target_turn, TeleopCommand_t* cmd_out) {
    // Webots ile birebir ayni komut yumusatma rampasi (EMA)
    smoothed_v_ = (1.0f - CMD_VEL_ALPHA) * smoothed_v_ + CMD_VEL_ALPHA * raw_target_v_;
    smoothed_turn_ = (1.0f - CMD_TURN_ALPHA) * smoothed_turn_ + CMD_TURN_ALPHA * raw_target_turn_;

    if (target_v) *target_v = smoothed_v_;
    if (target_turn) *target_turn = smoothed_turn_;

    if (cmd_out) {
        cmd_out->desired_velocity = raw_target_v_;
        cmd_out->desired_turn = raw_target_turn_;
        cmd_out->last_cmd_time_ms = last_cmd_time_ms_;
        cmd_out->estop_requested = estop_flag_;
        cmd_out->homing_requested = homing_flag_;
        cmd_out->reset_requested = reset_flag_;
    }
}

void RobotComm::printTelemetry(const IMUState_t& imu,
                               const OdometryState_t& odo,
                               const ControllerOutput_t& ctrl,
                               RobotMode_t mode) {
    if (plotter_mode_) {
        // Arduino Serial Plotter Uyumlu Cikti
        Serial.printf("Pitch_Deg:%.2f,Pitch_Target:%.2f,Vel_mps:%.2f,PWM_L:%d,PWM_R:%d\n",
                      imu.pitch_deg,
                      ctrl.pitch_target_rad * 57.2957795f,
                      odo.filtered_vel_mps,
                      ctrl.pwm_left,
                      ctrl.pwm_right);
    } else {
        // Okunabilir Durum Satiri (Her 50 ms)
        const char* mode_str = "DISARMED";
        switch (mode) {
            case MODE_BALANCING: mode_str = "BALANCING"; break;
            case MODE_DRIVING:   mode_str = "DRIVING"; break;
            case MODE_TURNING:   mode_str = "TURNING"; break;
            case MODE_HOMING:    mode_str = "HOMING"; break;
            case MODE_DOCKED:    mode_str = "DOCKED"; break;
            case MODE_ESTOP:     mode_str = "ESTOP_FALL"; break;
            default: break;
        }

        Serial.printf("[%s] Egim: %+.2f deg (Hedef: %+.2f) | Hiz: %+.2f m/s | Pos: %+.3f m | PWM(L/R): %+4d / %+4d | V(L/R): %+.1f / %+.1f V\n",
                      mode_str,
                      imu.pitch_deg,
                      ctrl.pitch_target_rad * 57.2957795f,
                      odo.filtered_vel_mps,
                      odo.linear_position_m,
                      ctrl.pwm_left,
                      ctrl.pwm_right,
                      ctrl.v_left_volts,
                      ctrl.v_right_volts);
    }
}

