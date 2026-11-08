/**
 * ================================================================================
 * IKI TEKERLEKLI HASTA DESTEK VE ILAC TASIMA DENGE ROBOTU
 * ARDUINO IDE GIRIS NOKTASI (ROBOT.INO)
 * ================================================================================
 * Bu proje hem PlatformIO hem de Arduino IDE ile 100% uyumludur.
 * PlatformIO kullaniyorsaniz: dogrudan `platformio run -t upload` komutuyla yukleyebilirsiniz.
 * Arduino IDE kullaniyorsaniz: Bu dosyayi Arduino IDE ile acip kart olarak
 * 'DOIT ESP32 DEVKIT V1' secerek derleyebilirsiniz.
 * ================================================================================
 */

#include "include/robot_config.h"
#include "include/types.h"
#include "src/sensor_fusion.cpp"
#include "src/mpu6050_driver.cpp"
#include "src/motor_driver.cpp"
#include "src/encoder_reader.cpp"
#include "src/pid_controller.cpp"
#include "src/ir_homing.cpp"
#include "src/robot_comm.cpp"
#include "src/main.cpp"

