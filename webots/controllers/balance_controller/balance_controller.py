"""
İki Tekerlekli Denge Robotu Webots Ana Kontrolcüsü
Modüler mimari: config, sensor_fusion, cascade_controller ve telemetry modüllerini koordine eder.
"""

import os
import sys
import math
from pathlib import Path
from controller import Supervisor, Keyboard

# Proje kökünü sys.path'e ekle (self_balancing_robot paketini yükler)
project_root = Path(__file__).resolve().parents[3]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from self_balancing_robot import (
    RobotConfig,
    SensorFusion,
    CascadeBalanceController,
    TelemetryLogger,
)

def parse_simulated_key(step: int, seq_str: str) -> int:
    """Ortam değişkeninden gelen otomatik test sürüş komutlarını ayrıştırır."""
    if not seq_str:
        return 0
    sim_key = 0
    for item in seq_str.split(","):
        if ":" in item:
            step_str, k_str = item.split(":", 1)
            if step >= int(step_str):
                k = k_str.strip().upper()
                if k in ('W', 'UP'):
                    sim_key = ord('W')
                elif k in ('S', 'DOWN'):
                    sim_key = ord('S')
                elif k in ('A', 'LEFT'):
                    sim_key = ord('A')
                elif k in ('D', 'RIGHT'):
                    sim_key = ord('D')
                elif k in ('SPACE', 'STOP'):
                    sim_key = ord(' ')
    return sim_key

def main():
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    if timestep <= 0:
        timestep = 10  # 10 ms (100 Hz kontrol döngüsü)

    dt = timestep / 1000.0

    print("=" * 65)
    print("YTU CDTP - Modüler Denge Robotu Kontrolcüsü")
    print(f"Simulasyon Zaman Adimi (dt): {timestep} ms ({1000 // timestep} Hz)")
    print("=" * 65)

    # 1. Modül Başlatımları
    cfg = RobotConfig()
    fusion = SensorFusion(cfg.GYRO_NOISE_STD, cfg.ACCEL_NOISE_STD)
    controller = CascadeBalanceController(cfg)
    telemetry = TelemetryLogger()

    # 2. Donanım ve Sensör Tanımlamaları
    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)

    imu = robot.getDevice("imu")
    imu.enable(timestep)
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    accel = robot.getDevice("accel")
    accel.enable(timestep)

    left_motor = robot.getDevice("left_motor")
    right_motor = robot.getDevice("right_motor")
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    left_encoder = robot.getDevice("left_encoder")
    right_encoder = robot.getDevice("right_encoder")
    left_encoder.enable(timestep)
    right_encoder.enable(timestep)

    # Robot ve İlaç Kutusu Düğümleri (Sıfırlama için)
    robot_node = robot.getSelf()
    init_trans = robot_node.getField("translation").getSFVec3f()
    init_rot = robot_node.getField("rotation").getSFRotation()

    aspirin_node = robot.getFromDef("aspirin_box")
    init_aspirin_trans = aspirin_node.getField("translation").getSFVec3f() if aspirin_node else None
    init_aspirin_rot = aspirin_node.getField("rotation").getSFRotation() if aspirin_node else None

    # Değişkenler
    last_left_pos = 0.0
    last_right_pos = 0.0
    first_step = True
    step_count = 0
    sim_time = 0.0

    target_velocity = 0.0
    turn_command = 0.0
    push_remaining_steps = 0
    push_force_y = 0.0
    push_sign = 1.0

    max_test_steps = int(os.environ.get("WEBOTS_TEST_STEPS", "0"))

    print("Klavye Kontrolleri:")
    print("  [W / YUKARI OK]  : İleri Git (+0.30 m/s)")
    print("  [S / ASAGI OK]   : Geri Git (-0.30 m/s)")
    print("  [A / SOL OK]     : Sola Dön")
    print("  [D / SAG OK]     : Sağa Dön")
    print("  [P]              : Robota Dış Dürtme / Bozucu Kuvvet Uygula (4.0 N)")
    print("  [BOSLUK (SPACE)] : Dur / Olduğun Yerde Dengeyi Koru")
    print("  [R]              : Robotu Başlangıç Noktasına ve Dengeye Sıfırla")
    print("-" * 65)

    while robot.step(timestep) != -1:
        step_count += 1
        sim_time += dt

        # Sensör Verilerini Oku
        gyro_vals = gyro.getValues()
        accel_vals = accel.getValues()
        left_pos = left_encoder.getValue()
        right_pos = right_encoder.getValue()

        if first_step:
            last_left_pos = left_pos
            last_right_pos = right_pos
            fusion.update(gyro_vals[0], accel_vals[1], accel_vals[2], dt)
            first_step = False
            continue

        # Sensör Füzyonu (Gürültü Enjeksiyonu + Tamamlayıcı Filtre)
        pitch_angle, filtered_rate = fusion.update(gyro_vals[0], accel_vals[1], accel_vals[2], dt)
        pitch_deg = math.degrees(pitch_angle)
        pitch_rate_dps = math.degrees(filtered_rate)

        # Odometri Güncellemesi
        d_left = left_pos - last_left_pos
        d_right = right_pos - last_right_pos
        last_left_pos = left_pos
        last_right_pos = right_pos
        filtered_vel = controller.update_odometry(d_left, d_right, dt)

        # Devrilme Emniyet Kesicisi (Fall Protection)
        if abs(pitch_deg) > cfg.FALL_LIMIT_DEG:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            if step_count % 50 == 0:
                print(f"[EMNIYET] Robot devrildi! Eğim: {pitch_deg:.1f} derece. Motorlar durduruldu.")
            if max_test_steps > 0 and step_count >= max_test_steps:
                print(f"[TEST BITTI] {step_count} adim tamamlandi (robot devrildi).")
                telemetry.close()
                robot.simulationQuit(1)
                break
            continue

        # Kullanıcı / Test Girdileri
        key = keyboard.getKey()
        desired_v = 0.0
        desired_turn = 0.0
        reset_requested = False

        sim_key = parse_simulated_key(step_count, os.environ.get("TEST_DRIVE_COMMANDS", ""))
        if sim_key != 0:
            key = sim_key

        test_push_seq = os.environ.get("TEST_PUSH_COMMANDS", "")
        if test_push_seq:
            for item in test_push_seq.split(","):
                if ":" in item:
                    s_str, f_str = item.split(":", 1)
                    if step_count == int(s_str):
                        push_remaining_steps = 8
                        push_force_y = float(f_str)
                        print(f"[DURTME] Adim {step_count}: Robota {push_force_y:+.1f} N test kuvveti uygulaniyor!")

        while key > 0:
            if key in (Keyboard.UP, ord('W'), ord('w')):
                desired_v += 0.30
            elif key in (Keyboard.DOWN, ord('S'), ord('s')):
                desired_v -= 0.30
            elif key in (Keyboard.LEFT, ord('A'), ord('a')):
                desired_turn -= cfg.TURN_SPEED
            elif key in (Keyboard.RIGHT, ord('D'), ord('d')):
                desired_turn += cfg.TURN_SPEED
            elif key in (ord('P'), ord('p')):
                push_remaining_steps = 8
                push_force_y = 4.0 * push_sign
                push_sign = -push_sign
                print(f"[DURTME] Robot ust govdesine {push_force_y:+.1f} N anlik kuvvet uygulandi!")
            elif key == ord(' '):
                desired_v = 0.0
                desired_turn = 0.0
                target_velocity = 0.0
            elif key in (ord('R'), ord('r')):
                reset_requested = True
            key = keyboard.getKey()

        # Fiziksel Sıfırlama (R Tuşu)
        if reset_requested:
            print("[SIFIRLAMA] Robot ve ilac kutusu baslangic konumuna getiriliyor...")
            robot_node.getField("translation").setSFVec3f(init_trans)
            robot_node.getField("rotation").setSFRotation(init_rot)
            robot_node.resetPhysics()
            if aspirin_node and init_aspirin_trans:
                aspirin_node.getField("translation").setSFVec3f(init_aspirin_trans)
                aspirin_node.getField("rotation").setSFRotation(init_aspirin_rot)
                aspirin_node.resetPhysics()
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            fusion.reset()
            controller.reset()
            target_velocity = 0.0
            turn_command = 0.0
            continue

        # Yumuşak Hız & Dönüş Rampası (Slew Filters)
        target_velocity = 0.96 * target_velocity + 0.04 * desired_v
        turn_command = 0.90 * turn_command + 0.10 * desired_turn

        # Kaskad Kontrol Çıktıları
        cmd_left, cmd_right = controller.compute(
            pitch_angle, filtered_rate, target_velocity, desired_v, turn_command, dt
        )

        # Dış Dürtme / Bozucu Kuvvet (Supervisor API)
        if push_remaining_steps > 0:
            push_remaining_steps -= 1
            try:
                robot_node.addForceWithOffset([0.0, push_force_y, 0.0], [0.0, 0.0, 0.08], True)
            except Exception:
                try:
                    robot_node.addForce([0.0, push_force_y, 0.0], True)
                except Exception:
                    pass

        # Motorlara Uygula
        left_motor.setVelocity(cmd_left)
        right_motor.setVelocity(cmd_right)

        # Telemetri Kaydı
        telemetry.log(sim_time, step_count, pitch_deg, pitch_rate_dps,
                      filtered_vel, target_velocity, cmd_left, cmd_right)

        # Konsol Bilgilendirmesi (200 ms)
        if step_count % 20 == 0:
            print(f"[t={sim_time:5.2f}s] Egim: {pitch_deg:+6.2f} deg | Pos: {controller.robot_pos:+6.3f} m | Hiz: {filtered_vel:+5.2f} m/s (Hedef: {target_velocity:+5.2f}) | Motor: {cmd_left:+5.1f} rad/s")

        # Test Sona Erme Kontrolü
        if max_test_steps > 0 and step_count >= max_test_steps:
            print(f"[TEST TAMAMLANDI] {step_count} adim ({sim_time:.2f} saniye). Son Egim: {pitch_deg:+.2f} deg.")
            telemetry.close()
            robot.simulationQuit(0)
            break

    telemetry.close()

if __name__ == "__main__":
    main()
