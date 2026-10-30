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

    # Robot ve İlaç Kutusu Düğümleri (Supervisor API)
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
    box_dropped = False

    max_test_steps = int(os.environ.get("WEBOTS_TEST_STEPS", "0"))

    print("Klavye Kontrolleri:")
    print(f"  [W / YUKARI OK]  : İleri Git (+{cfg.DRIVE_SPEED:.2f} m/s)")
    print(f"  [S / ASAGI OK]   : Geri Git (-{cfg.DRIVE_SPEED:.2f} m/s)")
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

        # Gerçek Fiziksel Eğim Açısı (Ground Truth - Webots IMU Roll around X)
        imu_rpy = imu.getRollPitchYaw()
        true_pitch_rad = imu_rpy[0]
        true_pitch_deg = math.degrees(true_pitch_rad)

        # Sensör Füzyonu (Gürültü Enjeksiyonu + Tamamlayıcı Filtre Tahmini)
        pitch_angle, filtered_rate = fusion.update(gyro_vals[0], accel_vals[1], accel_vals[2], dt)
        pitch_deg = math.degrees(pitch_angle)
        pitch_rate_dps = math.degrees(filtered_rate)

        # Odometri Güncellemesi
        d_left = left_pos - last_left_pos
        d_right = right_pos - last_right_pos
        last_left_pos = left_pos
        last_right_pos = right_pos
        filtered_vel = controller.update_odometry(d_left, d_right, dt)

        # İlaç Kutusu Konum Denetimi (Supervisor Doğrulama - Robot Yerel Body Frame Koordinatları)
        if aspirin_node:
            box_pos = aspirin_node.getPosition()
            rob_pos = robot_node.getPosition()
            # Dünya koordinatlarındaki fark vektörü
            dw = [box_pos[0] - rob_pos[0], box_pos[1] - rob_pos[1], box_pos[2] - rob_pos[2]]
            # Robot rotasyon matrisi R (3x3 row-major)
            R = robot_node.getOrientation()
            # R_inv = R^T ile robotun yerel eksenine (Body Frame) dönüştürme
            dx_loc = R[0] * dw[0] + R[3] * dw[1] + R[6] * dw[2]
            dy_loc = R[1] * dw[0] + R[4] * dw[1] + R[7] * dw[2]
            dz_loc = R[2] * dw[0] + R[5] * dw[1] + R[8] * dw[2]

            # Katman 4 kaset yuvası sınırları:
            # Hazne X genişliği 80 mm (+/- 40 mm), Aspirin kutusu 74 mm (+/- 37 mm)
            # Hazne Y derinliği 26 mm (+/- 13 mm), Aspirin kutusu 20 mm (+/- 10 mm)
            # Nominal Z yüksekliği: ~0.138 m (mil ekseninden tavan kasetine)
            # Kutu hazneden taşarsa veya düşerse hata üret:
            if abs(dx_loc) > 0.040 or abs(dy_loc) > 0.020 or dz_loc < 0.100 or dz_loc > 0.190:
                if not box_dropped:
                    box_dropped = True
                    print(f"[HATA] Ilac kutusu hazneden dustu! Yerel koordinat sapmasi: dx={dx_loc:+.3f}m, dy={dy_loc:+.3f}m, dz={dz_loc:+.3f}m")

        # Kullanıcı / Test Girdilerini Oku (Devrilme kontrolünden ÖNCE okunur, böylece devrilince R çalışır!)
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
                desired_v += cfg.DRIVE_SPEED
            elif key in (Keyboard.DOWN, ord('S'), ord('s')):
                desired_v -= cfg.DRIVE_SPEED
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

        # Fiziksel Sıfırlama (R Tuşu - Devrilmiş olsa bile çalışır!)
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
            box_dropped = False
            continue

        # Devrilme Emniyet Kesicisi (Fall Protection)
        if abs(true_pitch_deg) > cfg.FALL_LIMIT_DEG:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            if step_count % 50 == 0:
                print(f"[EMNIYET] Robot devrildi! Gerçek Eğim: {true_pitch_deg:.1f} derece. Motorlar durduruldu (Sıfırlamak için R'ye basın).")
            if max_test_steps > 0 and step_count >= max_test_steps:
                print(f"[TEST BITTI] {step_count} adim tamamlandi (robot devrildi).")
                telemetry.close()
                robot.simulationQuit(1)
                break
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

        # Telemetri Kaydı (Gerçek açı birincil 'pitch_deg' olarak, kestirim 'pitch_est_deg' olarak kaydedilir)
        telemetry.log(sim_time, step_count, true_pitch_deg, pitch_rate_dps,
                      filtered_vel, target_velocity, cmd_left, cmd_right,
                      pitch_est_deg=pitch_deg, pos_m=controller.robot_pos)

        # Konsol Bilgilendirmesi (200 ms)
        if step_count % 20 == 0:
            print(f"[t={sim_time:5.2f}s] Gercek Egim: {true_pitch_deg:+6.2f} deg (Filtre: {pitch_deg:+6.2f}) | Pos: {controller.robot_pos:+6.3f} m | Hiz: {filtered_vel:+5.2f} m/s (Hedef: {target_velocity:+5.2f}) | Motor: {cmd_left:+5.1f} rad/s")

        # Test Sona Erme Kontrolü
        if max_test_steps > 0 and step_count >= max_test_steps:
            print(f"[TEST TAMAMLANDI] {step_count} adim ({sim_time:.2f} saniye). Son Gercek Egim: {true_pitch_deg:+.2f} deg.")
            telemetry.close()
            robot.simulationQuit(0)
            break

    telemetry.close()

if __name__ == "__main__":
    main()
