"""
================================================================================
MODUL: WEBOTS IKI TEKERLEKLI DENGE ROBOTU ANA DONGUSU (WEBOTS MAIN)
================================================================================
Bu modul, Self-Balancing Robot uretim mimarisini (PIDController, DCMotorModel,
SensorFusion ve TelemetryLogger) Webots simulasyon ortaminda kosturan ince
donanim adaptoru ve ana calistiricisidir.

CALISMA ZAMANI SOZLESMESI:
- Tesis Dogrulugu: Gerilim (Voltaj/PWM) Komutu -> JGB37 DC Motor Modeli -> Tekerlek Torku.
- Kontrolcu: Kaskad PID denge ve enkoder geri beslemeli denge oncelikli PI donus.
- Sensor Fuzyonu: %98 Gyro + %2 Accel tamamlayici filtre; gurultu yalniz adaptorde eklenir.
- Yuk Emniyeti: Katman 4 uzerindeki dinamik Aspirin ilac kutusu yerel govde ekseninde izlenir.
================================================================================
"""

import csv
from datetime import datetime
import math
import os
from pathlib import Path
import random
import sys

from controller import Keyboard, Supervisor

# Proje kokunu ve venv site-packages'i sys.path'e ekle (Webots dahili Python korumasi)
project_root = Path(__file__).resolve().parents[3]
package_dir = project_root / "self_balancing_robot"
venv_site = project_root / "venv" / "Lib" / "site-packages"

for p in (str(project_root), str(package_dir), str(venv_site)):
    if p not in sys.path and Path(p).exists():
        sys.path.insert(0, p)

from self_balancing_robot import (
    CONFIG,
    IRHomingController,
    PIDController,
    SensorFusion,
    TelemetryLogger,
)


def parse_simulated_key(step: int, seq_str: str) -> int:
    """Ortam değişkeninden gelen adım tabanlı test sürüş komutlarını çözümler."""
    if not seq_str:
        return 0
    sim_key = 0
    for item in seq_str.split(","):
        if ":" in item:
            step_str, k_str = item.split(":", 1)
            if step >= int(step_str):
                k = k_str.strip().upper()
                if k in ("W", "UP"):
                    sim_key = ord("W")
                elif k in ("S", "DOWN"):
                    sim_key = ord("S")
                elif k in ("A", "LEFT"):
                    sim_key = ord("A")
                elif k in ("D", "RIGHT"):
                    sim_key = ord("D")
                elif k in ("H", "HOME", "DOCK"):
                    sim_key = ord("H")
                elif k in ("SPACE", "STOP"):
                    sim_key = ord(" ")
    return sim_key


def main() -> None:
    random.seed(int(os.environ.get("PID_RANDOM_SEED", "42")))
    robot = Supervisor()
    timestep = int(robot.getBasicTimeStep())
    if timestep <= 0:
        timestep = 10
    dt = timestep / 1000.0

    print("=" * 65)
    print("YTU CDTP - DC Motor & TB6612 Gerilim/PWM (Tork) Kontrolcüsü")
    print(f"Simulasyon Zaman Adimi: {timestep} ms ({1000 // timestep} Hz)")
    print("Motor Modeli: JGB37-520 12V (Back-EMF + Armatur Direnci + Reduktor)")
    print("=" * 65)

    cfg = CONFIG
    fusion = SensorFusion()
    controller = PIDController(cfg)
    homing = IRHomingController(cfg)
    motor_model = controller.motor

    log_path = os.environ.get("PID_LOG_PATH")
    if not log_path:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        log_path = str(project_root / "logs" / f"pid_{timestamp}.csv")
    telemetry = TelemetryLogger(filename=log_path, command_unit="volts")

    evaluation_file = None
    evaluation_writer = None
    eval_path = os.environ.get("PID_EVALUATION_PATH")
    if eval_path:
        evaluation_file = open(eval_path, "w", newline="", encoding="utf-8", buffering=1)
        evaluation_writer = csv.writer(evaluation_file)
        evaluation_writer.writerow([
            "step", "time", "pitch", "velocity", "target",
            "left_volts", "right_volts", "box_dropped", "rate",
            "omega", "torque", "accel_y", "accel_z",
            "slip_speed", "yaw_rate", "yaw_target", "position",
        ])

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
    left_motor.setAvailableTorque(motor_model.max_torque)
    right_motor.setAvailableTorque(motor_model.max_torque)
    left_motor.setTorque(0.0)
    right_motor.setTorque(0.0)

    left_encoder = robot.getDevice("left_encoder")
    right_encoder = robot.getDevice("right_encoder")
    left_encoder.enable(timestep)
    right_encoder.enable(timestep)

    robot_node = robot.getSelf()
    wheel_nodes = [robot.getFromDef("pid_left_wheel"), robot.getFromDef("pid_right_wheel")]
    init_trans = robot_node.getField("translation").getSFVec3f()
    init_rot = robot_node.getField("rotation").getSFRotation()

    aspirin_node = robot.getFromDef("aspirin_box")
    init_aspirin_trans = aspirin_node.getField("translation").getSFVec3f() if aspirin_node else None
    init_aspirin_rot = aspirin_node.getField("rotation").getSFRotation() if aspirin_node else None

    dock_node = robot.getFromDef("ir_dock_station")
    status_led_mat = robot.getFromDef("STATUS_LED_MAT")
    led_base_field = status_led_mat.getField("baseColor") if status_led_mat else None
    led_emissive_field = status_led_mat.getField("emissiveColor") if status_led_mat else None
    last_led_state = "OFF"

    def set_status_led(state: str) -> None:
        nonlocal last_led_state
        if state == last_led_state or not led_base_field or not led_emissive_field:
            return
        last_led_state = state
        if state == "DOCKED":
            led_base_field.setSFColor([0.1, 1.0, 0.2])
            led_emissive_field.setSFColor([0.1, 0.95, 0.2])
        elif state == "HOMING":
            led_base_field.setSFColor([1.0, 0.75, 0.0])
            led_emissive_field.setSFColor([0.95, 0.60, 0.0])
        else:
            led_base_field.setSFColor([0.22, 0.25, 0.28])
            led_emissive_field.setSFColor([0.0, 0.0, 0.0])
    init_spawn_pose = os.environ.get("TEST_INIT_POSE", "")
    if init_spawn_pose:
        parts = [float(x) for x in init_spawn_pose.split(",")]
        if len(parts) >= 3:
            spawn_trans = [parts[0], parts[1], init_trans[2]]
            spawn_rot = [0.0, 0.0, 1.0, parts[2]]
            robot_node.getField("translation").setSFVec3f(spawn_trans)
            robot_node.getField("rotation").setSFRotation(spawn_rot)
            robot_node.resetPhysics()
            if aspirin_node and init_aspirin_trans:
                aspirin_node.getField("translation").setSFVec3f(
                    [parts[0], parts[1], init_aspirin_trans[2]]
                )
                aspirin_node.getField("rotation").setSFRotation(spawn_rot)
                aspirin_node.resetPhysics()

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
    print(f"  [W / YUKARI OK]  : Ileri Git (+{cfg.DRIVE_SPEED:.2f} m/s)")
    print(f"  [S / ASAGI OK]   : Geri Git (-{cfg.DRIVE_SPEED:.2f} m/s)")
    print("  [A / SOL OK]     : Sola Don")
    print("  [D / SAG OK]     : Saga Don")
    print("  [H]              : Otonom 38 kHz IR Istasyon Yanasma Modu (Homing)")
    print("  [P]              : Robota Dis Durtme Uygula (4.0 N)")
    print("  [BOSLUK (SPACE)] : Dur / Oldugun Yerde Dengeyi Koru")
    print("  [R]              : Robotu Sifirla")
    print("-" * 65)

    try:
        while robot.step(timestep) != -1:
            step_count += 1
            sim_time += dt

            gyro_vals = gyro.getValues()
            accel_vals = accel.getValues()
            # Yapay sensör gürültüsü yalnız simülasyon adaptöründe üretilir; filtre saf örnek alır.
            gyro_x = gyro_vals[0] + random.gauss(0.0, cfg.fusion.gyro_noise_std)
            accel_y = accel_vals[1] + random.gauss(0.0, cfg.fusion.accel_noise_std)
            accel_z = accel_vals[2] + random.gauss(0.0, cfg.fusion.accel_noise_std)
            left_pos = left_encoder.getValue()
            right_pos = right_encoder.getValue()

            if first_step:
                last_left_pos = left_pos
                last_right_pos = right_pos
                fusion.update(gyro_x, accel_y, accel_z, dt)
                first_step = False
                continue

            # Tekerlek anlık dönüş hızları (rad/s)
            omega_left = (left_pos - last_left_pos) / dt
            omega_right = (right_pos - last_right_pos) / dt
            last_left_pos = left_pos
            last_right_pos = right_pos

            # Gerçek ve kestirim eğim açıları
            imu_rpy = imu.getRollPitchYaw()
            true_pitch_rad = imu_rpy[0]
            true_pitch_deg = math.degrees(true_pitch_rad)

            pitch_angle, filtered_rate = fusion.update(gyro_x, accel_y, accel_z, dt)
            pitch_deg = math.degrees(pitch_angle)
            pitch_rate_dps = math.degrees(filtered_rate)

            # Odometri güncellemesi
            filtered_vel = controller.update_odometry(omega_left, omega_right, dt)

            # İlaç kutusu yerel eksen (body-frame) denetimi
            if aspirin_node:
                box_pos = aspirin_node.getPosition()
                rob_pos = robot_node.getPosition()
                dw = [box_pos[0] - rob_pos[0], box_pos[1] - rob_pos[1], box_pos[2] - rob_pos[2]]
                rot = robot_node.getOrientation()
                dx_loc = rot[0] * dw[0] + rot[3] * dw[1] + rot[6] * dw[2]
                dy_loc = rot[1] * dw[0] + rot[4] * dw[1] + rot[7] * dw[2]
                dz_loc = rot[2] * dw[0] + rot[5] * dw[1] + rot[8] * dw[2]
                if (
                    abs(dx_loc) > cfg.mechanics.payload_max_dx_m
                    or abs(dy_loc) > cfg.mechanics.payload_max_dy_m
                    or dz_loc < cfg.mechanics.payload_min_dz_m
                    or dz_loc > cfg.mechanics.payload_max_dz_m
                ):
                    if not box_dropped:
                        box_dropped = True
                        print(
                            f"[HATA] Ilac kutusu hazneden dustu! "
                            f"Yerel sapma: dx={dx_loc:+.3f}m, dy={dy_loc:+.3f}m, dz={dz_loc:+.3f}m"
                        )

            # Klavye ve test komutlarını oku
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
                            print(
                                f"[DURTME] Adim {step_count}: "
                                f"Robota {push_force_y:+.1f} N test kuvveti uygulaniyor!"
                            )

            while key > 0:
                if key in (Keyboard.UP, ord("W"), ord("w")):
                    homing.stop()
                    desired_v += cfg.mechanics.drive_speed
                elif key in (Keyboard.DOWN, ord("S"), ord("s")):
                    homing.stop()
                    desired_v -= cfg.mechanics.drive_speed
                elif key in (Keyboard.LEFT, ord("A"), ord("a")):
                    homing.stop()
                    desired_turn -= cfg.mechanics.turn_speed
                elif key in (Keyboard.RIGHT, ord("D"), ord("d")):
                    homing.stop()
                    desired_turn += cfg.mechanics.turn_speed
                elif key in (ord("H"), ord("h")):
                    if not homing.active:
                        homing.start()
                        print("[IR HOMING] 38 kHz IR alici-verici yanasma modu AKTIF!")
                elif key in (ord("P"), ord("p")):
                    push_remaining_steps = 8
                    push_force_y = 4.0 * push_sign
                    push_sign = -push_sign
                    print(f"[DURTME] Robot ust govdesine {push_force_y:+.1f} N anlik kuvvet uygulandi!")
                elif key == ord(" "):
                    homing.stop()
                    desired_v = 0.0
                    desired_turn = 0.0
                    target_velocity = 0.0
                elif key in (ord("R"), ord("r")):
                    reset_requested = True
                key = keyboard.getKey()

            # Otonom 38 kHz IR Alıcı-Verici İstasyona Yanaşma (Modül C)
            if homing.active and dock_node is not None:
                rob_pos = robot_node.getPosition()
                dock_pos = dock_node.getPosition()
                beacon_xy = (dock_pos[0], dock_pos[1] + 0.012)
                ir_reading = homing.read_receivers(
                    robot_xy=(rob_pos[0], rob_pos[1]),
                    robot_yaw_rad=imu_rpy[2],
                    beacon_xy=beacon_xy,
                )
                desired_v, desired_turn, is_docked = homing.compute_commands(ir_reading)
                set_status_led("DOCKED" if is_docked else "HOMING")
                if step_count % 20 == 0:
                    if is_docked:
                        print(
                            f"[IR DOCKED] Mesafe: {ir_reading.distance_m * 100:5.1f} cm | "
                            f"Aci: {math.degrees(ir_reading.bearing_rad):+5.1f} deg | "
                            f"TCRT5000: {'AKTIF' if ir_reading.tcrt5000_triggered else '---'} | "
                            "LED: PARLAK YESIL (Istasyona Kenetlendi)"
                        )
                    elif not ir_reading.in_cone:
                        print(
                            f"[IR SINIR DISI] Mesafe: {ir_reading.distance_m * 100:5.1f} cm | "
                            f"Yanal Sapma: {ir_reading.lateral_error_m * 100:+5.1f} cm | "
                            "Robot +/-45 deg IR konisi disinda!"
                        )
                    elif ir_reading.ir_sum < 1e-3:
                        print(
                            f"[IR TARAMA (SCAN)] Mesafe: {ir_reading.distance_m * 100:5.1f} cm | "
                            f"Robot oldugu yerde donerek 38 kHz IR isigini ariyor ({desired_turn:+.2f} rad/s)..."
                        )
                    else:
                        print(
                            f"[IR HOMING] Mesafe: {ir_reading.distance_m * 100:5.1f} cm | "
                            f"Aci: {math.degrees(ir_reading.bearing_rad):+5.1f} deg | "
                            f"IR(L/R): {ir_reading.ir_left:.2f}/{ir_reading.ir_right:.2f} | "
                            f"TCRT5000: {'AKTIF' if ir_reading.tcrt5000_triggered else '---'} | "
                            f"Komut(v/w): {desired_v:+.2f} m/s, {desired_turn:+.2f} rad/s"
                        )
            else:
                set_status_led("OFF")

            if reset_requested:
                print("[SIFIRLAMA] Robot baslangic durumuna getiriliyor...")
                robot_node.getField("translation").setSFVec3f(init_trans)
                robot_node.getField("rotation").setSFRotation(init_rot)
                robot_node.resetPhysics()
                if aspirin_node and init_aspirin_trans:
                    aspirin_node.getField("translation").setSFVec3f(init_aspirin_trans)
                    aspirin_node.getField("rotation").setSFRotation(init_aspirin_rot)
                    aspirin_node.resetPhysics()
                left_motor.setTorque(0.0)
                right_motor.setTorque(0.0)
                fusion.reset()
                controller.reset()
                homing.stop()
                target_velocity = 0.0
                turn_command = 0.0
                box_dropped = False
                continue

            # Devrilme emniyet kesicisi
            if abs(true_pitch_deg) > cfg.mechanics.fall_limit_deg:
                left_motor.setTorque(0.0)
                right_motor.setTorque(0.0)
                if step_count % 50 == 0:
                    print(f"[EMNIYET] Robot devrildi! Egim: {true_pitch_deg:.1f} deg. Tork sifirlandi.")
                if max_test_steps > 0 and step_count >= max_test_steps:
                    robot.simulationQuit(1)
                    break
                continue

            vel_alpha = cfg.control.cmd_vel_alpha
            turn_alpha = cfg.control.cmd_turn_alpha
            target_velocity = (1.0 - vel_alpha) * target_velocity + vel_alpha * desired_v
            turn_command = (1.0 - turn_alpha) * turn_command + turn_alpha * desired_turn

            # Kontrolcü Gerilim / PWM Hesaplaması
            _, _, pwm_left, pwm_right = controller.compute(
                pitch_angle, filtered_rate, target_velocity, desired_v, turn_command, dt
            )

            # Sürücüye gönderilen nicemlenmiş 8-bit PWM karşılığı gerilim
            v_left = motor_model.pwm_to_voltage(pwm_left)
            v_right = motor_model.pwm_to_voltage(pwm_right)

            # DC Motor Fizik Modeli: Gerilim ve açısal hızdan tekerlek torku üretimi
            tau_left, i_left = motor_model.compute_torque(v_left, omega_left)
            tau_right, _ = motor_model.compute_torque(v_right, omega_right)

            # Harici bozucu dürtme kuvveti
            if push_remaining_steps > 0:
                push_remaining_steps -= 1
                try:
                    robot_node.addForceWithOffset([0.0, push_force_y, 0.0], [0.0, 0.0, 0.08], True)
                except Exception:
                    try:
                        robot_node.addForce([0.0, push_force_y, 0.0], True)
                    except Exception:
                        pass

            # Tekerleklere fiziksel tork uygula
            left_motor.setTorque(tau_left)
            right_motor.setTorque(tau_right)

            # Telemetri kaydı
            telemetry.log(
                sim_time,
                step_count,
                true_pitch_deg,
                pitch_rate_dps,
                filtered_vel,
                target_velocity,
                v_left,
                v_right,
                pitch_est_deg=pitch_deg,
                pos_m=controller.robot_pos,
            )

            if evaluation_writer is not None:
                orientation = robot_node.getOrientation()
                velocity = robot_node.getVelocity()
                forward_velocity = sum(orientation[i * 3 + 1] * velocity[i] for i in range(3))
                wheel_slips = []
                for node in wheel_nodes:
                    if node:
                        wv = node.getVelocity()
                        slip = (
                            (wv[0] - wv[4] * cfg.mechanics.wheel_radius) * orientation[1]
                            + (wv[1] + wv[3] * cfg.mechanics.wheel_radius) * orientation[4]
                        )
                        wheel_slips.append(slip)
                slip_speed = max((abs(x) for x in wheel_slips), default=float("nan"))
                yaw_target = (
                    -2.0 * cfg.mechanics.wheel_radius / cfg.mechanics.track_width * turn_command
                )
                evaluation_writer.writerow([
                    step_count,
                    sim_time,
                    true_pitch_deg,
                    forward_velocity,
                    desired_v,
                    v_left,
                    v_right,
                    int(box_dropped),
                    gyro_vals[0],
                    (omega_left + omega_right) / 2.0,
                    tau_left + tau_right,
                    accel_vals[1],
                    accel_vals[2],
                    slip_speed,
                    velocity[5],
                    yaw_target,
                    robot_node.getPosition()[1],
                ])

            # Konsol bilgilendirmesi (200 ms periyotla)
            if step_count % 20 == 0:
                print(
                    f"[t={sim_time:5.2f}s] Egim: {true_pitch_deg:+6.2f} deg | "
                    f"Pos: {controller.robot_pos:+6.3f} m | Hiz: {filtered_vel:+5.2f} m/s | "
                    f"Volt: {v_left:+5.1f}V (PWM: {pwm_left:+4d}) | "
                    f"Tork: {tau_left:+5.2f} Nm (Akim: {i_left:+4.2f}A)"
                )

            if max_test_steps > 0 and step_count >= max_test_steps:
                print(
                    f"[TEST TAMAMLANDI] {step_count} adim ({sim_time:.2f} s). "
                    f"Son Egim: {true_pitch_deg:+.2f} deg."
                )
                robot.simulationQuit(0)
                break
    finally:
        telemetry.close()
        if evaluation_file is not None:
            evaluation_file.close()


if __name__ == "__main__":
    main()
