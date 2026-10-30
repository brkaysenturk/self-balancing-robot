"""
YTU CDTP - Gerçekçi DC Motor & TB6612 Gerilim/PWM (Tork) Tabanlı Denge Kontrolcüsü
Bu kontrolcü, ESP32 ve TB6612 donanımının fiziksel çalışma prensibini ortalama gerilim modeliyle yaklaşık temsil eder:
Denge Kontrolcüsü -> Gerilim (Voltaj/PWM) Komutu -> JGB37 DC Motor Modeli (Ters EMK + Direnç) -> Tekerlek Torku
"""

import os
import sys
import math
from pathlib import Path
from controller import Supervisor, Keyboard

# Proje kök dizinini ekle
project_root = Path(__file__).resolve().parents[3]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from self_balancing_robot import (
    RobotConfig,
    SensorFusion,
    TelemetryLogger,
    DCMotorModel,
)

def clamp(val: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(val, max_val))

def parse_simulated_key(step: int, seq_str: str) -> int:
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

class VoltageBalanceController:
    """
    ESP32 üzerinde çalışacak FreeRTOS C koduna birebir eşdeğer
    PWM/Gerilim tabanlı kaskad PID denge algoritması.
    """
    def __init__(self, cfg: RobotConfig = RobotConfig()):
        self.cfg = cfg
        self.motor = DCMotorModel()

        # Durum Geri Beslemeli Gerilim / PWM Kazançları (State Feedback Voltages)
        self.KP_VOLT: float = 65.0    # Açı kazancı (V/rad) ~1.22 V/derece (veya ~26 PWM/derece)
        self.KD_VOLT: float = 1.60    # Eğim açısal hızı sönümleme gerilimi (V/(rad/s))
        self.KV_VEL: float = 0.15      # Hızdan hedef eğim sönümlemesi (rad/(m/s))
        self.KP_POS: float = 0.18      # Konumdan hedef eğim (rad/m)
        self.KI_POS: float = 0.015     # Konum integrali kazancı (rad/(m*s))
        self.KP_VEL_DRIVE: float = 0.12  # Sürüş anı hız hatası hedef açısı kazancı
        self.MAX_PITCH_TARGET: float = 0.025  # Azami hedef eğim açısı (~1.43 derece)
        self.MAX_POS_INT: float = 0.030        # Clamping integral sınırı (m*s)

        # Durum Değişkenleri
        self.robot_pos: float = 0.0
        self.target_pos: float = 0.0
        self.pos_integral: float = 0.0
        self.filtered_vel: float = 0.0
        self.is_holding_pos: bool = True
        self.last_u_bal = 0.0
        self.last_u_vel = 0.0
        self.last_u_pos = 0.0

    def update_odometry(self, omega_left: float, omega_right: float, dt: float) -> float:
        """Tekerlek açısal hızlarından robotun çizgisel hız ve konumunu hesaplar."""
        v_left = -(omega_left) * self.cfg.WHEEL_RADIUS
        v_right = -(omega_right) * self.cfg.WHEEL_RADIUS
        raw_vel = (v_left + v_right) / 2.0
        self.robot_pos += raw_vel * dt
        self.filtered_vel = 0.90 * self.filtered_vel + 0.10 * raw_vel
        return self.filtered_vel

    def compute(
        self,
        pitch_angle: float,
        filtered_rate: float,
        target_velocity: float,
        desired_v: float,
        turn_command: float,
        dt: float,
    ) -> tuple[float, float, int, int]:
        """
        Denge ve sürüş için motor gerilimlerini ve PWM değerlerini hesaplar.
        Dönüş: (v_left_volts, v_right_volts, pwm_left, pwm_right)
        """
        # 1. Dış Döngü: Kaskad Konum / Hız Hedef Eğim Açısı
        # Denge döngüsüyle çelişmemesi için hız ve konum, hedef eğim açısı (pitch_target) üzerinden yönetilir.
        if abs(desired_v) < 0.01 and abs(target_velocity) < 0.02 and abs(self.filtered_vel) < 0.02:
            if not self.is_holding_pos:
                self.is_holding_pos = True
                self.target_pos = self.robot_pos
                self.pos_integral = 0.0

            pos_err = self.robot_pos - self.target_pos
            self.pos_integral = clamp(self.pos_integral + pos_err * dt, -self.MAX_POS_INT, self.MAX_POS_INT)
            raw_target = (self.KP_POS * pos_err) + (self.KI_POS * self.pos_integral) + (self.KV_VEL * self.filtered_vel)
            pitch_target = clamp(raw_target, -self.MAX_PITCH_TARGET, self.MAX_PITCH_TARGET)
        else:
            self.is_holding_pos = False
            self.target_pos = self.robot_pos
            self.pos_integral = 0.0
            vel_err = target_velocity - self.filtered_vel
            pitch_target = -clamp(self.KP_VEL_DRIVE * vel_err, -self.MAX_PITCH_TARGET, self.MAX_PITCH_TARGET)

        # 2. İç Döngü: Gerilim PID Denge Kontrolü (Hıza müdahale etmez, sadece açıyı yakalar)
        pitch_error = pitch_angle - pitch_target
        u_balance = (self.KP_VOLT * pitch_error) + (self.KD_VOLT * filtered_rate)
        self.last_u_bal = u_balance
        self.last_u_vel = pitch_target
        self.last_u_pos = self.pos_integral

        u_volts = u_balance

        # 3. Gerilim Ölü Bölgesi / Pürüzsüz Sürtünme Telafisi (Smooth Stiction Compensation)
        u_volts += self.motor.deadband_volts * math.tanh(3.0 * u_volts)

        # 4. Diferansiyel Dönüş Eklenmesi (Gerilim cinsinden)
        turn_volts = turn_command * 2.5
        v_left = clamp(u_volts - turn_volts, -self.motor.v_battery, self.motor.v_battery)
        v_right = clamp(u_volts + turn_volts, -self.motor.v_battery, self.motor.v_battery)

        pwm_left = self.motor.voltage_to_pwm(v_left)
        pwm_right = self.motor.voltage_to_pwm(v_right)

        return v_left, v_right, pwm_left, pwm_right

    def reset(self):
        self.robot_pos = 0.0
        self.target_pos = 0.0
        self.pos_integral = 0.0
        self.filtered_vel = 0.0
        self.is_holding_pos = True

def main():
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

    cfg = RobotConfig()
    fusion = SensorFusion(cfg.GYRO_NOISE_STD, cfg.ACCEL_NOISE_STD)
    controller = VoltageBalanceController(cfg)
    motor_model = controller.motor
    telemetry = TelemetryLogger(filename=os.path.join(str(project_root), "logs", "voltage_telemetry.csv"), command_unit="volts")

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
    # Tork modunu hazirla
    left_motor.setAvailableTorque(motor_model.max_torque)
    right_motor.setAvailableTorque(motor_model.max_torque)
    left_motor.setTorque(0.0)
    right_motor.setTorque(0.0)

    left_encoder = robot.getDevice("left_encoder")
    right_encoder = robot.getDevice("right_encoder")
    left_encoder.enable(timestep)
    right_encoder.enable(timestep)

    robot_node = robot.getSelf()
    init_trans = robot_node.getField("translation").getSFVec3f()
    init_rot = robot_node.getField("rotation").getSFRotation()

    aspirin_node = robot.getFromDef("aspirin_box")
    init_aspirin_trans = aspirin_node.getField("translation").getSFVec3f() if aspirin_node else None
    init_aspirin_rot = aspirin_node.getField("rotation").getSFRotation() if aspirin_node else None

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
    print("  [P]              : Robota Dis Durtme Uygula (4.0 N)")
    print("  [BOSLUK (SPACE)] : Dur / Oldugun Yerde Dengeyi Koru")
    print("  [R]              : Robotu Sifirla")
    print("-" * 65)

    while robot.step(timestep) != -1:
        step_count += 1
        sim_time += dt

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

        # Tekerlek anlik donus hizlari (rad/s)
        omega_left = (left_pos - last_left_pos) / dt
        omega_right = (right_pos - last_right_pos) / dt
        last_left_pos = left_pos
        last_right_pos = right_pos

        # Gercek ve kestirim egim acilari
        imu_rpy = imu.getRollPitchYaw()
        true_pitch_rad = imu_rpy[0]
        true_pitch_deg = math.degrees(true_pitch_rad)

        pitch_angle, filtered_rate = fusion.update(gyro_vals[0], accel_vals[1], accel_vals[2], dt)
        pitch_deg = math.degrees(pitch_angle)
        pitch_rate_dps = math.degrees(filtered_rate)

        # Odometri
        filtered_vel = controller.update_odometry(omega_left, omega_right, dt)

        # Ilac kutusu denetimi
        if aspirin_node:
            box_pos = aspirin_node.getPosition()
            rob_pos = robot_node.getPosition()
            dw = [box_pos[0] - rob_pos[0], box_pos[1] - rob_pos[1], box_pos[2] - rob_pos[2]]
            R = robot_node.getOrientation()
            dx_loc = R[0] * dw[0] + R[3] * dw[1] + R[6] * dw[2]
            dy_loc = R[1] * dw[0] + R[4] * dw[1] + R[7] * dw[2]
            dz_loc = R[2] * dw[0] + R[5] * dw[1] + R[8] * dw[2]
            if abs(dx_loc) > 0.040 or abs(dy_loc) > 0.020 or dz_loc < 0.100 or dz_loc > 0.190:
                if not box_dropped:
                    box_dropped = True
                    print(f"[HATA] Ilac kutusu hazneden dustu! Yerel sapma: dx={dx_loc:+.3f}m, dy={dy_loc:+.3f}m, dz={dz_loc:+.3f}m")

        # Girdileri oku
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
            target_velocity = 0.0
            turn_command = 0.0
            box_dropped = False
            continue

        # Devrilme emniyeti
        if abs(true_pitch_deg) > cfg.FALL_LIMIT_DEG:
            left_motor.setTorque(0.0)
            right_motor.setTorque(0.0)
            if step_count % 50 == 0:
                print(f"[EMNIYET] Robot devrildi! Egim: {true_pitch_deg:.1f} deg. Tork sifirlandi.")
            if max_test_steps > 0 and step_count >= max_test_steps:
                telemetry.close()
                robot.simulationQuit(1)
                break
            continue

        target_velocity = 0.96 * target_velocity + 0.04 * desired_v
        turn_command = 0.90 * turn_command + 0.10 * desired_turn

        # Kontrolcü Gerilim / PWM Hesaplaması
        v_left_raw, v_right_raw, _, _ = controller.compute(
            pitch_angle, filtered_rate, target_velocity, desired_v, turn_command, dt
        )
        # Ters EMK İleri Beslemesi (Back-EMF Feedforward): Motor dönüş hızının gerilim kaybını kompanse eder
        v_left = v_left_raw + 0.85 * motor_model.ke_wheel * omega_left
        v_right = v_right_raw + 0.85 * motor_model.ke_wheel * omega_right
        v_left = clamp(v_left, -motor_model.v_battery, motor_model.v_battery)
        v_right = clamp(v_right, -motor_model.v_battery, motor_model.v_battery)
        pwm_left = motor_model.voltage_to_pwm(v_left)
        pwm_right = motor_model.voltage_to_pwm(v_right)

        # Apply the quantized PWM actually sent to the driver.
        v_left = motor_model.pwm_to_voltage(pwm_left)
        v_right = motor_model.pwm_to_voltage(pwm_right)

        # DC Motor Fizik Modeli: Voltaj ve Hızdan Tekerlek Torku Üretimi
        tau_left, i_left = motor_model.compute_torque(v_left, omega_left)
        tau_right, i_right = motor_model.compute_torque(v_right, omega_right)

        # Dış dürtme
        if push_remaining_steps > 0:
            push_remaining_steps -= 1
            try:
                robot_node.addForceWithOffset([0.0, push_force_y, 0.0], [0.0, 0.0, 0.08], True)
            except Exception:
                try:
                    robot_node.addForce([0.0, push_force_y, 0.0], True)
                except Exception:
                    pass

        # Tekerleklere Fiziksel Tork Uygula
        left_motor.setTorque(tau_left)
        right_motor.setTorque(tau_right)

        # Telemetri
        telemetry.log(
            sim_time, step_count, true_pitch_deg, pitch_rate_dps,
            filtered_vel, target_velocity, v_left, v_right,
            pitch_est_deg=pitch_deg, pos_m=controller.robot_pos
        )

        # Konsol Bilgilendirmesi (200 ms)
        if step_count % 20 == 0:
            print(f"[t={sim_time:5.2f}s] Egim: {true_pitch_deg:+6.2f} deg | Pos: {controller.robot_pos:+6.3f} m | Hiz: {filtered_vel:+5.2f} m/s | Volt: {v_left:+5.1f}V (PWM: {pwm_left:+4d}) | Tork: {tau_left:+5.2f} Nm (Akim: {i_left:+4.2f}A)")

        if max_test_steps > 0 and step_count >= max_test_steps:
            print(f"[TEST TAMAMLANDI] {step_count} adim ({sim_time:.2f} s). Son Egim: {true_pitch_deg:+.2f} deg.")
            telemetry.close()
            robot.simulationQuit(0)
            break

    telemetry.close()

if __name__ == "__main__":
    main()


