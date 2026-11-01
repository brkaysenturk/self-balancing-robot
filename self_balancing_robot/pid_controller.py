"""
================================================================================
MODUL: KASKAD PID DENGE VE DIFERANSIYEL PI DONUS KONTROLCUSU (PID CONTROLLER)
================================================================================
Ic dongude 100 Hz aci PD kontrolu ile denge gerilimi ureten, dis dongude
hiz hatasi ve konum PI ile hedef egim acisi belirleyen, enkoder geri beslemeli
denge oncelikli PI donus ve ters EMK ileri beslemesi iceren kontrol katmanidir.
================================================================================
"""

import json
import math
import os
from pathlib import Path

from .config import CONFIG, RobotConfig
from .dc_motor_model import DCMotorModel, clamp


class PIDController:
    """
    PWM/gerilim tabanlı kaskad PID denge ve PI dönüş kontrol algoritması.
    Donanım aktarımı ve ölçüme dayalı kalibrasyon henüz yapılmadı.
    """

    def __init__(self, cfg: RobotConfig = CONFIG):
        self.cfg = cfg
        self.motor = DCMotorModel()

        # SSOT CONFIG.control altından okunan aktif kazançlar ve eşikler
        self.KP_VOLT: float = cfg.control.kp_volt
        self.KD_VOLT: float = cfg.control.kd_volt
        self.KV_VEL: float = cfg.control.kv_vel
        self.KP_POS: float = cfg.control.kp_pos
        self.KI_POS: float = cfg.control.ki_pos
        self.KP_VEL_DRIVE: float = cfg.control.kp_vel_drive
        self.MAX_PITCH_TARGET: float = cfg.control.max_pitch_target
        self.MAX_POS_INT: float = cfg.control.max_pos_int
        self.hold_cmd_threshold: float = cfg.control.hold_cmd_threshold
        self.hold_vel_threshold: float = cfg.control.hold_vel_threshold
        self.odometry_vel_alpha: float = cfg.control.odometry_vel_alpha
        self.stiction_tanh_gain: float = cfg.control.stiction_tanh_gain
        self.back_emf_ff_ratio: float = cfg.control.back_emf_ff_ratio

        # Tekrarlanabilir optimizasyon koşuları için harici parametre dosyası desteği
        parameter_file = os.environ.get("PID_PARAMS_FILE")
        if parameter_file:
            values = json.loads(Path(parameter_file).read_text(encoding="utf-8"))
            allowed = {"KP_VOLT", "KD_VOLT", "KV_VEL", "KP_POS", "KI_POS", "KP_VEL_DRIVE"}
            if set(values) != allowed:
                raise ValueError("Parameter file must contain exactly the six voltage gains")
            for key, value in values.items():
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(value)
                    or value <= 0
                ):
                    raise ValueError(f"Invalid gain: {key}")
                setattr(self, key, float(value))

        # Durum değişkenleri
        self.robot_pos: float = 0.0
        self.target_pos: float = 0.0
        self.pos_integral: float = 0.0
        self.filtered_vel: float = 0.0
        self.is_holding_pos: bool = True
        self.last_u_bal: float = 0.0
        self.last_u_vel: float = 0.0
        self.last_u_pos: float = 0.0

        # Diferansiyel dönüş kontrol değişkenleri
        self.turn_omega: float = 0.0
        self.omega_left: float = 0.0
        self.omega_right: float = 0.0
        self.KP_TURN: float = cfg.control.kp_turn
        self.KI_TURN: float = cfg.control.ki_turn
        self.turn_integral: float = 0.0
        self.MAX_TURN_VOLTS: float = cfg.control.max_turn_volts
        self.turn_integral_limit: float = cfg.control.turn_integral_limit
        self.turn_friction_tanh_gain: float = cfg.control.turn_friction_tanh_gain
        self.turn_damping_ke_ratio: float = cfg.control.turn_damping_ke_ratio

    def update_odometry(self, omega_left: float, omega_right: float, dt: float) -> float:
        """Tekerlek açısal hızlarından robotun çizgisel hız ve konumunu hesaplar."""
        self.omega_left = omega_left
        self.omega_right = omega_right
        wheel_radius = self.cfg.mechanics.wheel_radius
        v_left = -omega_left * wheel_radius
        v_right = -omega_right * wheel_radius
        raw_vel = (v_left + v_right) / 2.0
        self.turn_omega = (omega_right - omega_left) / 2.0
        self.robot_pos += raw_vel * dt
        alpha = self.odometry_vel_alpha
        self.filtered_vel = (1.0 - alpha) * self.filtered_vel + alpha * raw_vel
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
        Denge, dönüş ve ters EMK ileri beslemesiyle gerilim/PWM hesaplar.
        Her çevrimde önce update_odometry ile güncel tekerlek hızlarını verin.
        Dönüş: (v_left_volts, v_right_volts, pwm_left, pwm_right)
        """
        # 1. Dış Döngü: Kaskad Konum / Hız Hedef Eğim Açısı
        if (
            abs(desired_v) < self.hold_cmd_threshold
            and abs(target_velocity) < self.hold_vel_threshold
            and abs(self.filtered_vel) < self.hold_vel_threshold
        ):
            if not self.is_holding_pos:
                self.is_holding_pos = True
                self.target_pos = self.robot_pos
                self.pos_integral = 0.0

            pos_err = self.robot_pos - self.target_pos
            self.pos_integral = clamp(self.pos_integral + pos_err * dt, -self.MAX_POS_INT, self.MAX_POS_INT)
            raw_target = (
                (self.KP_POS * pos_err)
                + (self.KI_POS * self.pos_integral)
                + (self.KV_VEL * self.filtered_vel)
            )
            pitch_target = clamp(raw_target, -self.MAX_PITCH_TARGET, self.MAX_PITCH_TARGET)
        else:
            self.is_holding_pos = False
            self.target_pos = self.robot_pos
            self.pos_integral = 0.0
            vel_err = target_velocity - self.filtered_vel
            pitch_target = -clamp(self.KP_VEL_DRIVE * vel_err, -self.MAX_PITCH_TARGET, self.MAX_PITCH_TARGET)

        # 2. İç Döngü: Gerilim PD Denge Kontrolü
        pitch_error = pitch_angle - pitch_target
        u_balance = (self.KP_VOLT * pitch_error) + (self.KD_VOLT * filtered_rate)
        self.last_u_bal = u_balance
        self.last_u_vel = pitch_target
        self.last_u_pos = self.pos_integral

        u_volts = u_balance

        # 3. Gerilim Ölü Bölgesi / Pürüzsüz Sürtünme Telafisi (Smooth Stiction Compensation)
        u_volts += self.motor.deadband_volts * math.tanh(self.stiction_tanh_gain * u_volts)

        # 4. Enkoder Geri Beslemeli Diferansiyel PI Dönüş Kontrolü (Denge Öncelikli)
        total_resistance = self.motor.r_armature + self.motor.driver_resistance
        headroom = min(self.MAX_TURN_VOLTS, max(0.0, self.motor.v_battery - abs(u_volts)))
        max_turn_speed = self.cfg.mechanics.turn_speed
        target_turn = clamp(turn_command, -max_turn_speed, max_turn_speed)

        friction_ff = self.motor.coulomb_friction * total_resistance / self.motor.kt_wheel
        damping_ff = (
            self.turn_damping_ke_ratio * self.motor.ke_wheel
            + self.motor.viscous_friction * total_resistance / self.motor.kt_wheel
        )
        turn_volts = self.KP_TURN * (target_turn - self.turn_omega)
        turn_volts += friction_ff * math.tanh(self.turn_friction_tanh_gain * target_turn) + damping_ff * target_turn

        if abs(target_turn) < 0.001:
            self.turn_integral = 0.0
        else:
            error = target_turn - self.turn_omega
            trial_integral = clamp(
                self.turn_integral + self.KI_TURN * error * dt,
                -self.turn_integral_limit,
                self.turn_integral_limit,
            )
            if abs(turn_volts + trial_integral) <= headroom or error * (turn_volts + trial_integral) < 0:
                self.turn_integral = trial_integral

        turn_volts += self.turn_integral
        headroom = min(self.MAX_TURN_VOLTS, max(0.0, self.motor.v_battery - abs(u_volts)))
        turn_volts = clamp(turn_volts, -headroom, headroom)

        v_left = clamp(u_volts - turn_volts, -self.motor.v_battery, self.motor.v_battery)
        v_right = clamp(u_volts + turn_volts, -self.motor.v_battery, self.motor.v_battery)

        # 5. Tekerlek Başına Ters EMK İleri Beslemesi (%85 Back-EMF Feedforward)
        v_left = clamp(
            v_left + self.back_emf_ff_ratio * self.motor.ke_wheel * self.omega_left,
            -self.motor.v_battery,
            self.motor.v_battery,
        )
        v_right = clamp(
            v_right + self.back_emf_ff_ratio * self.motor.ke_wheel * self.omega_right,
            -self.motor.v_battery,
            self.motor.v_battery,
        )

        pwm_left = self.motor.voltage_to_pwm(v_left)
        pwm_right = self.motor.voltage_to_pwm(v_right)

        return v_left, v_right, pwm_left, pwm_right

    def reset(self) -> None:
        """Kontrolcü durum ve integral değişkenlerini sıfırlar."""
        self.robot_pos = 0.0
        self.target_pos = 0.0
        self.pos_integral = 0.0
        self.filtered_vel = 0.0
        self.is_holding_pos = True
        self.turn_omega = 0.0
        self.omega_left = 0.0
        self.omega_right = 0.0
        self.turn_integral = 0.0
