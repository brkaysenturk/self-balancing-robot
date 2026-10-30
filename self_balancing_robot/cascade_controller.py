"""
Çift Döngülü Kaskad Denge ve Sürüş Kontrolcüsü
İç Döngü: Eğim açısı PID kontrolü
Dış Döngü: Hız / Konum takip ve Clamping Anti-Windup entegrasyonu
İleri Besleme: Taban tekerlek açısal hızı (Base Omega Feedforward)
"""

import math
try:
    from self_balancing_robot.config import RobotConfig
except ImportError:
    from config import RobotConfig

def clamp(val: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(val, max_val))

class CascadeBalanceController:
    def __init__(self, cfg: RobotConfig = RobotConfig()):
        self.cfg = cfg
        self.robot_pos = 0.0
        self.target_pos = 0.0
        self.pos_integral = 0.0
        self.is_holding_pos = True
        self.filtered_vel = 0.0

    def update_odometry(self, d_left: float, d_right: float, dt: float) -> float:
        """
        Enkoder artışlarından tekerlek çizgisel hızını ve konumunu hesaplar.
        """
        v_left_raw = -(d_left / dt) * self.cfg.WHEEL_RADIUS
        v_right_raw = -(d_right / dt) * self.cfg.WHEEL_RADIUS
        raw_vel = (v_left_raw + v_right_raw) / 2.0
        self.robot_pos += raw_vel * dt
        self.filtered_vel = 0.90 * self.filtered_vel + 0.10 * raw_vel
        return self.filtered_vel

    def compute(self, pitch_angle: float, filtered_rate: float,
                target_velocity: float, desired_v: float,
                turn_command: float, dt: float) -> tuple[float, float]:
        """
        Kaskad kontrol hesaplamasını yürütür ve motor hız komutlarını üretir.
        Dönüş: (cmd_left, cmd_right)
        """
        # 1. Dış Döngü (Konum Tutma veya Hız/Frenleme Referansı Üretimi)
        if abs(desired_v) < 0.01 and abs(target_velocity) < 0.02 and abs(self.filtered_vel) < 0.05:
            if not self.is_holding_pos:
                self.is_holding_pos = True
                self.target_pos = self.robot_pos
                self.pos_integral = 0.0

            pos_error = self.robot_pos - self.target_pos
            raw_target = (self.cfg.KP_POS * pos_error) + (self.cfg.KI_POS * self.pos_integral) + 0.15 * self.filtered_vel
            pitch_target = clamp(raw_target, -self.cfg.MAX_PITCH_TARGET, self.cfg.MAX_PITCH_TARGET)
        else:
            self.is_holding_pos = False
            self.target_pos = self.robot_pos
            self.pos_integral = 0.0
            vel_err = target_velocity - self.filtered_vel
            pitch_target = -clamp(self.cfg.KP_VEL * vel_err, -self.cfg.MAX_PITCH_TARGET, self.cfg.MAX_PITCH_TARGET)

        # 2. Taban Hız İleri Beslemesi (Base Omega Feedforward)
        base_omega = -(self.filtered_vel / self.cfg.WHEEL_RADIUS)

        # 3. İç Döngü (Eğim Açısı PID)
        pitch_error = pitch_angle - pitch_target
        u_balance = (self.cfg.KP_PITCH * pitch_error) + (self.cfg.KD_PITCH * filtered_rate)

        # 4. Clamping Anti-Windup (Koşullu Entegrasyon)
        u_unsat = base_omega + u_balance
        if self.is_holding_pos and abs(u_unsat) < self.cfg.MAX_VELOCITY and abs(pitch_target) < self.cfg.MAX_PITCH_TARGET:
            self.pos_integral += (self.robot_pos - self.target_pos) * dt
            self.pos_integral = clamp(self.pos_integral, -self.cfg.MAX_POS_INT, self.cfg.MAX_POS_INT)

        # 5. Diferansiyel Dönüş Eklenmesi
        raw_cmd_left = clamp(u_unsat - turn_command, -self.cfg.MAX_VELOCITY, self.cfg.MAX_VELOCITY)
        raw_cmd_right = clamp(u_unsat + turn_command, -self.cfg.MAX_VELOCITY, self.cfg.MAX_VELOCITY)

        # 6. Motor Ölü Bölgesi Telafisi & Simülasyonu
        cmd_left = self._process_motor_output(raw_cmd_left)
        cmd_right = self._process_motor_output(raw_cmd_right)

        return cmd_left, cmd_right

    def _process_motor_output(self, raw_cmd: float) -> float:
        """Ölü bölge telafisi uygular ve redüktör sürtünmesini filtreler."""
        # Telafi (Feedforward Compensation)
        if abs(raw_cmd) > self.cfg.DEADBAND_THRESHOLD:
            comp_cmd = raw_cmd + math.copysign(self.cfg.DEADBAND_COMPENSATION, raw_cmd)
        else:
            comp_cmd = 0.0

        # DC Motor Fiziksel Ölü Bölgesi (Stiction)
        if abs(comp_cmd) < self.cfg.MOTOR_DEADBAND:
            actual_cmd = 0.0
        else:
            actual_cmd = comp_cmd

        return clamp(actual_cmd, -self.cfg.MAX_VELOCITY, self.cfg.MAX_VELOCITY)

    def reset(self):
        self.robot_pos = 0.0
        self.target_pos = 0.0
        self.pos_integral = 0.0
        self.is_holding_pos = True
        self.filtered_vel = 0.0

