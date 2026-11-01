"""
================================================================================
MODUL: JGB37-520 DC MOTOR VE TB6612FNG SURUCU FIZIKSEL MODELI (DC MOTOR MODEL)
================================================================================
ESP32 PWM / gerilim komutundan tekerlek torku ve armatur akimi uretimini
modelleyen, Ters EMK (Back-EMF), armatur direnci, reduktor verimi ve
Coulomb/viskoz surtunmeyi hesaplayan fiziksel eyleyici katmanidir.
================================================================================
"""

import math

from .config import CONFIG


def clamp(val: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(val, max_val))


class DCMotorModel:
    """JGB37-520 12V DC redüktörlü motor ve TB6612FNG H-köprüsü elektromekanik modeli."""

    def __init__(
        self,
        v_battery: float = CONFIG.motor.v_battery,
        r_armature: float = CONFIG.motor.r_armature,
        gear_ratio: float = CONFIG.motor.gear_ratio,
        kt_motor: float = CONFIG.motor.kt_motor,
        ke_motor: float = CONFIG.motor.ke_motor,
        efficiency: float = CONFIG.motor.efficiency,
        coulomb_friction: float = CONFIG.motor.coulomb_friction,
        viscous_friction: float = CONFIG.motor.viscous_friction,
        max_current: float | None = None,
        driver_resistance: float = CONFIG.motor.driver_resistance,
        deadband_volts: float = CONFIG.motor.deadband_volts,
        breakaway_omega: float = CONFIG.motor.breakaway_omega,
        coulomb_tanh_gain: float = CONFIG.motor.coulomb_tanh_gain,
        max_torque: float = CONFIG.motor.max_torque,
        max_pwm: float = CONFIG.motor.max_pwm,
    ):
        if min(v_battery, r_armature, gear_ratio, kt_motor, ke_motor) <= 0:
            raise ValueError("Motor parameters must be positive")
        if driver_resistance < 0 or not 0 < efficiency <= 1:
            raise ValueError("Invalid driver resistance or efficiency")
        if max_current is not None and max_current <= 0:
            raise ValueError("Current limit must be positive or None")

        self.driver_resistance = driver_resistance
        self.v_battery = v_battery
        self.r_armature = r_armature
        self.gear_ratio = gear_ratio
        self.kt_motor = kt_motor
        self.ke_motor = ke_motor
        self.efficiency = efficiency
        self.coulomb_friction = coulomb_friction
        self.viscous_friction = viscous_friction
        self.max_current = max_current
        self.deadband_volts = deadband_volts
        self.breakaway_omega = breakaway_omega
        self.coulomb_tanh_gain = coulomb_tanh_gain
        self.max_pwm = max_pwm

        # Tekerlek miline indirgenmiş sabitler
        self.kt_wheel = self.kt_motor * self.gear_ratio * self.efficiency  # N*m / A
        self.ke_wheel = self.ke_motor * self.gear_ratio                    # V / (rad/s)
        # Webots eyleyici tavanı (patinaj sınırı değildir; kaymayı temas fiziği çözer)
        self.max_torque = max_torque

    def pwm_to_voltage(self, pwm: float, max_pwm: float | None = None) -> float:
        """ESP32 PWM değerini (-255 ile +255 arası) TB6612 ortalama motor gerilimine çevirir."""
        limit = self.max_pwm if max_pwm is None else max_pwm
        duty = clamp(pwm / limit, -1.0, 1.0)
        return duty * self.v_battery

    def voltage_to_pwm(self, voltage: float, max_pwm: float | None = None) -> int:
        """Motor gerilimini (-12V ile +12V) ESP32 8-bit işaretli PWM tam sayısına çevirir."""
        limit = self.max_pwm if max_pwm is None else max_pwm
        duty = clamp(voltage / self.v_battery, -1.0, 1.0)
        return int(round(duty * limit))

    def compute_torque(self, voltage: float, wheel_omega: float) -> tuple[float, float]:
        """
        Uygulanan gerilim (V) ve tekerlek anlık dönüş hızından (rad/s)
        mil torkunu (N*m) ve armatür akımını (A) hesaplar.
        Dönüş: (net_torque_Nm, armature_current_A)
        """
        # 1. Gerilimi batarya sınırlarına kırp
        v_app = clamp(voltage, -self.v_battery, self.v_battery)

        # 2. Ters EMK (Back-EMF)
        v_emf = self.ke_wheel * wheel_omega

        # 3. Net Armatür Gerilimi ve Akımı
        v_net = v_app - v_emf
        i_arm = v_net / (self.r_armature + self.driver_resistance)
        if self.max_current is not None:
            i_arm = clamp(i_arm, -self.max_current, self.max_current)

        # 4. Elektromanyetik Brüt Tork
        tau_gross = self.kt_wheel * i_arm

        # 5. Redüktör İç Sürtünmesi (Coulomb + Viskoz)
        tau_fric = (
            self.coulomb_friction * math.tanh(self.coulomb_tanh_gain * wheel_omega)
            + self.viscous_friction * wheel_omega
        )

        # 6. Net Çıkış Torku
        tau_net = tau_gross - tau_fric

        # Statik Ölü Bölge (Breakaway threshold)
        if abs(v_app) < self.deadband_volts and abs(wheel_omega) < self.breakaway_omega:
            tau_net = 0.0

        return tau_net, i_arm
