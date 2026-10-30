"""
JGB37-520 12V DC Motor ve TB6612FNG H-Köprüsü Fiziksel Modeli
ESP32 PWM / Gerilim komutundan tekerlek torku ve akım üretimini simüle eder.
Ters EMK (Back-EMF), armatür direnci, redüktör verimi ve Coulomb/viskoz sürtünmeyi hesaplar.
"""

import math

def clamp(val: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(val, max_val))

class DCMotorModel:
    def __init__(
        self,
        v_battery: float = 12.0,
        r_armature: float = 4.2,
        gear_ratio: float = 30.0,
        kt_motor: float = 0.012,
        ke_motor: float = 0.012,
        efficiency: float = 0.85,
        coulomb_friction: float = 0.035,
        viscous_friction: float = 0.0015,
        max_current: float | None = None,
        driver_resistance: float = 0.5,
        deadband_volts: float = 0.60,
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

        # Tekerlek miline indirgenmiş sabitler
        self.kt_wheel = self.kt_motor * self.gear_ratio * self.efficiency  # N*m / A
        self.ke_wheel = self.ke_motor * self.gear_ratio                    # V / (rad/s)
        # Webots actuator ceiling, NOT a traction limit. Contact physics handles slip.
        self.max_torque = 3.0

    def pwm_to_voltage(self, pwm: float, max_pwm: float = 255.0) -> float:
        """
        ESP32 PWM değerini (-255 ile +255 arası) TB6612 ortalama motor voltajına çevirir.
        """
        duty = clamp(pwm / max_pwm, -1.0, 1.0)
        return duty * self.v_battery

    def voltage_to_pwm(self, voltage: float, max_pwm: float = 255.0) -> int:
        """
        Motor voltajını (-12V ile +12V) ESP32 PWM duty tam sayısına çevirir.
        """
        duty = clamp(voltage / self.v_battery, -1.0, 1.0)
        return int(round(duty * max_pwm))

    def compute_torque(self, voltage: float, wheel_omega: float) -> tuple[float, float]:
        """
        Uygulanan gerilim (V) ve tekerlek anlık dönüş hızından (rad/s)
        mil torkunu (N*m) ve armatür akımını (A) hesaplar.
        Dönüş: (net_torque_Nm, armature_current_A)
        """
        # 1. Voltajı batarya sınırlarına kırp
        v_app = clamp(voltage, -self.v_battery, self.v_battery)

        # 2. Ters EMK (Back-EMF)
        v_emf = self.ke_wheel * wheel_omega

        # 3. Net Armatür Gerilimi ve Akımı
        v_net = v_app - v_emf
        i_arm = v_net / (self.r_armature + self.driver_resistance)
        # Optional ideal external current regulator; TB6612 does not provide this.
        if self.max_current is not None:
            i_arm = clamp(i_arm, -self.max_current, self.max_current)

        # 4. Elektromanyetik Brüt Tork
        tau_gross = self.kt_wheel * i_arm

        # 5. Redüktör İç Sürtünmesi (Coulomb + Viskoz)
        tau_fric = (
            self.coulomb_friction * math.tanh(6.0 * wheel_omega)
            + self.viscous_friction * wheel_omega
        )

        # 6. Net Çıkış Torku
        tau_net = tau_gross - tau_fric

        # Statik Ölü Bölge (Breakaway threshold): Voltaj sürtünmeyi yenemezse ve motor duruyorsa tork üretilmez
        if abs(v_app) < self.deadband_volts and abs(wheel_omega) < 0.15:
            tau_net = 0.0

        return tau_net, i_arm
