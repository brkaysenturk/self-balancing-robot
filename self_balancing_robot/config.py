"""
================================================================================
MODUL: DENGE ROBOTU MERKEZI KONFIGURASYONU (SSOT - CONFIG)
================================================================================
Tum alt sistemlerin (Mekanik Geometri, Kaskad PID & PI Donus Kontrolcusu,
JGB37 DC Motor & TB6612 Surucu Fizigi, MPU-6050 Sensor Fuzyonu ve Yuk Emniyeti)
ortak fiziksel ve algoritmik parametrelerini tek bir kaynakta (Single Source of Truth)
toplayan, harici kutuphane bagimliligi olmayan saf yaprak moduldur.
================================================================================
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class MechanicsConfig:
    """Robot fiziksel geometrisi, surus hedefleri, yuk kutusu ve emniyet sinirlari."""

    wheel_radius: float = 0.0425       # 85 mm tekerlek capi / 2 = 0.0425 m
    track_width: float = 0.170         # Iki tekerlek arasi iz genisligi 170 mm
    fall_limit_deg: float = 45.0       # Devrilme emniyet kesici siniri (derece)
    drive_speed: float = 0.20          # m/s guvenli nominal surus hizi (Modul B)
    turn_speed: float = 1.20           # rad/s diferansiyel tekerlek hiz farki (~34 derece/saniye)
    payload_max_dx_m: float = 0.040    # Katman 4 kaset yuvasinda X ekseni azami sapma (m)
    payload_max_dy_m: float = 0.020    # Katman 4 kaset yuvasinda Y ekseni azami sapma (m)
    payload_min_dz_m: float = 0.100    # Katman 4 kaset yuvasinda Z ekseni alt yukseklik siniri (m)
    payload_max_dz_m: float = 0.190    # Katman 4 kaset yuvasinda Z ekseni ust yukseklik siniri (m)


@dataclass(frozen=True)
class ControlConfig:
    """Kaskad PID denge, konum tutma ve diferansiyel PI donus kontrolcu sabitleri."""

    # Ic ve Dis Dongu Kaskad PID Kazanclari (Optuna TPE dogrudan Webots optimal)
    kp_volt: float = 60.7944           # Aci kazanci (V/rad) ~1.06 V/derece
    kd_volt: float = 1.2985            # Egim acisal hizi sonumleme gerilimi (V/(rad/s))
    kv_vel: float = 0.2952             # Hizdan hedef egim sonumlemesi (rad/(m/s))
    kp_pos: float = 0.1000             # Konumdan hedef egim (rad/m)
    ki_pos: float = 0.0104             # Konum integrali kazanci (rad/(m*s))
    kp_vel_drive: float = 0.2486       # Surus ani hiz hatasi hedef acisi kazanci

    # Kaskad Denge Doyum ve Konum Tutma Esikleri
    max_pitch_target: float = 0.025    # Azami hedef egim acisi (~1.43 derece)
    max_pos_int: float = 0.030         # Clamping konum integral siniri (m*s)
    hold_cmd_threshold: float = 0.01   # Durus moduna gecis komut hiz esigi (m/s)
    hold_vel_threshold: float = 0.02   # Durus moduna gecis gercek hiz esigi (m/s)
    odometry_vel_alpha: float = 0.10   # Tekerlek odometri hizi EMA filtre katsayisi
    stiction_tanh_gain: float = 3.0    # Puruzsuz surtunme telafisi tanh egimi
    back_emf_ff_ratio: float = 0.85    # Tekerlek basina ters EMK ileri besleme orani (%85)

    # Enkoder Geri Beslemeli Diferansiyel PI Donus Kontrolu
    kp_turn: float = 0.20              # Diferansiyel tekerlek hiz hatasi kazanci (V/(rad/s))
    ki_turn: float = 1.00              # Donus integrali kazanci (V/rad)
    max_turn_volts: float = 2.00       # Donus icin ayrilan azami diferansiyel gerilim (V)
    turn_integral_limit: float = 1.50  # Donus anti-windup integral ust siniri (V)
    turn_friction_tanh_gain: float = 6.0  # Donus Coulomb surtunme ileri besleme tanh egimi
    turn_damping_ke_ratio: float = 0.15   # Donus viskoz/EMK sonumleme ileri besleme katsayisi

    # Webots / Komut Yumusatma Filtreleri
    cmd_vel_alpha: float = 0.04        # Hedef dogrusal hiz rampa EMA katsayisi
    cmd_turn_alpha: float = 0.10       # Hedef donus hizi rampa EMA katsayisi


@dataclass(frozen=True)
class MotorConfig:
    """JGB37-520 12V DC reduktorlu motor ve TB6612FNG H-koprusu elektromekanik parametreleri."""

    v_battery: float = 12.0            # 3S 18650 Li-ion nominal batarya gerilimi (V)
    r_armature: float = 4.2            # JGB37 motor sargi (armatur) direnci (Ohm)
    driver_resistance: float = 0.5     # TB6612FNG MOSFET iletim ic direnci R_ds(on) (Ohm)
    gear_ratio: float = 30.0           # 30:1 metal duz disli reduktor orani
    kt_motor: float = 0.012            # Motor tork sabiti (N*m/A)
    ke_motor: float = 0.012            # Motor ters EMK sabiti (V/(rad/s))
    efficiency: float = 0.85           # Reduktor mekanik aktarim verimi (%85)
    coulomb_friction: float = 0.035    # Mil kuru (Coulomb) surtunme torku (N*m)
    viscous_friction: float = 0.0015   # Mil viskoz surtunme katsayisi (N*m/(rad/s))
    deadband_volts: float = 0.60       # Kalkis (breakaway) gerilim olu bolgesi (V)
    breakaway_omega: float = 0.15      # Statik olu bolge tekerlek acisal hiz esigi (rad/s)
    coulomb_tanh_gain: float = 6.0     # Reduktor Coulomb surtunmesi pürüzsüz geçiş tanh katsayısı
    max_torque: float = 3.0            # Webots eyleyici tork tavani (N*m)
    max_pwm: float = 255.0             # ESP32 8-bit PWM tam olcek degeri


@dataclass(frozen=True)
class FusionConfig:
    """MPU-6050 tamamlayici filtre (%98 Gyro + %2 Accel) ve sensor gurultu parametreleri."""

    alpha: float = 0.98                # Tamamlayici filtre jiroskop agirligi (%98)
    rate_ema_alpha: float = 0.30       # Jiroskop acisal hiz EMA alcak geciren filtre katsayisi
    min_accel_z_g: float = 0.01        # atan2 tekillik koruma esigi (m/s^2)
    gyro_noise_std: float = 0.002      # 100 Hz jiroskop beyaz gurultu standart sapmasi (rad/s)
    accel_noise_std: float = 0.012     # 100 Hz ivmeolcer beyaz gurultu standart sapmasi (m/s^2)


@dataclass(frozen=True)
class HomingConfig:
    """Modul C: 38 kHz IR verici feneri ve 2x TSOP38438 diferansiyel IR yanasma parametreleri."""

    receiver_spacing_m: float = 0.085          # Katman 3 on kenarda 2x TSOP38438 IR goz araligi (85 mm)
    receiver_forward_offset_m: float = 0.0475  # Robot merkezinden on kenara IR goz mesafesi (47.5 mm)
    beacon_half_angle_deg: float = 45.0        # Sartname Modul C: 90 derece yaklasma konisi (yarim aci 45 derece)
    receiver_fov_half_deg: float = 60.0        # TSSP4038 / TSOP38438 IR alici sifirlanma yarim acisi (+/- 45 deg yarim guc, +/- 60 deg kesim)
    max_detection_range_m: float = 0.85        # Siperlikli 38 kHz IR verici efektif algilama menzili (m)
    dock_stop_distance_m: float = 0.065        # Istasyon on yuzeyine hedef yanasma/durus mesafesi (m)
    dock_align_tolerance_rad: float = 0.060    # Hizalanma tamamlanma aci toleransi (~3.4 derece)
    cruise_speed_mps: float = 0.16             # Uzak mesafe (> 30 cm) IR yaklasma hizi (m/s)
    creep_speed_mps: float = 0.08              # Yakin mesafe (<= 30 cm yaklasma bolgesi) hassas yanasma hizi (m/s)
    slowdown_distance_m: float = 0.30          # Sartname 30 cm yaricapli hassas yaklasma bolgesi siniri (m)
    kp_ir_turn: float = 2.20                   # Diferansiyel IR sinyali donus kazanci (rad/s)
    kp_bearing_turn: float = 1.80              # Yonelim acisi (bearing) donus kazanci (rad/s / rad)
    max_homing_turn_rate: float = 0.90         # IR yanasma sirasinda azami donus komutu (rad/s)
    search_turn_rate: float = 0.75             # IR isigi gorulmediginde oldugu yerde tarama (scan) donus hizi (rad/s)


@dataclass(frozen=True)
class RobotConfig:
    """Tum alt sistem konfigurasyonlarini tek bir SSOT agacinda birlestiren kok sinif."""

    mechanics: MechanicsConfig = MechanicsConfig()
    control: ControlConfig = ControlConfig()
    motor: MotorConfig = MotorConfig()
    fusion: FusionConfig = FusionConfig()
    homing: HomingConfig = HomingConfig()

    # Geriye donuk uyumluluk ve ust seviye erisim kisayollari
    @property
    def WHEEL_RADIUS(self) -> float:
        return self.mechanics.wheel_radius

    @property
    def TRACK_WIDTH(self) -> float:
        return self.mechanics.track_width

    @property
    def FALL_LIMIT_DEG(self) -> float:
        return self.mechanics.fall_limit_deg

    @property
    def DRIVE_SPEED(self) -> float:
        return self.mechanics.drive_speed

    @property
    def TURN_SPEED(self) -> float:
        return self.mechanics.turn_speed

    @property
    def GYRO_NOISE_STD(self) -> float:
        return self.fusion.gyro_noise_std

    @property
    def ACCEL_NOISE_STD(self) -> float:
        return self.fusion.accel_noise_std


CONFIG = RobotConfig()
