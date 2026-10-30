"""
YTU EEF CDTP - Robot ve Simülasyon Konfigürasyonu
Fiziksel boyutlar, sensör gürültüleri, kontrolör kazançları ve sınırlar.
"""

from dataclasses import dataclass

@dataclass(frozen=True)
class RobotConfig:
    # Mekanik Parametreler
    WHEEL_RADIUS: float = 0.0425   # 85 mm çap / 2 = 0.0425 m
    TRACK_WIDTH: float = 0.170     # İki tekerlek arası mesafe 170 mm
    MAX_VELOCITY: float = 30.0     # rad/s azami motor hızı (~286 RPM)
    FALL_LIMIT_DEG: float = 45.0   # Devrilme emniyet sınırı (derece)

    # İç Döngü (Eğim Açısı PID - 100 Hz)
    KP_PITCH: float = 85.0         # Açı oransal kazancı
    KD_PITCH: float = 2.40         # Açı türevsel kazancı

    # Sürüş Döngüsü (Hız Hatası -> Hedef Eğim)
    KP_VEL: float = 0.06           # Hız hatası hedef eğim kazancı
    MAX_PITCH_TARGET: float = 0.025 # ~1.4 derece azami hedef eğim açısı

    # Duruş / Konum Tutma Döngüsü (Clamping Anti-Windup)
    KP_POS: float = 0.15           # Konum hatası kazancı (rad/m)
    KI_POS: float = 0.02           # Konum integral kazancı (rad/m-s)
    MAX_POS_INT: float = 0.030     # Azami konum integrali sınırı (~1.7 derece)

    # Dönüş Kazancı (Yumuşak Diferansiyel Dönüş)
    TURN_SPEED: float = 1.2        # rad/s diferansiyel dönüş hızı (~34 derece/saniye)

    # DC Motor Ölü Bölgesi (Deadband) & Statik Sürtünme Telafisi
    DEADBAND_THRESHOLD: float = 0.05
    DEADBAND_COMPENSATION: float = 0.50
    MOTOR_DEADBAND: float = 0.60   # rad/s altındaki gerilimlerde motor dönmez

    # MPU-6050 Sensör Gürültü Seviyeleri (100 Hz Standart Sapma)
    GYRO_NOISE_STD: float = 0.002   # rad/s
    ACCEL_NOISE_STD: float = 0.012  # m/s^2

