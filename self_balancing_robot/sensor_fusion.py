"""
================================================================================
MODUL: TAMAMLAYICI FILTRE VE SENSOR FUZYONU (SENSOR FUSION)
================================================================================
MPU-6050 jiroskop ve ivmeolcer verilerini birlestirerek gurultuden arindirilmis
govde egim acisini (%98 Gyro + %2 Accel) ve EMA filtreli acisal hizi deterministik
olarak kestiren saf filtre modülüdür.
================================================================================
"""

import math

from .config import CONFIG


class SensorFusion:
    """Deterministik tamamlayıcı filtre (%98 Jiroskop + %2 İvmeölçer) ve açısal hız EMA filtresi."""

    def __init__(
        self,
        alpha: float = CONFIG.fusion.alpha,
        rate_ema_alpha: float = CONFIG.fusion.rate_ema_alpha,
        min_accel_z_g: float = CONFIG.fusion.min_accel_z_g,
    ):
        self.alpha = alpha
        self.rate_ema_alpha = rate_ema_alpha
        self.min_accel_z_g = min_accel_z_g
        self.pitch_est = 0.0
        self.filtered_rate = 0.0
        self.initialized = False

    def update(self, gyro_x: float, accel_y: float, accel_z: float, dt: float) -> tuple[float, float]:
        """
        Sensör verilerini işler, tamamlayıcı filtre ve gyro türev filtresi uygular.
        Dönüş: (pitch_angle_rad, filtered_rate_rads)
        """
        accel_pitch = math.atan2(accel_y, accel_z) if abs(accel_z) > self.min_accel_z_g else 0.0

        if not self.initialized:
            self.pitch_est = accel_pitch
            self.filtered_rate = gyro_x
            self.initialized = True
            return self.pitch_est, self.filtered_rate

        # Tamamlayıcı Filtre (%98 Gyro integrali + %2 İvmeölçer açısı)
        self.pitch_est = self.alpha * (self.pitch_est + gyro_x * dt) + (1.0 - self.alpha) * accel_pitch

        # Jiroskop alçak geçiren filtre (EMA - 50 Hz ayrık titremeyi önler)
        self.filtered_rate = (
            (1.0 - self.rate_ema_alpha) * self.filtered_rate + self.rate_ema_alpha * gyro_x
        )

        return self.pitch_est, self.filtered_rate

    def reset(self) -> None:
        """Filtre durumunu başlangıç değerlerine sıfırlar."""
        self.pitch_est = 0.0
        self.filtered_rate = 0.0
        self.initialized = False
