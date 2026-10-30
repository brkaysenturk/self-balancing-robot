"""
YTU EEF CDTP - Tamamlayıcı Filtre & Sensör Füzyonu
MPU-6050 jiroskop ve ivmeölçer verilerini birleştirerek gürültüden arındırılmış gövde eğim açısını kestirir.
ESP32 donanım firmware algoritmasıyla birebir uyumludur.
"""

import math
import random

class SensorFusion:
    def __init__(self, gyro_noise_std: float = 0.002, accel_noise_std: float = 0.012, alpha: float = 0.98):
        self.gyro_noise_std = gyro_noise_std
        self.accel_noise_std = accel_noise_std
        self.alpha = alpha
        self.pitch_est = 0.0
        self.filtered_rate = 0.0
        self.initialized = False

    def update(self, gyro_x: float, accel_y: float, accel_z: float, dt: float) -> tuple[float, float]:
        """
        Sensör verilerini işler, tamamlayıcı filtre ve gyro türev filtresi uygular.
        Dönüş: (pitch_angle_rad, filtered_rate_rads)
        """
        # Gerçekçi Gauss beyaz gürültüsü enjeksiyonu
        noisy_rate = gyro_x + random.gauss(0.0, self.gyro_noise_std)
        noisy_ay = accel_y + random.gauss(0.0, self.accel_noise_std)
        noisy_az = accel_z + random.gauss(0.0, self.accel_noise_std)

        # İvmeölçer statik eğim açısı
        accel_pitch = math.atan2(noisy_ay, noisy_az) if abs(noisy_az) > 0.01 else 0.0

        if not self.initialized:
            self.pitch_est = accel_pitch
            self.filtered_rate = noisy_rate
            self.initialized = True
            return self.pitch_est, self.filtered_rate

        # Tamamlayıcı Filtre (%98 Gyro integrali + %2 İvmeölçer açısı)
        self.pitch_est = self.alpha * (self.pitch_est + noisy_rate * dt) + (1.0 - self.alpha) * accel_pitch

        # Jiroskop alçak geçiren filtre (EMA - 50 Hz ayrık titremeyi önler)
        self.filtered_rate = 0.70 * self.filtered_rate + 0.30 * noisy_rate

        return self.pitch_est, self.filtered_rate

    def reset(self):
        self.pitch_est = 0.0
        self.filtered_rate = 0.0
        self.initialized = False

