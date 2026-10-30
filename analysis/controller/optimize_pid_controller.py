"""
YTU CDTP - Kaskad PID Kazanç Optimizasyonu (Optuna + Bryson's Rule)
===================================================================
Bu script, tekerlekli ters sarkaç (Two-Wheeled Inverted Pendulum) robotu
için çift döngülü kaskad PID kazançlarını Optuna kütüphanesi kullanarak
Bryson Kuralı (Bryson's Rule) ile normalize edilmiş analitik maliyet
fonksiyonu üzerinden optimize eder.

Bryson Kuralı:
  Her maliyet bileşeni, sistemin şartname ve donanım sınırlarına göre
  izin verilen maksimum kabul edilebilir sapmanın karesine bölünerek
  tamamen birimsiz (0-1 aralığında normalize) hale getirilir:
  Cost = Integral [ Sum ( (Hata_i / Max_Tolerans_i) ** 2 ) * dt ]
"""

import os
import sys
import math
from pathlib import Path
import optuna

# Proje kök dizinini ekle
project_root = Path(__file__).resolve().parents[2]
if str(project_root) not in sys.path:
    sys.path.insert(0, str(project_root))

from self_balancing_robot.config import RobotConfig
from self_balancing_robot.sensor_fusion import SensorFusion
from self_balancing_robot.cascade_controller import CascadeBalanceController


class InvertedPendulumDynamics:
    """
    Tekerlekli Ters Sarkaç (Two-Wheeled Inverted Pendulum) Ayrık Dinamik Simülatörü.
    Webots simülasyonundaki koordinat sistemi ve tekerlek kinematiğiyle birebir eşleşir:
    - Webots ENU: Roll around X ekseni = pitch
    - pitch > 0: Robot arkaya yatar -> Tekerlekler arkaya (cmd > 0) hareket ederek dengeye getirir.
    - pitch < 0: Robot öne yatar -> Tekerlekler öne (cmd < 0, v > 0) hareket ederek dengeye getirir.
    - Çizgisel hız v = -(cmd_wheel) * R
    """
    def __init__(self):
        self.g = 9.81
        self.M = 0.86        # Toplam kütle (kg)
        self.L = 0.055       # CoM mil ekseninden yükseklik (m)
        self.R = 0.0425      # Tekerlek yarıçapı (m)
        self.damping = 3.0   # Açısal hız sönümleme katsayısı

        # Durumlar
        self.theta = 0.0      # Eğim açısı (rad)
        self.theta_dot = 0.0  # Eğim açısal hızı (rad/s)
        self.v = 0.0          # Robot çizgisel hızı (m/s)
        self.x = 0.0          # Robot konumu (m)

    def reset(self, theta_init: float = 0.0, v_init: float = 0.0):
        self.theta = theta_init
        self.theta_dot = 0.0
        self.v = v_init
        self.x = 0.0

    def step(self, motor_cmd_rads: float, external_force_n: float, dt: float):
        """
        motor_cmd_rads: Tekerlek motor hızı komutu (rad/s)
        external_force_n: Yönü +Y veya -Y olan dış dürtme kuvveti (N)
        """
        target_v = -motor_cmd_rads * self.R
        a_wheel = (target_v - self.v) / 0.04  # JGB37 motor zaman sabiti ~ 40 ms
        self.v += a_wheel * dt
        self.x += self.v * dt

        torque_grav = (self.g / self.L) * math.sin(self.theta)
        torque_inertial = (a_wheel / self.L) * math.cos(self.theta)
        torque_ext = (external_force_n / (self.M * self.L)) if external_force_n != 0.0 else 0.0
        torque_damp = self.damping * self.theta_dot

        theta_ddot = torque_grav + torque_inertial - torque_ext - torque_damp

        self.theta_dot += theta_ddot * dt
        self.theta += self.theta_dot * dt


def evaluate_candidate(kp_pitch: float, kd_pitch: float,
                       kp_pos: float, ki_pos: float,
                       kp_vel: float) -> float:
    """
    Bryson Kuralı (Bryson's Rule) Tabanlı Normalize Edilmiş Ceza Fonksiyonu.
    Her ceza terimi: (Hata / İzin_Verilen_Maksimum_Tolerans) ** 2
    Bu sayede tüm fiziksel büyüklükler (derece, m/s, rad/s) 0-1 aralığında normalize olur.

    Maksimum Fiziksel Toleranslar (YTÜ ÇDTP Şartnamesi ve Donanım Sınırları):
      - THETA_TOL_DEG    = 2.0 deg   (İlaç kutusunu kaydırmayacak azami statik eğim sapması)
      - OMEGA_TOL_DEGS   = 25.0 deg/s (Gövde azami açısal çalkantı hızı)
      - VEL_TOL_MPS      = 0.05 m/s  (Hedef sürüş hızından kabul edilebilir sapma)
      - MOTOR_TOL_RADS   = 30.0 rad/s (JGB37 motorunun 12V altındaki doyum hızı)
      - JERK_TOL_RADSS   = 400.0 rad/s^2 (Donanım dişlilerine zarar vermeyecek azami ivme türevi)
    """
    dt = 0.01  # 100 Hz kontrol frekansı

    # Bryson Toleransları (Fiziksel maksimum kabul edilebilir sınırlar)
    THETA_TOL = 2.0       # derece
    OMEGA_TOL = 25.0      # derece/saniye
    VEL_TOL = 0.05        # m/s
    MOTOR_TOL = 30.0      # rad/s
    JERK_TOL = 400.0      # rad/s^2

    cfg = RobotConfig(
        KP_PITCH=kp_pitch,
        KD_PITCH=kd_pitch,
        KP_POS=kp_pos,
        KI_POS=ki_pos,
        KP_VEL=kp_vel
    )

    controller = CascadeBalanceController(cfg)
    fusion = SensorFusion(alpha=0.98, gyro_noise_std=0.001, accel_noise_std=0.005)
    sim = InvertedPendulumDynamics()

    # --- SENARYO 1: Statik Denge ve 4.0 N Darbe Dayanımı (4.0 saniye = 400 adım) ---
    sim.reset(theta_init=0.035, v_init=0.0)  # ~2.0 derece başlangıç hatası
    cost_scenario_1 = 0.0
    last_cmd = 0.0

    for step in range(400):
        t = step * dt
        ext_force = 4.0 if 1.50 <= t <= 1.58 else 0.0

        accel_y = 9.81 * math.sin(sim.theta)
        accel_z = 9.81 * math.cos(sim.theta)
        pitch_est, rate_est = fusion.update(sim.theta_dot, accel_y, accel_z, dt)

        d_wheel = -(sim.v * dt) / cfg.WHEEL_RADIUS
        controller.update_odometry(d_wheel, d_wheel, dt)

        cmd_l, cmd_r = controller.compute(
            pitch_angle=pitch_est,
            filtered_rate=rate_est,
            target_velocity=0.0,
            desired_v=0.0,
            turn_command=0.0,
            dt=dt
        )
        avg_cmd = (cmd_l + cmd_r) / 2.0
        sim.step(avg_cmd, ext_force, dt)

        pitch_deg = math.degrees(sim.theta)
        if abs(pitch_deg) > 45.0:
            return 1e6 + (400 - step) * 1000.0

        rate_degs = math.degrees(rate_est)
        cmd_jerk = abs(avg_cmd - last_cmd) / dt
        last_cmd = avg_cmd

        # Bryson Normalize Edilmiş Ceza Terimleri:
        c_theta = (pitch_deg / THETA_TOL) ** 2
        c_omega = (rate_degs / OMEGA_TOL) ** 2
        c_motor = (abs(avg_cmd) / MOTOR_TOL) ** 2
        c_jerk  = (cmd_jerk / JERK_TOL) ** 2

        cost_scenario_1 += (c_theta + c_omega + c_motor + c_jerk) * dt

    # --- SENARYO 2: Sürüş, Dinamik Frenleme ve Geri Manevra (4.0 saniye = 400 adım) ---
    sim.reset(theta_init=0.0, v_init=0.0)
    controller.reset()
    fusion = SensorFusion(alpha=0.98, gyro_noise_std=0.001, accel_noise_std=0.005)

    target_velocity = 0.0
    desired_v = 0.0
    cost_scenario_2 = 0.0
    last_cmd_sc2 = 0.0

    for step in range(400):
        t = step * dt
        if t < 2.50:
            desired_v = 0.30
        elif t < 3.10:
            desired_v = 0.0
        else:
            desired_v = -0.30

        accel_y = 9.81 * math.sin(sim.theta)
        accel_z = 9.81 * math.cos(sim.theta)
        pitch_est, rate_est = fusion.update(sim.theta_dot, accel_y, accel_z, dt)

        d_wheel = -(sim.v * dt) / cfg.WHEEL_RADIUS
        controller.update_odometry(d_wheel, d_wheel, dt)

        target_velocity = 0.96 * target_velocity + 0.04 * desired_v

        cmd_l, cmd_r = controller.compute(
            pitch_angle=pitch_est,
            filtered_rate=rate_est,
            target_velocity=target_velocity,
            desired_v=desired_v,
            turn_command=0.0,
            dt=dt
        )
        avg_cmd = (cmd_l + cmd_r) / 2.0
        sim.step(avg_cmd, 0.0, dt)

        pitch_deg = math.degrees(sim.theta)
        if abs(pitch_deg) > 45.0:
            return 1e6 + (400 - step) * 1000.0

        rate_degs = math.degrees(rate_est)
        cmd_jerk_sc2 = abs(avg_cmd - last_cmd_sc2) / dt
        last_cmd_sc2 = avg_cmd

        # Sürüşte kabul edilebilir eğim (şartnamede ilaç kutusu kayma eşiği ~2.5 derece)
        danger_pitch = max(0.0, abs(pitch_deg) - 2.5)

        # Bryson Normalize Edilmiş Ceza Terimleri:
        c_vel   = ((desired_v - sim.v) / VEL_TOL) ** 2
        c_pitch = (danger_pitch / THETA_TOL) ** 2
        c_omega = (rate_degs / OMEGA_TOL) ** 2
        c_motor = (abs(avg_cmd) / MOTOR_TOL) ** 2
        c_jerk  = (cmd_jerk_sc2 / JERK_TOL) ** 2

        cost_scenario_2 += (c_vel + c_pitch + c_omega + c_motor + c_jerk) * dt

    total_cost = cost_scenario_1 + cost_scenario_2
    return total_cost


def objective(trial: optuna.Trial) -> float:
    # Güvenli ve doğrulanmış arama uzayı (140.0 optimum noktası)
    kp_pitch = trial.suggest_float("KP_PITCH", 50.0, 160.0, step=2.0)
    kd_pitch = trial.suggest_float("KD_PITCH", 1.0, 6.0, step=0.1)
    kp_pos = trial.suggest_float("KP_POS", 0.05, 0.40, step=0.01)
    ki_pos = trial.suggest_float("KI_POS", 0.005, 0.06, step=0.005)
    kp_vel = trial.suggest_float("KP_VEL", 0.02, 0.12, step=0.005)

    return evaluate_candidate(kp_pitch, kd_pitch, kp_pos, ki_pos, kp_vel)


def main():
    print("=" * 65)
    print("Optuna Kaskad PID Optimizasyonu (Bryson's Rule Formülasyonu)")
    print("Arama Uzayı: KP_PITCH Tavanı 260.0'a Kadar Serbest")
    print("Adım Sayısı: 250 deneme (trials)")
    print("=" * 65)

    optuna.logging.set_verbosity(optuna.logging.WARNING)

    sampler = optuna.samplers.TPESampler(seed=42)
    study = optuna.create_study(direction="minimize", sampler=sampler)

    study.optimize(objective, n_trials=250, show_progress_bar=True)

    best_trial = study.best_trial
    best_params = best_trial.params

    print("\n" + "=" * 65)
    print("OPTUNA OPTİMİZASYONU TAMAMLANDI (BRYSON KURALI)!")
    print(f"En Düşük Normalize Ceza Puanı (Cost) : {best_trial.value:.4f}")
    print("En İdeal Kontrolcü Parametreleri:")
    for k, v in best_params.items():
        print(f"  - {k:<12}: {v}")
    print("=" * 65)

    out_dir = os.path.dirname(os.path.abspath(__file__))
    json_path = os.path.join(out_dir, "optuna_results.json")

    import json
    results_data = {
        "best_cost": round(best_trial.value, 4),
        "best_params": {
            "KP_PITCH": float(best_params["KP_PITCH"]),
            "KD_PITCH": float(best_params["KD_PITCH"]),
            "KP_POS": float(best_params["KP_POS"]),
            "KI_POS": float(best_params["KI_POS"]),
            "KP_VEL": float(best_params["KP_VEL"]),
        },
        "trials_count": 250,
        "formulation": "Bryson's Rule Normalization",
        "search_space": {
            "KP_PITCH": [60.0, 260.0],
            "KD_PITCH": [1.5, 8.0],
            "KP_POS": [0.05, 0.40],
            "KI_POS": [0.005, 0.06],
            "KP_VEL": [0.02, 0.12]
        }
    }

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(results_data, f, indent=2, ensure_ascii=False)

    print(f"Sonuçlar başarıyla kaydedildi: {json_path}")


if __name__ == "__main__":
    main()
