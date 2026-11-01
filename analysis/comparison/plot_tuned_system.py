#!/usr/bin/env python3
"""
================================================================================
MODUL: WEBOTS DIJITAL IKIZ VE SISTEM DINAMIGI GRAFIK MOTORU (PLOT TUNED SYSTEM)
================================================================================
Bu betik; Webots R2025a fizik motorunu dogrudan baslatarak (veya mevcut dogrulama
izlerini kullanarak) iki tekerlekli denge robotunun tum sartname modullerini
(Modul A: 30s Statik Denge ve Durtme Toparlanmasi, Modul B: 0.8-1.2 m Surus ve
3s Durus, Diferansiyel PI Donus, Modul E: Ilac Kutusu Emniyeti ve JGB37/MPU-6050
Alt Sistem Karakteristikleri) 300 DPI akademik makale kalitesinde gorsellestirir.

URETILEN GRAFIKLER (analysis/figures/python/):
- python_fig1_static_balance_and_push_recovery.png
- python_fig2_driving_and_position_hold.png
- python_fig3_differential_turning_and_slip.png
- python_fig4_sensor_fusion_and_motor_characteristics.png
- python_fig5_ir_homing_docking_response.png
================================================================================
"""

import csv
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile

import matplotlib.pyplot as plt
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
PID_DIR = PROJECT_ROOT / "analysis" / "controllers" / "pid"
for p in (str(PROJECT_ROOT), str(PID_DIR)):
    if p not in sys.path:
        sys.path.insert(0, p)

from self_balancing_robot import CONFIG, DCMotorModel, IRHomingController, PIDController
import simulation_runner as simulation
from simulation_runner import KEYS

FIG_DIR = PROJECT_ROOT / "analysis" / "figures" / "python"
FIG_DIR.mkdir(parents=True, exist_ok=True)

# Akademik grafik stil ayarlari (300 DPI, yuksek kontrast)
plt.rcParams["font.sans-serif"] = "DejaVu Sans"
plt.rcParams["axes.edgecolor"] = "#333333"
plt.rcParams["axes.linewidth"] = 1.0
plt.rcParams["grid.color"] = "#E0E0E0"
plt.rcParams["grid.linestyle"] = "--"
plt.rcParams["grid.alpha"] = 0.75

COLOR_PRIMARY = "#0055A5"    # Guclu Mavi (Gercek Fizik / Konum)
COLOR_SECONDARY = "#27AE60"  # Yesil (Filtre Kestirimi / Sag Tekerlek)
COLOR_ACCENT = "#D9534F"     # Kirmizi (Limit / Durtme / Hedef)
COLOR_ACTUATOR = "#E67E22"   # Turuncu (Motor Gerilimi / Tork)
COLOR_PURPLE = "#8E44AD"     # Mor (Acisal Hiz / Kayma)
COLOR_REF = "#222222"        # Koyu Gri / Siyah (Referans)


def safe_savefig(fig: plt.Figure, path: Path, dpi: int = 300) -> None:
    """Grafikleri cakisik dosya kilitlerine takilmadan guvenle kaydeder."""
    try:
        fig.savefig(str(path), dpi=dpi, bbox_inches="tight", facecolor="white")
    except Exception:
        tmp = path.with_name(path.stem + "_tmp" + path.suffix)
        fig.savefig(str(tmp), dpi=dpi, bbox_inches="tight", facecolor="white")
        try:
            os.replace(str(tmp), str(path))
        except Exception:
            pass
    plt.close(fig)


def load_csv_columns(path: Path) -> dict[str, np.ndarray]:
    """CSV dosyasini sutun bazli NumPy dizilerine yukler."""
    with path.open(encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def run_webots_scenarios() -> dict[str, dict[str, dict[str, np.ndarray]]]:
    """
    Webots uzerinde 3 temel dogrulama senaryosunu (push, driving, turns) kosturur
    ve hem evaluation.csv hem telemetry.csv izlerini dondurur.
    """
    webots_exe = simulation.get_webots_executable()
    controller = PIDController()
    params = {key: getattr(controller, key) for key in KEYS}

    scenarios = {
        "push": (3000, "", "500:1.5,1700:-1.5"),
        "driving": (2600, "200:W,650:SPACE,1600:S,2100:SPACE", ""),
        "turns": (2200, "200:A,700:SPACE,1000:D,1500:SPACE", ""),
    }

    old_scenarios = simulation.SCENARIOS
    simulation.SCENARIOS = scenarios
    try:
        with tempfile.TemporaryDirectory() as tmp_dir:
            out = Path(tmp_dir) / "webots_plots"
            runner = simulation.Runner(out, webots_exe, 1295)
            cost, metrics = runner.evaluate(params, "nominal", [42])
            if cost >= 1e6:
                raise RuntimeError(f"Webots senaryo kosusu basarisiz: {metrics}")

            traces: dict[str, dict[str, dict[str, np.ndarray]]] = {}
            for name in scenarios:
                run_dir = out / "nominal" / f"seed_42_{name}"
                traces[name] = {
                    "eval": load_csv_columns(run_dir / "evaluation.csv"),
                    "telem": load_csv_columns(run_dir / "telemetry.csv"),
                }

            # 4. Senaryo: 38 kHz IR Alıcı-Verici İstasyona Otonom Yanaşma (Homing)
            homing_dir = out / "nominal" / "seed_42_homing"
            homing_dir.mkdir(parents=True, exist_ok=True)
            env = os.environ.copy()
            env.update(
                PYTHONUTF8="1",
                PID_RANDOM_SEED="42",
                PID_LOG_PATH=str(homing_dir / "telemetry.csv"),
                PID_EVALUATION_PATH=str(homing_dir / "evaluation.csv"),
                WEBOTS_TEST_STEPS="2200",
                TEST_INIT_POSE="-0.10,1.12,-0.22",
                TEST_DRIVE_COMMANDS="100:H",
            )
            env.pop("TEST_PUSH_COMMANDS", None)
            subprocess.run(
                [
                    webots_exe,
                    "--port=1296",
                    "--minimize",
                    "--mode=fast",
                    "--stdout",
                    "--stderr",
                    str(runner.world),
                ],
                cwd=runner.project,
                env=env,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=60,
                check=True,
            )
            traces["homing"] = {
                "eval": load_csv_columns(homing_dir / "evaluation.csv"),
                "telem": load_csv_columns(homing_dir / "telemetry.csv"),
            }
            return traces
    finally:
        simulation.SCENARIOS = old_scenarios


def plot_fig1_static_balance_and_push(data: dict[str, dict[str, np.ndarray]]) -> Path:
    """Fig 1: 30s Statik Denge ve +/-1.5 N Dis Durtme Toparlanma Analizi (Modul A & E)."""
    ev = data["eval"]
    tm = data["telem"]
    t = ev["time"]

    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    fig.suptitle(
        "Modül A & E: 30 Saniye Statik Denge ve +/-1.5 N Dürtme Toparlanma Yanıtı (Webots R2025a)",
        fontsize=13,
        fontweight="bold",
        y=0.98,
    )

    # Panel 1: Gövde Eğim Açısı (Gerçek vs Tamamlayıcı Filtre)
    ax = axes[0]
    ax.plot(t, ev["pitch"], color=COLOR_PRIMARY, lw=1.5, label="Gerçek Eğim Açısı (pitch_deg)")
    ax.plot(
        tm["time_s"],
        tm["pitch_est_deg"],
        color=COLOR_SECONDARY,
        lw=1.1,
        ls="--",
        label="Tamamlayıcı Filtre Kestirimi (%98 Gyro + %2 Accel)",
    )
    ax.axvline(5.0, color=COLOR_ACCENT, ls=":", lw=1.2, label="+1.5 N Dürtme (t = 5.0 s, 80 ms)")
    ax.axvline(17.0, color=COLOR_PURPLE, ls=":", lw=1.2, label="-1.5 N Dürtme (t = 17.0 s, 80 ms)")
    ax.set_ylabel("Eğim Açısı (derece)", fontweight="bold")
    ax.set_title(
        f"Gövde Eğim Açısı | Tepe Sapma: {np.max(np.abs(ev['pitch'])):.2f} derece (Emniyet Sınırı: +/-15 derece)",
        fontsize=10,
    )
    ax.legend(loc="upper right", fontsize=8.5, ncol=2)
    ax.grid(True)

    # Panel 2: Açısal Hız (Gyro X)
    ax = axes[1]
    ax.plot(t, np.degrees(ev["rate"]), color=COLOR_PURPLE, lw=1.3, label="Gövde Açısal Hızı (derece/s)")
    ax.axvline(5.0, color=COLOR_ACCENT, ls=":", lw=1.2)
    ax.axvline(17.0, color=COLOR_PURPLE, ls=":", lw=1.2)
    ax.set_ylabel("Açısal Hız (derece/s)", fontweight="bold")
    ax.set_title("MPU-6050 Jiroskop Eğim Açısal Hız Yanıtı ve Sönümleme", fontsize=10)
    ax.legend(loc="upper right", fontsize=8.5)
    ax.grid(True)

    # Panel 3: Doğrusal Hız ve Konum Tutma
    ax = axes[2]
    ax.plot(t, ev["velocity"], color=COLOR_PRIMARY, lw=1.4, label="Doğrusal Hız (m/s)")
    ax.axhline(0.0, color=COLOR_REF, ls="--", lw=0.9, label="Referans Hız (0.0 m/s)")
    ax.axvline(5.0, color=COLOR_ACCENT, ls=":", lw=1.2)
    ax.axvline(17.0, color=COLOR_PURPLE, ls=":", lw=1.2)
    ax.set_ylabel("Çizgisel Hız (m/s)", fontweight="bold")
    tail_rms = float(np.sqrt(np.mean(ev["velocity"][t >= 27.0] ** 2)))
    ax.set_title(f"Dürtme Sonrası Hız Sönümleme | Son 3s RMS Hız: {tail_rms:.4f} m/s (< 0.02 m/s)", fontsize=10)
    ax.legend(loc="upper right", fontsize=8.5)
    ax.grid(True)

    # Panel 4: TB6612 Motor Gerilimi ve Tekerlek Torku
    ax = axes[3]
    ax.plot(t, ev["left_volts"], color=COLOR_ACTUATOR, lw=1.3, label="TB6612 Motor Gerilimi (V)")
    ax.axhline(CONFIG.motor.v_battery, color=COLOR_ACCENT, ls="--", lw=0.8, alpha=0.6, label="+/-12V Doyum Sınırı")
    ax.axhline(-CONFIG.motor.v_battery, color=COLOR_ACCENT, ls="--", lw=0.8, alpha=0.6)
    ax.set_ylabel("Motor Gerilimi (V)", fontweight="bold")
    ax.set_xlabel("Zaman (saniye)", fontweight="bold")
    ax.set_title(
        f"Kaskad PD Gerilim Komutu (KP_VOLT={CONFIG.control.kp_volt:.2f}, KD_VOLT={CONFIG.control.kd_volt:.2f}) | İlaç Kutusu Düşme: {int(np.max(ev['box_dropped']))}",
        fontsize=10,
    )
    ax.set_ylim(-13.5, 13.5)
    ax.legend(loc="upper right", fontsize=8.5)
    ax.grid(True)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out_path = FIG_DIR / "python_fig1_static_balance_and_push_recovery.png"
    safe_savefig(fig, out_path)
    return out_path


def plot_fig2_driving_and_hold(data: dict[str, dict[str, np.ndarray]]) -> Path:
    """Fig 2: Modül B & E İleri Sürüş (0.8-1.2 m), 3s Konum Tutma ve Geri Sürüş."""
    ev = data["eval"]
    t = ev["time"]
    pos_rel = ev["position"] - ev["position"][0]

    stop_mask = (t >= 13.0) & (t < 16.0)
    hold_dist = float(np.mean(pos_rel[stop_mask]))
    hold_vel_mean = float(np.mean(np.abs(ev["velocity"][stop_mask])))

    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    fig.suptitle(
        "Modül B & E: Kablosuz Yönlendirme (0.8–1.2 m İleri Sürüş, Frenleme/Duruş ve Geri Sürüş)",
        fontsize=13,
        fontweight="bold",
        y=0.98,
    )

    # Panel 1: Katedilen Mesafe
    ax = axes[0]
    ax.plot(t, pos_rel, color=COLOR_PRIMARY, lw=1.8, label="Robot Y Ekseni Konumu (m)")
    ax.axhspan(0.80, 1.20, color=COLOR_SECONDARY, alpha=0.15, label="Şartname Hedef Bölgesi (0.80 m – 1.20 m)")
    ax.axvspan(13.0, 16.0, color=COLOR_ACTUATOR, alpha=0.15, label="3 Saniye Hareketsiz Duruş Penceresi (13–16 s)")
    ax.set_ylabel("Mesafe (m)", fontweight="bold")
    ax.set_title(
        f"Katedilen Fiziksel Mesafe | Duruş Penceresi Ortalaması: {hold_dist:.3f} m (Şartname: 0.80–1.20 m)",
        fontsize=10,
    )
    ax.legend(loc="upper right", fontsize=8.5)
    ax.grid(True)

    # Panel 2: Doğrusal Hız Takibi
    ax = axes[1]
    ax.plot(t, ev["target"], color=COLOR_REF, ls="--", lw=1.4, label="Eldiven Hız Komutu (desired_v)")
    ax.plot(t, ev["velocity"], color=COLOR_PRIMARY, lw=1.4, label="Gerçek Gövde Çizgisel Hızı (m/s)")
    ax.axvspan(13.0, 16.0, color=COLOR_ACTUATOR, alpha=0.15)
    ax.set_ylabel("Hız (m/s)", fontweight="bold")
    ax.set_title(
        f"Doğrusal Hız Takibi ve Frenleme | Duruş Ortalama Hızı: {hold_vel_mean:.4f} m/s (< 0.02 m/s)",
        fontsize=10,
    )
    ax.legend(loc="upper right", fontsize=8.5)
    ax.grid(True)

    # Panel 3: Sürüş Esnasında Gövde Eğim Açısı
    ax = axes[2]
    ax.plot(t, ev["pitch"], color=COLOR_PURPLE, lw=1.4, label="Gövde Eğim Açısı (derece)")
    ax.axhline(0.0, color=COLOR_REF, ls=":", lw=0.9)
    ax.set_ylabel("Eğim Açısı (derece)", fontweight="bold")
    ax.set_title(
        f"Dış Döngü Hız-Eğim Kaskad Yanıtı (KP_VEL_DRIVE={CONFIG.control.kp_vel_drive:.4f}, KP_POS={CONFIG.control.kp_pos:.4f})",
        fontsize=10,
    )
    ax.legend(loc="upper right", fontsize=8.5)
    ax.grid(True)

    # Panel 4: Motor Gerilimi ve Tekerlek Kayması
    ax = axes[3]
    ax.plot(t, ev["left_volts"], color=COLOR_ACTUATOR, lw=1.2, label="Sol Motor Gerilimi (V)")
    ax.plot(t, ev["right_volts"], color=COLOR_SECONDARY, lw=1.2, ls="--", label="Sağ Motor Gerilimi (V)")
    ax.set_ylabel("Gerilim (V)", fontweight="bold")
    ax.set_xlabel("Zaman (saniye)", fontweight="bold")
    ax.set_title(
        f"Tekerlek Gerilim Komutları ve Ters EMK İleri Beslemesi (%85) | Azami Kayma Hızı: {np.max(ev['slip_speed']):.4f} m/s",
        fontsize=10,
    )
    ax.legend(loc="upper right", fontsize=8.5)
    ax.grid(True)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out_path = FIG_DIR / "python_fig2_driving_and_position_hold.png"
    safe_savefig(fig, out_path)
    return out_path


def plot_fig3_turning_and_slip(data: dict[str, dict[str, np.ndarray]]) -> Path:
    """Fig 3: Denge Öncelikli Diferansiyel PI Dönüş Kontrolü ve Kayma Analizi (Modül B)."""
    ev = data["eval"]
    t = ev["time"]

    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    fig.suptitle(
        "Modül B: Enkoder Geri Beslemeli Diferansiyel PI Dönüş Kontrolü ve Tekerlek Kayma Analizi",
        fontsize=13,
        fontweight="bold",
        y=0.98,
    )

    # Panel 1: Yaw Rate (Dönüş Açısal Hızı) Takibi
    ax = axes[0]
    ax.plot(t, ev["yaw_target"], color=COLOR_REF, ls="--", lw=1.5, label="Hedef Gövde Dönüş Hızı (rad/s)")
    ax.plot(t, ev["yaw_rate"], color=COLOR_PRIMARY, lw=1.5, label="Gerçek Gövde Dönüş Hızı (yaw_rate)")
    ax.set_ylabel("Dönüş Hızı (rad/s)", fontweight="bold")
    left_turn_mean = float(np.mean(ev["yaw_rate"][(t > 4.0) & (t < 6.5)]))
    right_turn_mean = float(np.mean(ev["yaw_rate"][(t > 12.0) & (t < 14.5)]))
    ax.set_title(
        f"Sola ve Sağa Yerinde Dönüş Takibi | Sol Ort.: {left_turn_mean:+.3f} rad/s, Sağ Ort.: {right_turn_mean:+.3f} rad/s (Hedef: +/-0.60 rad/s)",
        fontsize=10,
    )
    ax.legend(loc="upper right", fontsize=8.5)
    ax.grid(True)

    # Panel 2: Sol ve Sağ Tekerlek Diferansiyel Gerilimleri
    ax = axes[1]
    ax.plot(t, ev["left_volts"], color=COLOR_PRIMARY, lw=1.3, label="Sol Motor Gerilimi (V)")
    ax.plot(t, ev["right_volts"], color=COLOR_ACTUATOR, lw=1.3, label="Sağ Motor Gerilimi (V)")
    ax.set_ylabel("Motor Gerilimi (V)", fontweight="bold")
    ax.set_title(
        f"Diferansiyel PI Dönüş Gerilimleri (KP_TURN={CONFIG.control.kp_turn:.2f}, KI_TURN={CONFIG.control.ki_turn:.2f}, Azami Dönüş Payı={CONFIG.control.max_turn_volts:.1f} V)",
        fontsize=10,
    )
    ax.legend(loc="upper right", fontsize=8.5)
    ax.grid(True)

    # Panel 3: Dönüş Sırasında Gövde Eğim Kararlılığı
    ax = axes[2]
    ax.plot(t, ev["pitch"], color=COLOR_SECONDARY, lw=1.4, label="Gövde Eğim Açısı (derece)")
    ax.axhline(0.0, color=COLOR_REF, ls=":", lw=0.9)
    ax.set_ylabel("Eğim Açısı (derece)", fontweight="bold")
    ax.set_title(
        f"Dönüş Sırasında Denge Kararlılığı | Azami Eğim Sapması: {np.max(np.abs(ev['pitch'])):.3f} derece",
        fontsize=10,
    )
    ax.legend(loc="upper right", fontsize=8.5)
    ax.grid(True)

    # Panel 4: Tekerlek Zemin Kayma Hızı (Slip Speed)
    ax = axes[3]
    ax.plot(t, ev["slip_speed"], color=COLOR_PURPLE, lw=1.3, label="Tekerlek Zemin Kayma Hızı (m/s)")
    ax.axhline(0.05, color=COLOR_ACCENT, ls="--", lw=1.1, label="Kabul Edilebilir Kayma Eşiği (0.05 m/s)")
    ax.set_ylabel("Kayma Hızı (m/s)", fontweight="bold")
    ax.set_xlabel("Zaman (saniye)", fontweight="bold")
    slip_frac = float(np.mean(np.abs(ev["slip_speed"]) > 0.05)) * 100.0
    ax.set_title(
        f"Tekerlek-Zemin Temas Kayması (coulombFriction=1.8) | 0.05 m/s Üzeri Oran: %{slip_frac:.2f}",
        fontsize=10,
    )
    ax.legend(loc="upper right", fontsize=8.5)
    ax.grid(True)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out_path = FIG_DIR / "python_fig3_differential_turning_and_slip.png"
    safe_savefig(fig, out_path)
    return out_path


def plot_fig4_fusion_and_motor(data: dict[str, dict[str, np.ndarray]]) -> Path:
    """Fig 4: Tamamlayıcı Filtre Kestirim Doğruluğu ve JGB37-520 Elektromekanik Karakteristiği."""
    tm = data["push"]["telem"]
    t = tm["time_s"]
    est_err = tm["pitch_est_deg"] - tm["pitch_deg"]
    rmse_deg = float(np.sqrt(np.mean(est_err ** 2)))

    motor = DCMotorModel()
    omegas = np.linspace(-25.0, 25.0, 300)
    volts_list = [3.0, 6.0, 9.0, 12.0]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8.5))
    fig.suptitle(
        "Alt Sistem Doğrulaması: MPU-6050 Tamamlayıcı Filtre ve JGB37-520 + TB6612 Motor Modeli",
        fontsize=13,
        fontweight="bold",
        y=0.98,
    )

    # Panel (0,0): Tamamlayıcı Filtre vs Gerçek Eğim (Dürtme Yakın Plan)
    ax = axes[0, 0]
    zoom = (t >= 4.0) & (t <= 10.0)
    ax.plot(t[zoom], tm["pitch_deg"][zoom], color=COLOR_PRIMARY, lw=1.6, label="Gerçek Eğim (Webots IMU)")
    ax.plot(
        t[zoom],
        tm["pitch_est_deg"][zoom],
        color=COLOR_SECONDARY,
        ls="--",
        lw=1.4,
        label=f"Tamamlayıcı Filtre (alpha={CONFIG.fusion.alpha:.2f})",
    )
    ax.set_xlabel("Zaman (saniye)", fontweight="bold")
    ax.set_ylabel("Eğim Açısı (derece)", fontweight="bold")
    ax.set_title("Dürtme Anında Filtre Takibi (4.0 s – 10.0 s Yakın Plan)", fontsize=10)
    ax.legend(loc="upper right", fontsize=8.5)
    ax.grid(True)

    # Panel (0,1): Filtre Kestirim Hatası Dağılımı
    ax = axes[0, 1]
    ax.plot(t, est_err, color=COLOR_PURPLE, lw=1.0, label=f"Kestirim Hatası (RMSE = {rmse_deg:.4f} derece)")
    ax.axhline(0.0, color=COLOR_REF, ls="--", lw=0.9)
    ax.set_xlabel("Zaman (saniye)", fontweight="bold")
    ax.set_ylabel("Kestirim Hatası (derece)", fontweight="bold")
    ax.set_title(
        f"MPU-6050 Gürültüsü Altında Açı Kestirim Hatası (gyro_std={CONFIG.fusion.gyro_noise_std}, accel_std={CONFIG.fusion.accel_noise_std})",
        fontsize=10,
    )
    ax.legend(loc="upper right", fontsize=8.5)
    ax.grid(True)

    # Panel (1,0): JGB37 Tork - Açısal Hız Eğrileri (Farklı Gerilimlerde)
    ax = axes[1, 0]
    palette = [COLOR_SECONDARY, COLOR_PRIMARY, COLOR_ACTUATOR, COLOR_ACCENT]
    for v_cmd, col in zip(volts_list, palette):
        torques = [motor.compute_torque(v_cmd, w)[0] for w in omegas]
        ax.plot(omegas, torques, color=col, lw=1.6, label=f"V_uygulanan = +{v_cmd:.0f} V")
    ax.axhline(0.0, color=COLOR_REF, ls=":", lw=0.9)
    ax.axvline(0.0, color=COLOR_REF, ls=":", lw=0.9)
    ax.set_xlabel("Tekerlek Açısal Hızı (rad/s)", fontweight="bold")
    ax.set_ylabel("Net Çıkış Torku (N*m)", fontweight="bold")
    ax.set_title(
        f"JGB37-520 (30:1) Tork-Hız Karakteristiği (R_top={motor.r_armature + motor.driver_resistance:.1f} Ohm)",
        fontsize=10,
    )
    ax.legend(loc="upper right", fontsize=8.5)
    ax.grid(True)

    # Panel (1,1): Ölü Bölge (Deadband) ve Pürüzsüz Sürtünme Telafisi
    ax = axes[1, 1]
    u_raw = np.linspace(-3.0, 3.0, 400)
    u_comp = u_raw + motor.deadband_volts * np.tanh(CONFIG.control.stiction_tanh_gain * u_raw)
    tau_uncomp = [motor.compute_torque(u, 0.0)[0] for u in u_raw]
    tau_comp = [motor.compute_torque(u, 0.0)[0] for u in u_comp]
    ax.plot(u_raw, tau_uncomp, color=COLOR_ACCENT, ls="--", lw=1.5, label="Telafisiz Kalkış Torku (Ölü Bölgeli)")
    ax.plot(
        u_raw,
        tau_comp,
        color=COLOR_PRIMARY,
        lw=1.6,
        label=f"Yumuşak Telafili Tork (deadband={motor.deadband_volts:.2f} V)",
    )
    ax.set_xlabel("Ham Kontrolcü Gerilim Komutu u_balance (V)", fontweight="bold")
    ax.set_ylabel("Kalkış Mil Torku (N*m, omega=0)", fontweight="bold")
    ax.set_title("Kalkış Ölü Bölgesi (Breakaway) ve Tanh Sürtünme Telafisi", fontsize=10)
    ax.legend(loc="upper left", fontsize=8.5)
    ax.grid(True)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out_path = FIG_DIR / "python_fig4_sensor_fusion_and_motor_characteristics.png"
    safe_savefig(fig, out_path)
    return out_path


def plot_fig5_ir_homing_docking(data: dict[str, dict[str, np.ndarray]]) -> Path:
    """Fig 5: Modül C & E 38 kHz IR Alıcı-Verici İstasyona Otonom Yanaşma Analizi."""
    ev = data["eval"]
    t = ev["time"]

    # 2x TSOP38438 IR alıcılarının 30 cm yaklaşma konisi boyunca açısal ve mesafesel sinyal profili
    homing = IRHomingController()
    bearings_deg = np.linspace(-55.0, 55.0, 300)
    ir_left_curve = []
    ir_right_curve = []
    ir_diff_curve = []
    for b_deg in bearings_deg:
        r = homing.read_receivers(
            robot_xy=(0.0, 1.18),
            robot_yaw_rad=-math.radians(b_deg),
            beacon_xy=(0.0, 1.462),
        )
        ir_left_curve.append(r.ir_left)
        ir_right_curve.append(r.ir_right)
        ir_diff_curve.append(r.ir_diff)

    front_dist_cm = (1.462 - (ev["position"] + CONFIG.homing.receiver_forward_offset_m)) * 100.0
    final_dist_cm = float(front_dist_cm[-1])

    fig, axes = plt.subplots(2, 2, figsize=(12, 8.5))
    fig.suptitle(
        "Modül C & E: 38 kHz IR Alıcı-Verici (2x TSOP38438) İstasyona Otonom Yanaşma Analizi",
        fontsize=13,
        fontweight="bold",
        y=0.98,
    )

    # Panel (0,0): 2x TSOP38438 Sol/Sağ Alıcı Sinyal Şiddeti ve Fark Karakteristiği
    ax = axes[0, 0]
    ax.plot(bearings_deg, ir_left_curve, color=COLOR_PRIMARY, lw=1.6, label="Sol TSOP38438 Alıcı (X = -42.5 mm)")
    ax.plot(bearings_deg, ir_right_curve, color=COLOR_SECONDARY, lw=1.6, label="Sağ TSOP38438 Alıcı (X = +42.5 mm)")
    ax.plot(bearings_deg, ir_diff_curve, color=COLOR_ACCENT, ls="--", lw=1.5, label="Normalize Fark Sinyali (ir_diff)")
    ax.axvline(0.0, color=COLOR_REF, ls=":", lw=0.9)
    ax.axhline(0.0, color=COLOR_REF, ls=":", lw=0.9)
    ax.set_xlabel("İstasyon Yönelim Açısı / Bearing (derece)", fontweight="bold")
    ax.set_ylabel("Normalize IR Sinyal Şiddeti", fontweight="bold")
    ax.set_title("85 mm Aralıklı Çift TSOP38438 Alıcı Diferansiyel Yönelim Karakteristiği", fontsize=10)
    ax.legend(loc="lower right", fontsize=8.0)
    ax.grid(True)

    # Panel (0,1): İstasyona Kalan Mesafe ve Hedef Duruş
    ax = axes[0, 1]
    ax.plot(t, front_dist_cm, color=COLOR_PRIMARY, lw=1.8, label="Robot Ön Kenar – IR Verici Mesafesi (cm)")
    ax.axhline(
        CONFIG.homing.dock_stop_distance_m * 100.0,
        color=COLOR_ACCENT,
        ls="--",
        lw=1.3,
        label=f"Hedef Yanaşma Duruş Eşiği ({CONFIG.homing.dock_stop_distance_m * 100:.1f} cm)",
    )
    ax.axhspan(0.0, 30.0, color=COLOR_SECONDARY, alpha=0.12, label="30 cm Yaklaşma Bölgesi")
    ax.set_xlabel("Zaman (saniye)", fontweight="bold")
    ax.set_ylabel("İstasyona Mesafe (cm)", fontweight="bold")
    ax.set_title(f"30 cm Yaklaşma Konisinden Otonom Yanaşma | Son Mesafe: {final_dist_cm:.1f} cm", fontsize=10)
    ax.legend(loc="upper right", fontsize=8.0)
    ax.grid(True)

    # Panel (1,0): Yanaşma Esnasında Doğrusal Hız ve Dönüş Hızı
    ax = axes[1, 0]
    ax.plot(t, ev["velocity"], color=COLOR_PRIMARY, lw=1.5, label="Doğrusal Yaklaşma Hızı (m/s)")
    ax.plot(t, ev["yaw_rate"], color=COLOR_PURPLE, lw=1.3, ls="--", label="Gövde Dönüş Hızı yaw_rate (rad/s)")
    ax.axhline(0.0, color=COLOR_REF, ls=":", lw=0.9)
    ax.set_xlabel("Zaman (saniye)", fontweight="bold")
    ax.set_ylabel("Hız (m/s) / Dönüş (rad/s)", fontweight="bold")
    ax.set_title("Açısal Hizalanma, Hassas Yanaşma (Creep Speed) ve Otomatik Duruş", fontsize=10)
    ax.legend(loc="upper right", fontsize=8.5)
    ax.grid(True)

    # Panel (1,1): Yanaşma Esnasında Gövde Eğimi ve Motor Gerilimleri
    ax = axes[1, 1]
    ax.plot(t, ev["pitch"], color=COLOR_SECONDARY, lw=1.5, label="Gövde Eğim Açısı (derece)")
    ax.plot(t, ev["left_volts"], color=COLOR_ACTUATOR, lw=1.1, alpha=0.85, label="Sol Motor Gerilimi (V)")
    ax.plot(t, ev["right_volts"], color=COLOR_PRIMARY, lw=1.1, ls="--", alpha=0.85, label="Sağ Motor Gerilimi (V)")
    ax.set_xlabel("Zaman (saniye)", fontweight="bold")
    ax.set_ylabel("Eğim (derece) / Gerilim (V)", fontweight="bold")
    ax.set_title(
        f"Yanaşma Sırasında Denge Kararlılığı | İlaç Kutusu Düşme: {int(np.max(ev['box_dropped']))}",
        fontsize=10,
    )
    ax.legend(loc="upper right", fontsize=8.0)
    ax.grid(True)

    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out_path = FIG_DIR / "python_fig5_ir_homing_docking_response.png"
    safe_savefig(fig, out_path)
    return out_path


def main() -> None:
    print("=" * 72)
    print("Webots Dijital İkiz Doğrulama ve Akademik Grafik Üreticisi Başlatılıyor...")
    print("=" * 72)
    traces = run_webots_scenarios()

    paths = [
        plot_fig1_static_balance_and_push(traces["push"]),
        plot_fig2_driving_and_hold(traces["driving"]),
        plot_fig3_turning_and_slip(traces["turns"]),
        plot_fig4_fusion_and_motor(traces),
        plot_fig5_ir_homing_docking(traces["homing"]),
    ]
    for p in paths:
        print(f"[OK] Grafik kaydedildi: {p.relative_to(PROJECT_ROOT)}")
    print("=" * 72)


if __name__ == "__main__":
    main()

