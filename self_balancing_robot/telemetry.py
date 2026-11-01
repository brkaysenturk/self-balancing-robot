"""
================================================================================
MODUL: ZAMAN SERISI TELEMETRI KAYDEDICISI (TELEMETRY LOGGER)
================================================================================
Simulasyon ve kontrol dongusu verilerini logs/ dizini altindaki CSV dosyasina
kaydeder:
- pitch_deg     : Webots fizik motorundaki gercek govde acisi (Ground Truth)
- pitch_est_deg : Tamamlayici filtre tarafindan kestirilen egim acisi
- pos_m         : Robotun katettigi net odometri mesafesi (metre)
================================================================================
"""

import os
from pathlib import Path


class TelemetryLogger:
    """Zaman serisi CSV telemetri kayıt yöneticisi."""

    def __init__(self, filename: str = "simulation_telemetry.csv", command_unit: str = "volts"):
        default_log_dir = Path(__file__).resolve().parent.parent / "logs"
        file_path = Path(filename)
        if not file_path.is_absolute():
            file_path = default_log_dir / file_path
        os.makedirs(file_path.parent, exist_ok=True)
        self.filepath = str(file_path)
        self.file = None
        try:
            self.file = open(self.filepath, "w", encoding="utf-8")
            header = (
                f"time_s,step,pitch_deg,pitch_est_deg,pitch_rate_dps,"
                f"vel_meas_mps,vel_target_mps,cmd_left_{command_unit},cmd_right_{command_unit},pos_m\n"
            )
            self.file.write(header)
        except Exception as exc:
            print(f"[UYARI] Telemetri dosyasi acilamadi: {exc}")

    def log(
        self,
        sim_time: float,
        step: int,
        pitch_deg: float,
        pitch_rate_dps: float,
        vel_meas: float,
        vel_target: float,
        cmd_left: float,
        cmd_right: float,
        pitch_est_deg: float | None = None,
        pos_m: float = 0.0,
    ) -> None:
        if self.file and step % 2 == 0:
            est_val = pitch_deg if pitch_est_deg is None else pitch_est_deg
            line = (
                f"{sim_time:.3f},{step},{pitch_deg:.2f},{est_val:.2f},{pitch_rate_dps:.2f},"
                f"{vel_meas:.3f},{vel_target:.3f},{cmd_left:.2f},{cmd_right:.2f},{pos_m:.3f}\n"
            )
            self.file.write(line)
            if step % 50 == 0:
                self.file.flush()

    def close(self) -> None:
        if self.file and not self.file.closed:
            self.file.flush()
            self.file.close()
