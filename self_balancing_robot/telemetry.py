"""
Simülasyon Telemetri Kaydedicisi
Zaman serisi telemetri verilerini logs/simulation_telemetry.csv dosyasına yazar.
"""

import os

class TelemetryLogger:
    def __init__(self, filename: str = "simulation_telemetry.csv"):
        log_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "logs"))
        os.makedirs(log_dir, exist_ok=True)
        self.filepath = os.path.join(log_dir, filename)
        self.file = None
        try:
            self.file = open(self.filepath, "w", encoding="utf-8")
            self.file.write("time_s,step,pitch_deg,pitch_rate_dps,vel_meas_mps,vel_target_mps,cmd_left_rads,cmd_right_rads\n")
        except Exception as e:
            print(f"[UYARI] Telemetri dosyasi acilamadi: {e}")

    def log(self, sim_time: float, step: int, pitch_deg: float, pitch_rate_dps: float,
            vel_meas: float, vel_target: float, cmd_left: float, cmd_right: float):
        if self.file and step % 2 == 0:
            self.file.write(f"{sim_time:.3f},{step},{pitch_deg:.2f},{pitch_rate_dps:.2f},{vel_meas:.3f},{vel_target:.3f},{cmd_left:.2f},{cmd_right:.2f}\n")
            if step % 50 == 0:
                self.file.flush()

    def close(self):
        if self.file and not self.file.closed:
            self.file.flush()
            self.file.close()

