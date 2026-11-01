"""
YTU CDTP - Modül B & E Sürüş, Duruş ve Geri Sürüş Doğrulama Testi
Şartname mesafesi (0.8 m - 1.2 m), frenleme sonrası 3 saniye hareketsiz duruş,
geri sürüş ve dinamik ilaç kutusu güvenliğini doğrudan Webots fizik motorunda doğrular.
"""

from datetime import datetime
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for path_entry in (str(ROOT), str(ROOT / "analysis/controllers/pid")):
    if path_entry not in sys.path:
        sys.path.insert(0, path_entry)

from self_balancing_robot import PIDController
import simulation_runner as simulation
from simulation_runner import KEYS


def main() -> None:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = ROOT / "logs/pid_tuning" / f"driving_test_{timestamp}"
    out.mkdir(parents=True, exist_ok=True)

    simulation.SCENARIOS = {
        "driving": (2600, "200:W,650:SPACE,1600:S,2100:SPACE", ""),
    }

    controller = PIDController()
    params = {key: getattr(controller, key) for key in KEYS}
    runner = simulation.Runner(out, simulation.get_webots_executable(), 1291)

    cost, metrics = runner.evaluate(params, "pid", [42])
    assert cost < 1e6, f"Simülasyon başarısız oldu: {metrics}"

    trace = np.genfromtxt(out / "pid/seed_42_driving/evaluation.csv", delimiter=",", names=True)
    stop_window = (trace["time"] >= 13.0) & (trace["time"] < 16.0)

    distance = float(np.mean(trace["position"][stop_window]) - trace["position"][0])
    forward_vel = float(np.mean(trace["velocity"][(trace["time"] > 3.0) & (trace["time"] < 6.0)]))
    reverse_vel = float(np.mean(trace["velocity"][(trace["time"] > 17.0) & (trace["time"] < 20.0)]))

    print(
        f"Distance: {distance:.4f} m | Forward: {forward_vel:.4f} m/s | Reverse: {reverse_vel:.4f} m/s",
        flush=True,
    )

    assert 0.8 <= distance <= 1.2, f"Mesafe şartname sınırları dışında: {distance:.4f} m"
    assert forward_vel > 0.05 and reverse_vel < -0.05, "İleri/geri sürüş hızı yetersiz"
    assert np.mean(np.abs(trace["velocity"][stop_window])) < 0.02, "Duruş ortalama hızı yüksek"
    assert np.max(np.abs(trace["velocity"][stop_window])) < 0.04, "Duruş azami hızı yüksek"
    print(f"PASS: {out}", flush=True)


if __name__ == "__main__":
    main()
