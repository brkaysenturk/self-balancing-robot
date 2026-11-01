"""
YTU CDTP - Modül B & E Diferansiyel Dönüş ve Duruş Doğrulama Testi
İki farklı gürültü tohumunda (42, 142) yerinde sola/sağa dönüş hız takibini,
komut bırakıldığında duruşu, tekerlek kaymasını ve ilaç kutusu güvenliğini doğrular.
"""

from datetime import datetime
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for path_entry in (str(ROOT), str(ROOT / "analysis/controllers/pid")):
    if path_entry not in sys.path:
        sys.path.insert(0, path_entry)

from self_balancing_robot.pid_controller import PIDController
import simulation_runner as simulation
from simulation_runner import KEYS


def main() -> None:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = ROOT / "logs/pid_tuning" / f"turn_test_{timestamp}"
    out.mkdir(parents=True, exist_ok=True)

    simulation.SCENARIOS = {
        "turns": (2200, "200:A,700:SPACE,1000:D,1500:SPACE", ""),
    }

    controller = PIDController()
    params = {key: getattr(controller, key) for key in KEYS}
    runner = simulation.Runner(out, simulation.get_webots_executable(), 1291)

    cost, metrics = runner.evaluate(params, "feedback", [42, 142])
    assert cost < 1e6, f"Dönüş simülasyonu başarısız oldu: {metrics}"

    for seed in (42, 142):
        trace = np.genfromtxt(
            out / f"feedback/seed_{seed}_turns/evaluation.csv", delimiter=",", names=True
        )
        for start, stop, target in ((4.0, 6.5, 0.6), (12.0, 14.5, -0.6)):
            mask = (trace["time"] > start) & (trace["time"] < stop)
            mean_yaw = float(np.mean(trace["yaw_rate"][mask]))
            assert abs(mean_yaw - target) < 0.16, (seed, mean_yaw, target)
            print(f"Seed {seed} | Mean yaw rate: {mean_yaw:+.4f} rad/s (target: {target:+.2f})", flush=True)

        assert np.max(np.abs(trace["yaw_rate"])) < 1.2, "Aşırı dönüş hızı (overshoot)"
        assert np.max(np.abs(trace["yaw_rate"][trace["time"] > 18.0])) < 0.08, "Dönüş sonrası durmadı"
        assert metrics[f"{seed}_turns"]["slip_over_005_fraction"] < 0.05, "Aşırı tekerlek kayması"

    print(f"PASS: {out}", flush=True)


if __name__ == "__main__":
    main()
