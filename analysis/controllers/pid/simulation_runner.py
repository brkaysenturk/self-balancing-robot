"""
================================================================================
MODUL: WEBOTS SIMULASYON KOSUCUSU VE TELEMETRI PUANLAYICISI (SIMULATION RUNNER)
================================================================================
Dogrudan Webots fizik kosucusu, cok-senaryolu telemetri puanlayicisi ve arama
sinirlarini icerir. Optuna tabanli kazanc aramasi ve regresyon testleri
tarafindan ortak kullanilir.
================================================================================
"""

import csv
from datetime import datetime
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT / "tests") not in sys.path:
    sys.path.insert(0, str(ROOT / "tests"))

from test_simulation_pid import get_webots_executable as get_webots_executable

KEYS = ("KP_VOLT", "KD_VOLT", "KV_VEL", "KP_POS", "KI_POS", "KP_VEL_DRIVE")
SPACE = {
    "KP_VOLT": (40.0, 100.0),
    "KD_VOLT": (0.8, 3.0),
    "KV_VEL": (0.06, 0.30),
    "KP_POS": (0.08, 0.35),
    "KI_POS": (0.005, 0.04),
    "KP_VEL_DRIVE": (0.06, 0.25),
}
SCENARIOS = {
    "standing": (3000, "", ""),
    "push": (3000, "", "500:1.5,1700:-1.5"),
    "drive": (3500, "200:W,1200:SPACE,1800:S,2800:SPACE", ""),
}


def score_trace(path: Path, steps: int) -> tuple[float, dict]:
    """
    Simülasyon değerlendirme kaydını (evaluation.csv) doğrular ve maliyet puanını hesaplar.
    Eksik adım, devrilme, kutu düşmesi veya geçersiz sayısal değer varsa ValueError fırlatır.
    """
    with path.open(encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    if not rows or len(rows) != steps - 1 or int(rows[-1]["step"]) != steps:
        raise ValueError("Incomplete simulation trace")

    trace = {key: np.array([float(row[key]) for row in rows]) for key in rows[0]}
    if not all(np.isfinite(values).all() for values in trace.values()):
        raise ValueError("Non-finite telemetry")

    expected_steps = np.arange(2, steps + 1)
    if not np.array_equal(trace["step"], expected_steps) or not np.allclose(np.diff(trace["time"]), 0.01):
        raise ValueError("Missing or disordered samples")

    if trace["box_dropped"].any() or np.max(np.abs(trace["pitch"])) > 15.0:
        raise ValueError("Payload loss or excessive tilt")

    dv = np.diff(np.column_stack([trace["left_volts"], trace["right_volts"]]), axis=0) / 0.01
    terms = {
        "angle": float(np.mean((trace["pitch"] / 1.0) ** 2)),
        "speed": float(np.mean(((trace["velocity"] - trace["target"]) / 0.05) ** 2)),
        "effort": float(np.mean((trace["left_volts"] ** 2 + trace["right_volts"] ** 2) / (2.0 * 12.0 ** 2))),
        "slew": float(np.mean((dv / 100.0) ** 2)),
    }
    tail = trace["time"] >= (steps * 0.01 - 3.0)
    terms["settling"] = float(np.mean((trace["velocity"][tail] / 0.02) ** 2))

    cost = (
        terms["angle"]
        + terms["speed"]
        + 0.1 * terms["effort"]
        + 0.2 * terms["slew"]
        + 2.0 * terms["settling"]
    )
    metrics = {
        "terms": terms,
        "peak_pitch": float(np.max(np.abs(trace["pitch"]))),
        "tail_max_speed": float(np.max(np.abs(trace["velocity"][tail]))),
        "tail_rms_speed": float(np.sqrt(np.mean(trace["velocity"][tail] ** 2))),
        "cost": cost,
    }
    if "slip_speed" in trace:
        metrics["slip_rms"] = float(np.sqrt(np.mean(trace["slip_speed"] ** 2)))
        metrics["slip_over_005_fraction"] = float(np.mean(np.abs(trace["slip_speed"]) > 0.05))

    return cost, metrics


class Runner:
    """İzole bir çalışma kopyası üzerinde Webots senaryolarını çalıştırır ve puanlar."""

    def __init__(self, output: Path, executable: str, port: int, reuse: bool = False):
        self.output = output
        self.executable = executable
        self.port = port
        self.project = output / "project"

        if not reuse:
            self.project.mkdir(parents=True)
            for folder in ("self_balancing_robot", "webots"):
                shutil.copytree(
                    ROOT / folder,
                    self.project / folder,
                    ignore=shutil.ignore_patterns("__pycache__", "*.wbproj", ".*.jpg", ".*.png"),
                )

        self.world = self.project / "webots/worlds/webots_self_balancing_robot.wbt"
        if not self.world.is_file():
            raise FileNotFoundError(f"Active PID world is missing: {self.world}")

        world_text = self.world.read_text(encoding="utf-8")
        self.world_source = re.sub(r"(?m)^\s*randomSeed\s+\d+\s*$", "", world_text)

    def evaluate(
        self,
        params: dict[str, float],
        label: str,
        seeds: tuple[int, ...] | list[int],
        reuse_completed: bool = False,
    ) -> tuple[float, dict]:
        all_metrics: dict[str, dict] = {}
        costs: list[float] = []
        seed_tuple = tuple(seeds)
        if not seed_tuple or not SCENARIOS:
            raise ValueError("At least one seed and scenario are required")

        for seed in seed_tuple:
            seeded_world = self.world_source.replace(
                "WorldInfo {", f"WorldInfo {{\n  randomSeed {seed}"
            )
            self.world.write_text(seeded_world, encoding="utf-8")

            for name, (steps, drive, push) in SCENARIOS.items():
                run = self.output / label / f"seed_{seed}_{name}"
                if reuse_completed and run.exists():
                    saved = json.loads((run / "params.json").read_text(encoding="utf-8"))
                    if saved != params:
                        raise ValueError(f"Resume parameters differ: {run}")
                    try:
                        cost, metrics = score_trace(run / "evaluation.csv", steps)
                    except (ValueError, OSError, IndexError):
                        suffix = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                        run.rename(run.with_name(f"{run.name}_interrupted_{suffix}"))
                    else:
                        costs.append(cost)
                        all_metrics[f"{seed}_{name}"] = metrics
                        continue

                run.mkdir(parents=True)
                (run / "params.json").write_text(json.dumps(params), encoding="utf-8")

                env = os.environ.copy()
                env.update(
                    PYTHONUTF8="1",
                    PID_PARAMS_FILE=str(run / "params.json"),
                    PID_RANDOM_SEED=str(seed),
                    PID_LOG_PATH=str(run / "telemetry.csv"),
                    PID_EVALUATION_PATH=str(run / "evaluation.csv"),
                    WEBOTS_TEST_STEPS=str(steps),
                    TEST_DRIVE_COMMANDS=drive,
                    TEST_PUSH_COMMANDS=push,
                )

                try:
                    result = subprocess.run(
                        [
                            self.executable,
                            f"--port={self.port}",
                            "--minimize",
                            "--mode=fast",
                            "--stdout",
                            "--stderr",
                            str(self.world),
                        ],
                        cwd=self.project,
                        env=env,
                        capture_output=True,
                        text=True,
                        encoding="utf-8",
                        errors="replace",
                        timeout=60,
                    )
                except subprocess.TimeoutExpired as exc:
                    raise RuntimeError(f"Webots timeout: {run}") from exc

                (run / "console.txt").write_text(result.stdout + result.stderr, encoding="utf-8")
                try:
                    if (
                        result.returncode
                        or "Robot devrildi" in result.stdout
                        or "Ilac kutusu hazneden dustu" in result.stdout
                    ):
                        raise ValueError(f"Simulation failed: exit {result.returncode}")
                    cost, metrics = score_trace(run / "evaluation.csv", steps)
                except (ValueError, OSError) as exc:
                    return 1e6, {f"{seed}_{name}": {"failure": str(exc)}}

                costs.append(cost)
                all_metrics[f"{seed}_{name}"] = metrics

        return float(np.mean(costs)), all_metrics


def dump(path: Path, value: object) -> None:
    """Veriyi UTF-8 JSON olarak diske yazar."""
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")


def robust(metrics: dict[str, dict]) -> float:
    """Senaryo maliyetlerinin ortalaması + 0.25 * en kötü senaryo maliyetini döndürür."""
    if not metrics:
        raise ValueError("No simulation metrics")
    values = [m.get("cost", 1e6) for m in metrics.values()]
    return float(np.mean(values) + 0.25 * max(values))


def eligible(metrics: dict[str, dict]) -> bool:
    """Tüm senaryoların hatasız ve düşük tekerlek kaymasıyla tamamlandığını doğrular."""
    return bool(metrics) and all(
        "failure" not in m and m.get("slip_over_005_fraction", 1.0) <= 0.05
        for m in metrics.values()
    )
