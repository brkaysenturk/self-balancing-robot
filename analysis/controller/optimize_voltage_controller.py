"""Optuna search using isolated Webots voltage-model runs (no surrogate physics)."""
import argparse
import csv
import hashlib
import json
import os
import re
from pathlib import Path
import shutil
import subprocess
import sys
from datetime import datetime

import numpy as np
import optuna

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tests"))
from test_simulation_voltage import get_webots_executable

BASELINE = dict(KP_VOLT=65.0, KD_VOLT=1.6, KV_VEL=0.15,
                KP_POS=0.18, KI_POS=0.015, KP_VEL_DRIVE=0.12)
SPACE = dict(KP_VOLT=(40., 100.), KD_VOLT=(0.8, 3.0), KV_VEL=(0.06, 0.3),
             KP_POS=(0.08, 0.35), KI_POS=(0.005, 0.04), KP_VEL_DRIVE=(0.06, 0.25))
SCENARIOS = {
    "standing": (3000, "", ""),
    "push": (3000, "", "500:1.5,1700:-1.5"),
    "drive": (3500, "200:W,1200:SPACE,1800:S,2800:SPACE", ""),
}


def score_trace(path, steps):
    """Physical observations, complete sampling and independently visible failures."""
    with path.open() as f:
        rows = list(csv.DictReader(f))
    if len(rows) != steps - 1 or int(rows[-1]["step"]) != steps:
        raise ValueError("Incomplete simulation trace")
    a = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    if not all(np.isfinite(v).all() for v in a.values()):
        raise ValueError("Non-finite telemetry")
    if not np.array_equal(a["step"], np.arange(2, steps + 1)) or not np.allclose(np.diff(a["time"]), .01):
        raise ValueError("Missing or disordered samples")
    if a["box_dropped"].any() or np.max(np.abs(a["pitch"])) > 15:
        raise ValueError("Payload loss or excessive tilt")
    dv = np.diff(np.column_stack([a["left_volts"], a["right_volts"]]), axis=0) / .01
    # Each normalized squared term may exceed one. Weights express design priorities.
    terms = dict(angle=float(np.mean((a["pitch"] / 1.0)**2)),
                 speed=float(np.mean(((a["velocity"]-a["target"]) / .05)**2)),
                 effort=float(np.mean((a["left_volts"]**2+a["right_volts"]**2)/(2*12**2))),
                 slew=float(np.mean((dv/100)**2)))
    tail = a["time"] >= (steps*.01 - 3)
    terms["settling"] = float(np.mean((a["velocity"][tail]/.02)**2))
    value = terms["angle"] + terms["speed"] + .1*terms["effort"] + .2*terms["slew"] + 2*terms["settling"]
    metrics = dict(terms=terms, peak_pitch=float(np.max(np.abs(a["pitch"]))),
                   tail_max_speed=float(np.max(np.abs(a["velocity"][tail]))),
                   tail_rms_speed=float(np.sqrt(np.mean(a["velocity"][tail]**2))), cost=value)
    return value, metrics


class Runner:
    def __init__(self, output, executable, port, reuse=False):
        self.output, self.executable, self.port = output, executable, port
        self.project = output / "project"
        if not reuse:
            self.project.mkdir(parents=True)
            for folder in ["self_balancing_robot", "webots"]:
                shutil.copytree(ROOT / folder, self.project / folder,
                                ignore=shutil.ignore_patterns("__pycache__", "*.wbproj"))
        self.world = self.project / "webots/worlds/cdtp_voltage_world.wbt"
        self.world_source = re.sub(r"(?m)^\s*randomSeed\s+\d+\s*$", "", self.world.read_text(encoding="utf-8"))

    def evaluate(self, params, label, seeds):
        all_metrics = {}
        costs = []
        for seed in seeds:
            # Same world and sensor seeds for each candidate; validation uses unseen seeds.
            self.world.write_text(self.world_source.replace("WorldInfo {", f"WorldInfo {{\n  randomSeed {seed}"), encoding="utf-8")
            for name, (steps, drive, push) in SCENARIOS.items():
                run = self.output / label / f"seed_{seed}_{name}"
                run.mkdir(parents=True)
                (run / "params.json").write_text(json.dumps(params), encoding="utf-8")
                env = os.environ.copy()
                env.update(PYTHONUTF8="1", VOLTAGE_PARAMS_FILE=str(run / "params.json"),
                           VOLTAGE_RANDOM_SEED=str(seed), VOLTAGE_LOG_PATH=str(run / "telemetry.csv"),
                           VOLTAGE_EVALUATION_PATH=str(run / "evaluation.csv"),
                           WEBOTS_TEST_STEPS=str(steps), TEST_DRIVE_COMMANDS=drive, TEST_PUSH_COMMANDS=push)
                try:
                    result = subprocess.run([self.executable, f"--port={self.port}", "--minimize", "--mode=fast",
                                             "--stdout", "--stderr", str(self.world)], cwd=self.project,
                                            env=env, capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=60)
                except subprocess.TimeoutExpired as exc:
                    # Abort infrastructure failure rather than rewarding or misranking a trial.
                    raise RuntimeError(f"Webots timeout: {run}") from exc
                (run / "console.txt").write_text(result.stdout + result.stderr, encoding="utf-8")
                try:
                    if result.returncode or "Robot devrildi" in result.stdout or "Ilac kutusu hazneden dustu" in result.stdout:
                        raise ValueError(f"Simulation failed: exit {result.returncode}")
                    cost, metrics = score_trace(run / "evaluation.csv", steps)
                except (ValueError, OSError) as exc:
                    return 1e6, {f"{seed}_{name}": {"failure": str(exc)}}
                costs.append(cost)
                all_metrics[f"{seed}_{name}"] = metrics
        return float(np.mean(costs)), all_metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=30)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--port", type=int, default=1291)
    parser.add_argument("--webots", default=None)
    parser.add_argument("--evaluate-run", type=Path, help="Validate an existing interrupted study without more search trials")
    args = parser.parse_args()
    if args.trials < 1:
        parser.error("--trials must be positive")
    output = args.evaluate_run.resolve() if args.evaluate_run else ROOT / "logs/voltage_optuna" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    if not args.evaluate_run:
        output.mkdir(parents=True)
    runner = Runner(output, args.webots or get_webots_executable(), args.port, reuse=bool(args.evaluate_run))
    manifest = {str(p.relative_to(runner.project)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in runner.project.rglob("*") if p.is_file()}
    if not args.evaluate_run:
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    optuna.logging.set_verbosity(optuna.logging.WARNING)
    study = optuna.create_study(direction="minimize", study_name="voltage_webots",
                               storage="sqlite:///" + (output / "study.sqlite3").as_posix(),
                               load_if_exists=bool(args.evaluate_run),
                               sampler=optuna.samplers.TPESampler(seed=args.seed, n_startup_trials=8))
    if not args.evaluate_run:
        study.enqueue_trial(BASELINE)

    def objective(trial):
        params = {k: trial.suggest_float(k, *bounds) for k, bounds in SPACE.items()}
        value, metrics = runner.evaluate(params, f"trial_{trial.number:04d}", [args.seed])
        trial.set_user_attr("metrics", metrics)
        print(f"Trial {trial.number}: cost={value:.4f}", flush=True)
        return value

    print(f"Output: {output}", flush=True)
    if not args.evaluate_run:
        study.optimize(objective, n_trials=args.trials, n_jobs=1)
    completed = [t for t in study.trials if t.state == optuna.trial.TrialState.COMPLETE][:args.trials]
    if len(completed) != args.trials:
        raise RuntimeError(f"Expected {args.trials} complete trials, found {len(completed)}")
    best = min(completed, key=lambda t: t.value)
    # Holdout results do not feed back into the search.
    seeds = [args.seed + 100, args.seed + 200]
    base_cost, base_metrics = runner.evaluate(BASELINE, "baseline_validation", seeds)
    best_cost, best_metrics = runner.evaluate(best.params, "candidate_validation", seeds)
    settled = best_cost < 1e6 and all(m.get("tail_max_speed", float("inf")) < .04 for m in best_metrics.values())
    accepted = settled and best_cost < base_cost
    report = dict(trials=len(completed), seed=args.seed, search_space=SPACE,
                  baseline_params=BASELINE, candidate_params=best.params,
                  training_baseline_cost=completed[0].value, training_best_cost=best.value,
                  validation_baseline_cost=base_cost, validation_candidate_cost=best_cost,
                  validation_baseline_metrics=base_metrics, validation_candidate_metrics=best_metrics,
                  accepted=accepted, scope="Straight driving, stops, static balance, +/-1.5 N pushes; not turns or hardware validation")
    (output / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    (output / "candidate_params.json").write_text(json.dumps(best.params, indent=2), encoding="utf-8")
    print(json.dumps({"accepted": accepted, "baseline_cost": base_cost, "candidate_cost": best_cost, "output": str(output)}, indent=2), flush=True)


if __name__ == "__main__":
    main()
