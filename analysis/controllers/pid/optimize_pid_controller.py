#!/usr/bin/env python3
"""
================================================================================
MODUL: KASKAD PID KONTROLCU COK DEGISKENLI BAYESYEN TPE OPTIMIZASYONU
================================================================================
Bu betik; alti kaskad PID kazancini (KP_VOLT, KD_VOLT, KV_VEL, KP_POS, KI_POS,
KP_VEL_DRIVE) yalnizca dogrudan Webots fizik olcumleriyle optimize eder.
Arama, aday secimi ve nihai dogrulama birbirinden ayrik rastgele tohumlar kullanir.
================================================================================
"""

import argparse
from datetime import datetime
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
for path_entry in (str(ROOT), str(Path(__file__).resolve().parent)):
    if path_entry not in sys.path:
        sys.path.insert(0, path_entry)

from self_balancing_robot import PIDController
from simulation_runner import (
    KEYS,
    SPACE,
    Runner,
    dump,
    eligible,
    get_webots_executable,
    robust,
)

SEARCH_SEEDS = (42, 43)
SELECTION_SEEDS = (501, 502, 503, 504, 505)
FINAL_SEEDS = (901, 902, 903, 904, 905)


def measure(runner: Runner, params: dict[str, float], label: str, seeds: tuple[int, ...]) -> dict:
    """Verilen parametre setini belirtilen tohumlarda çalıştırıp özet sonucu döndürür."""
    _, metrics = runner.evaluate(params, label, seeds, reuse_completed=True)
    return {
        "params": params,
        "metrics": metrics,
        "cost": robust(metrics),
        "eligible": eligible(metrics),
    }


def tune(runner: Runner, baseline: dict[str, float], trials: int, out: Path) -> None:
    """Optuna TPE araması, en iyi 3 aday seçimi ve bağımsız tohum doğrulamasını yürütür."""
    import optuna

    study = optuna.create_study(
        direction="minimize",
        sampler=optuna.samplers.TPESampler(seed=731),
    )
    study.enqueue_trial(baseline)
    candidates: list[dict] = []

    def objective(trial: optuna.Trial) -> float:
        if (out / "STOP").exists():
            study.stop()
            raise optuna.TrialPruned()
        params = {key: trial.suggest_float(key, *SPACE[key]) for key in KEYS}
        result = measure(runner, params, f"trial_{trial.number:04d}", SEARCH_SEEDS)
        candidates.append(result)
        dump(out / "trials.json", candidates)
        print(f"Trial {trial.number + 1}/{trials}: {result['cost']:.6g}", flush=True)
        return result["cost"] if result["eligible"] else 1e6

    study.optimize(objective, n_trials=trials)
    if (out / "STOP").exists():
        dump(out / "report.json", {"accepted": False, "reason": "Stopped before selection"})
        return

    # En iyi 3 farklı geçerli adayı ve mevcut referans kazançları seçim tohumlarında yeniden değerlendir
    shortlist = {"current": baseline}
    for result in sorted(candidates, key=lambda item: item["cost"]):
        if result["eligible"] and result["params"] not in shortlist.values():
            shortlist[f"candidate_{len(shortlist)}"] = result["params"]
        if len(shortlist) == 4:
            break

    selection = {}
    for name, params in shortlist.items():
        selection[name] = measure(runner, params, f"selection_{name}", SELECTION_SEEDS)
        dump(out / "selection.json", selection)

    valid = [name for name, result in selection.items() if result["eligible"]]
    winner = min(valid, key=lambda name: selection[name]["cost"]) if valid else "current"

    # Seçimi dondurduktan sonra görülmemiş nihai tohumlarda referansla karşılaştır
    final = {
        name: measure(runner, shortlist[name], f"final_{name}", FINAL_SEEDS)
        for name in dict.fromkeys(["current", winner])
    }
    accepted = (
        winner != "current"
        and final["current"]["eligible"]
        and final[winner]["eligible"]
        and final[winner]["cost"] < final["current"]["cost"]
    )

    report = {
        "method": "Direct Webots Optuna search",
        "trials": len(candidates),
        "baseline_params": baseline,
        "winner": winner,
        "accepted": accepted,
        "params": shortlist[winner],
        "selection": selection,
        "final": final,
        "search_seeds": SEARCH_SEEDS,
        "selection_seeds": SELECTION_SEEDS,
        "final_seeds": FINAL_SEEDS,
        "defaults_modified": False,
        "scope": "Standing, pushes, straight driving and braking; turning and hardware require separate tests",
    }
    dump(out / "report.json", report)
    if accepted:
        dump(out / "candidate_params.json", shortlist[winner])
    print(f"COMPLETE: {winner}, accepted={accepted}; active gains unchanged", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trials",
        type=int,
        default=100,
        help="Total candidates, including the current gains (default: 100)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="New experiment directory; defaults to logs/pid_tuning/<timestamp>",
    )
    parser.add_argument("--port", type=int, default=1291)
    args = parser.parse_args()

    if args.trials < 1:
        parser.error("--trials must be positive")
    if os.environ.get("PID_PARAMS_FILE"):
        parser.error("Unset PID_PARAMS_FILE so the baseline is the active source gains")

    executable = get_webots_executable()
    controller = PIDController()
    baseline = {key: getattr(controller, key) for key in KEYS}

    default_out = ROOT / "logs/pid_tuning" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    out = (args.output or default_out).resolve()
    out.mkdir(parents=True, exist_ok=False)

    runner = Runner(out, executable, args.port)
    dump(out / "baseline_params.json", baseline)
    print(f"Output: {out}", flush=True)
    tune(runner, baseline, args.trials, out)


if __name__ == "__main__":
    main()
