"""
Optuna PID arama ve puanlama hattı için yanıltıcı sonuçları engelleyen regresyon testleri.
"""

import csv
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

spec = importlib.util.spec_from_file_location(
    "pid_search", ROOT / "analysis/controllers/pid/simulation_runner.py"
)
search = importlib.util.module_from_spec(spec)
spec.loader.exec_module(search)


class ScoringTests(unittest.TestCase):
    def _write_trace(
        self,
        folder: str | Path,
        steps: int = 100,
        remove: int | None = None,
        velocity: float = 0.0,
        target: float = 0.0,
        drop: int = 0,
    ) -> Path:
        trace_path = Path(folder) / "trace.csv"
        with trace_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "step", "time", "pitch", "velocity", "target",
                "left_volts", "right_volts", "box_dropped",
            ])
            for step in range(2, steps + 1):
                if step != remove:
                    writer.writerow([step, step * 0.01, 0.0, velocity, target, 0.0, 0.0, drop])
        return trace_path

    def test_resume_uses_complete_trace_without_launching_webots(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            run = root / "candidate/seed_42_standing"
            run.mkdir(parents=True)
            self._write_trace(run).rename(run / "evaluation.csv")
            (run / "params.json").write_text(
                json.dumps({k: 1.0 for k in search.KEYS}), encoding="utf-8"
            )

            runner = search.Runner.__new__(search.Runner)
            runner.output = root
            runner.world = root / "world.wbt"
            runner.world_source = "WorldInfo {\n}"

            with (
                patch.object(search, "SCENARIOS", {"standing": (100, "", "")}),
                patch.object(search.subprocess, "run") as launch,
            ):
                cost, metrics = runner.evaluate(
                    {k: 1.0 for k in search.KEYS}, "candidate", [42], reuse_completed=True
                )
                launch.assert_not_called()
                self.assertEqual(cost, 0.0)
                self.assertIn("42_standing", metrics)

    def test_resume_rejects_different_parameters(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)
            run = root / "candidate/seed_42_standing"
            run.mkdir(parents=True)
            (run / "params.json").write_text("{}", encoding="utf-8")

            runner = search.Runner.__new__(search.Runner)
            runner.output = root
            runner.world = root / "world.wbt"
            runner.world_source = "WorldInfo {}"

            with patch.object(search, "SCENARIOS", {"standing": (100, "", "")}):
                with self.assertRaisesRegex(ValueError, "parameters differ"):
                    runner.evaluate(
                        {k: 1.0 for k in search.KEYS}, "candidate", [42], reuse_completed=True
                    )

    def test_early_finish_rejected(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            with self.assertRaises(ValueError):
                search.score_trace(self._write_trace(tmp_dir, steps=90), 100)

    def test_missing_internal_sample_rejected(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            with self.assertRaises(ValueError):
                search.score_trace(self._write_trace(tmp_dir, remove=50), 100)

    def test_staying_upright_but_not_driving_has_cost(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            cost, _ = search.score_trace(self._write_trace(tmp_dir, target=0.2), 100)
            self.assertGreater(cost, 10.0)

    def test_payload_loss_rejected(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            with self.assertRaises(ValueError):
                search.score_trace(self._write_trace(tmp_dir, drop=1), 100)


class DirectTuningTests(unittest.TestCase):
    def test_final_regression_rejects_search_winner_and_preserves_gains(self):
        controller_dir = str(ROOT / "analysis/controllers/pid")
        if controller_dir not in sys.path:
            sys.path.insert(0, controller_dir)

        import optimize_pid_controller as tuning
        from self_balancing_robot import PIDController

        with patch.dict(os.environ, {}, clear=True):
            baseline = {k: getattr(PIDController(), k) for k in tuning.KEYS}
            self.assertEqual(
                list(baseline.values()),
                [60.7944, 1.2985, 0.2952, 0.1000, 0.0104, 0.2486],
            )
            calls = []

            def measured(runner, params, label, seeds):
                calls.append((label, tuple(seeds)))
                is_current = params == baseline
                cost = 2.0 if is_current else (3.0 if label.startswith("final_") else 1.0)
                return {"params": params, "metrics": {}, "cost": cost, "eligible": True}

            with tempfile.TemporaryDirectory() as folder:
                out = Path(folder)
                with patch.object(tuning, "measure", side_effect=measured):
                    tuning.tune(None, baseline, 2, out)

                report = json.loads((out / "report.json").read_text(encoding="utf-8"))
                self.assertNotEqual(report["winner"], "current")
                self.assertFalse(report["accepted"])
                self.assertFalse((out / "candidate_params.json").exists())
                self.assertTrue(any(s == tuning.FINAL_SEEDS for _, s in calls))
                self.assertTrue(set(tuning.FINAL_SEEDS).isdisjoint(tuning.SEARCH_SEEDS))
                self.assertTrue(set(tuning.FINAL_SEEDS).isdisjoint(tuning.SELECTION_SEEDS))
                self.assertEqual(baseline, {k: getattr(PIDController(), k) for k in tuning.KEYS})

    def test_missing_active_world_does_not_reuse_deleted_world(self):
        with tempfile.TemporaryDirectory() as folder:
            worlds = Path(folder) / "project/webots/worlds"
            worlds.mkdir(parents=True)
            (worlds / "cdtp_voltage_world.wbt").write_text("WorldInfo {}", encoding="utf-8")
            with self.assertRaisesRegex(FileNotFoundError, "Active PID world"):
                search.Runner(Path(folder), "unused", 1291, reuse=True)

    def test_empty_telemetry_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "empty.csv"
            path.write_text("step,time\n", encoding="utf-8")
            with self.assertRaises(ValueError):
                search.score_trace(path, 100)

    def test_slip_and_failed_runs_are_ineligible(self):
        self.assertFalse(search.eligible({"run": {"failure": "payload"}}))
        self.assertFalse(search.eligible({"run": {"slip_over_005_fraction": 0.06}}))
        self.assertTrue(search.eligible({"run": {"slip_over_005_fraction": 0.01}}))


if __name__ == "__main__":
    unittest.main()
