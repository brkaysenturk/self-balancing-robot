"""Regression checks for rejecting misleading optimization results."""
import csv
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("voltage_search", Path(__file__).resolve().parents[1] / "analysis/controller/optimize_voltage_controller.py")
search = importlib.util.module_from_spec(spec)
spec.loader.exec_module(search)

class ScoringTests(unittest.TestCase):
    def trace(self, folder, steps=100, remove=None, velocity=0, target=0, drop=0):
        p = Path(folder) / "trace.csv"
        with p.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["step", "time", "pitch", "velocity", "target", "left_volts", "right_volts", "box_dropped"])
            for step in range(2, steps + 1):
                if step != remove:
                    writer.writerow([step, step*.01, 0, velocity, target, 0, 0, drop])
        return p

    def test_early_finish_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(ValueError):
                search.score_trace(self.trace(d, steps=90), 100)

    def test_missing_internal_sample_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(ValueError):
                search.score_trace(self.trace(d, remove=50), 100)

    def test_staying_upright_but_not_driving_has_cost(self):
        with tempfile.TemporaryDirectory() as d:
            cost, _ = search.score_trace(self.trace(d, target=.2), 100)
            self.assertGreater(cost, 10)

    def test_payload_loss_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(ValueError):
                search.score_trace(self.trace(d, drop=1), 100)

if __name__ == "__main__":
    unittest.main()
