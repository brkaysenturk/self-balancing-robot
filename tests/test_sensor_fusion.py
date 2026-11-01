"""
Simülasyon gürültü üretiminden bağımsız, deterministik tamamlayıcı filtre birim testleri.
"""

import math
from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from self_balancing_robot.sensor_fusion import SensorFusion


class SensorFusionTests(unittest.TestCase):
    def test_tilt_and_reset(self):
        fusion = SensorFusion()
        theta = 0.1
        inputs = (0.0, 9.81 * math.sin(theta), 9.81 * math.cos(theta), 0.01)
        for _ in range(100):
            pitch, rate = fusion.update(*inputs)
            self.assertAlmostEqual(pitch, theta)
            self.assertEqual(rate, 0.0)
        fusion.reset()
        self.assertEqual(fusion.update(0.0, 0.0, 9.81, 0.01), (0.0, 0.0))

    def test_filter_does_not_consume_random_numbers(self):
        fusion = SensorFusion()
        state = random.getstate()
        fusion.update(0.2, 0.1, 9.81, 0.01)
        self.assertEqual(random.getstate(), state)


if __name__ == "__main__":
    unittest.main()
