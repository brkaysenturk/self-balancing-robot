"""
38 kHz IR verici feneri ve 2x TSOP38438 alıcı nirengisiyle istasyona yanaşma birim testleri.
"""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from self_balancing_robot import CONFIG, IRHomingController


class IRHomingTests(unittest.TestCase):
    def test_symmetric_signal_when_aligned_on_centerline(self):
        homing = IRHomingController()
        reading = homing.read_receivers(
            robot_xy=(0.0, 1.00),
            robot_yaw_rad=0.0,
            beacon_xy=(0.0, 1.462),
        )
        self.assertTrue(reading.in_cone)
        self.assertGreater(reading.ir_left, 0.1)
        self.assertAlmostEqual(reading.ir_left, reading.ir_right, places=6)
        self.assertAlmostEqual(reading.ir_diff, 0.0, places=6)
        self.assertAlmostEqual(reading.bearing_rad, 0.0, places=6)

    def test_differential_signal_steers_toward_beacon(self):
        homing = IRHomingController()
        homing.start()

        # Robot X = -0.12 m solda, düz ileri (+Y) bakıyor -> İstasyon robotun sağında (+bearing, ir_right > ir_left)
        reading_left_pos = homing.read_receivers(
            robot_xy=(-0.12, 1.15),
            robot_yaw_rad=0.0,
            beacon_xy=(0.0, 1.462),
        )
        self.assertGreater(reading_left_pos.ir_right, reading_left_pos.ir_left)
        self.assertGreater(reading_left_pos.ir_diff, 0.0)
        self.assertGreater(reading_left_pos.bearing_rad, 0.0)

        v_cmd, turn_cmd, docked = homing.compute_commands(reading_left_pos)
        self.assertFalse(docked)
        self.assertGreater(v_cmd, 0.0)
        self.assertGreater(turn_cmd, 0.0)  # Sağa dönüş (+turn_command)

        # Robot X = +0.12 m sağda -> İstasyon robotun solunda (-bearing, ir_left > ir_right)
        reading_right_pos = homing.read_receivers(
            robot_xy=(0.12, 1.15),
            robot_yaw_rad=0.0,
            beacon_xy=(0.0, 1.462),
        )
        self.assertGreater(reading_right_pos.ir_left, reading_right_pos.ir_right)
        self.assertLess(reading_right_pos.ir_diff, 0.0)
        _, turn_cmd_left, _ = homing.compute_commands(reading_right_pos)
        self.assertLess(turn_cmd_left, 0.0)  # Sola dönüş (-turn_command)

    def test_stops_cleanly_at_dock_distance(self):
        homing = IRHomingController()
        homing.start()
        # Robotun ön kenarı vericiye dock_stop_distance_m (6.5 cm) mesafedeyken
        robot_y = 1.462 - CONFIG.homing.receiver_forward_offset_m - (CONFIG.homing.dock_stop_distance_m - 0.01)
        reading = homing.read_receivers(
            robot_xy=(0.0, robot_y),
            robot_yaw_rad=0.0,
            beacon_xy=(0.0, 1.462),
        )
        v_cmd, turn_cmd, docked = homing.compute_commands(reading)
        self.assertTrue(docked)
        self.assertEqual(v_cmd, 0.0)
        self.assertEqual(turn_cmd, 0.0)

        homing.stop()
        self.assertFalse(homing.active)
        self.assertEqual(homing.compute_commands(reading), (0.0, 0.0, False))

    def test_rotates_in_place_when_heading_error_is_large(self):
        homing = IRHomingController()
        homing.start()
        # Robot 65 derece sola dönükken (yaw = +65 deg ENU'da sola bakış), istasyon sağında kalır
        reading = homing.read_receivers(
            robot_xy=(0.0, 1.15),
            robot_yaw_rad=math.radians(65.0),
            beacon_xy=(0.0, 1.462),
        )
        v_cmd, turn_cmd, docked = homing.compute_commands(reading)
        self.assertFalse(docked)
        self.assertEqual(v_cmd, 0.0)  # Önce yerinde hizalanır
        self.assertGreater(turn_cmd, 0.0)

        # Robot koni içinde ama tam arkası dönükken (yaw = 140 deg, ir_sum == 0) yerinde tarama (scan) yapar
        reading_back = homing.read_receivers(
            robot_xy=(0.0, 1.15),
            robot_yaw_rad=math.radians(140.0),
            beacon_xy=(0.0, 1.462),
        )
        self.assertEqual(reading_back.ir_sum, 0.0)
        v_scan, turn_scan, docked_scan = homing.compute_commands(reading_back)
        self.assertFalse(docked_scan)
        self.assertEqual(v_scan, 0.0)
        self.assertAlmostEqual(abs(turn_scan), CONFIG.homing.search_turn_rate, places=6)


if __name__ == "__main__":
    unittest.main()
