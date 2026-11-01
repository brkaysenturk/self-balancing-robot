"""
Webots'tan bağımsız DC motor ve TB6612 sürücü elektriksel model birim testleri.
"""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from self_balancing_robot.dc_motor_model import DCMotorModel


class MotorModelTests(unittest.TestCase):
    def test_voltage_increases_stall_torque(self):
        motor = DCMotorModel()
        low, _ = motor.compute_torque(6.0, 0.0)
        high, _ = motor.compute_torque(12.0, 0.0)
        self.assertGreater(high, low)
        self.assertGreater(high, 0.32)

    def test_back_emf_and_short_brake(self):
        motor = DCMotorModel()
        _, stationary = motor.compute_torque(6.0, 0.0)
        _, moving = motor.compute_torque(6.0, 10.0)
        torque, current = motor.compute_torque(0.0, 10.0)
        self.assertLess(moving, stationary)
        self.assertLess(torque, 0.0)
        self.assertLess(current, 0.0)

    def test_direction_symmetry(self):
        motor = DCMotorModel()
        positive = motor.compute_torque(6.0, 10.0)
        negative = motor.compute_torque(-6.0, -10.0)
        for a, b in zip(positive, negative):
            self.assertAlmostEqual(a, -b)

    def test_pwm_resolution(self):
        motor = DCMotorModel()
        for requested in (-12.0, -1.23, 0.0, 3.14, 12.0):
            actual = motor.pwm_to_voltage(motor.voltage_to_pwm(requested))
            self.assertLessEqual(abs(actual - requested), 12.0 / 510.0 + 1e-12)

    def test_current_regulation_is_opt_in(self):
        free = DCMotorModel()
        limited = DCMotorModel(max_current=1.0)
        self.assertGreater(free.compute_torque(12.0, 0.0)[1], 1.0)
        self.assertEqual(limited.compute_torque(12.0, 0.0)[1], 1.0)


if __name__ == "__main__":
    unittest.main()
