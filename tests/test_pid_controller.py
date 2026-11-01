"""Üretim kaskad PID denge, PI dönüş kontrolcüsü ve ters EMK ileri beslemesi birim testleri."""

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from self_balancing_robot.pid_controller import PIDController


class PIDControllerTests(unittest.TestCase):
    def test_balance_saturation_prevents_turn_windup(self):
        controller = PIDController()
        for _ in range(500):
            left, right, _, _ = controller.compute(1.0, 0.0, 0.0, 0.0, 1.2, 0.01)
            self.assertEqual(left, right)
        self.assertEqual(controller.turn_integral, 0.0)

    def test_feedback_reduces_voltage_when_turning_too_fast(self):
        slow = PIDController()
        fast = PIDController()
        slow.update_odometry(0.0, 0.0, 0.01)
        fast.update_odometry(-5.0, 5.0, 0.01)

        l1, r1, _, _ = slow.compute(0.0, 0.0, 0.0, 0.0, 1.2, 0.01)
        l2, r2, _, _ = fast.compute(0.0, 0.0, 0.0, 0.0, 1.2, 0.01)

        # Tekerlek başına ters EMK ileri beslemesi çıkarıldıktan sonra geri besleme düzeltmesini karşılaştır
        correction_fast = (r2 - l2) - 0.85 * fast.motor.ke_wheel * (fast.omega_right - fast.omega_left)
        self.assertLess(correction_fast, r1 - l1)

        fast.reset()
        self.assertEqual(fast.turn_omega, 0.0)
        self.assertEqual(fast.turn_integral, 0.0)

    def test_feedforward_uses_each_wheel_speed_and_limits_final_voltage(self):
        controller = PIDController()
        # Simetrik hızlar doğrusal odometriyi ve denge komutunu sıfırda tutar
        controller.update_odometry(-1.0, 1.0, 0.01)
        left, right, pwm_left, pwm_right = controller.compute(0.0, 0.0, 0.0, 0.0, 0.0, 0.01)

        self.assertAlmostEqual((left + right) / 2.0, 0.0)
        # PI sönümleme sağ tekerleğe -0.2 V verir; ters EMK ileri beslemesi +0.306 V ekler
        self.assertAlmostEqual(right, -0.2 + 0.85 * controller.motor.ke_wheel)
        self.assertEqual(pwm_left, controller.motor.voltage_to_pwm(left))
        self.assertEqual(pwm_right, controller.motor.voltage_to_pwm(right))

        controller.update_odometry(-100.0, 100.0, 0.01)
        left, right, pwm_left, pwm_right = controller.compute(0.0, 0.0, 0.0, 0.0, 0.0, 0.01)
        self.assertEqual((left, right), (-12.0, 12.0))
        self.assertEqual((pwm_left, pwm_right), (-255, 255))

        controller.reset()
        self.assertEqual((controller.omega_left, controller.omega_right), (0.0, 0.0))
        self.assertEqual(controller.compute(0.0, 0.0, 0.0, 0.0, 0.0, 0.01), (0.0, 0.0, 0, 0))


if __name__ == "__main__":
    unittest.main()
