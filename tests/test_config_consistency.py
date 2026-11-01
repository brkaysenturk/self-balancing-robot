"""SSOT (Single Source of Truth) CONFIG ve alt bilesen varsayilanlarinin tutarlilik testleri."""

import math
from pathlib import Path
import re
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from self_balancing_robot import (
    CONFIG,
    ControlConfig,
    DCMotorModel,
    FusionConfig,
    HomingConfig,
    IRHomingController,
    MechanicsConfig,
    MotorConfig,
    PIDController,
    RobotConfig,
    SensorFusion,
)


class ConfigConsistencyTests(unittest.TestCase):
    def test_ssot_config_instance_and_component_defaults(self):
        """Tum alt bilesenlerin (PIDController, DCMotorModel, SensorFusion, IRHomingController) CONFIG SSOT degerlerini dogru okudugunu dogrular."""
        self.assertIsInstance(CONFIG, RobotConfig)
        self.assertIsInstance(CONFIG.mechanics, MechanicsConfig)
        self.assertIsInstance(CONFIG.control, ControlConfig)
        self.assertIsInstance(CONFIG.motor, MotorConfig)
        self.assertIsInstance(CONFIG.fusion, FusionConfig)
        self.assertIsInstance(CONFIG.homing, HomingConfig)

        controller = PIDController()
        motor = DCMotorModel()
        fusion = SensorFusion()
        homing = IRHomingController()

        self.assertIs(controller.cfg, CONFIG)
        self.assertIs(homing.cfg, CONFIG)
        self.assertIsInstance(controller.motor, DCMotorModel)

        # 1. Mekanik ve Geriye Donuk Kisayollar
        self.assertTrue(math.isclose(CONFIG.WHEEL_RADIUS, CONFIG.mechanics.wheel_radius, abs_tol=1e-9))
        self.assertTrue(math.isclose(CONFIG.TRACK_WIDTH, CONFIG.mechanics.track_width, abs_tol=1e-9))
        self.assertTrue(math.isclose(CONFIG.FALL_LIMIT_DEG, CONFIG.mechanics.fall_limit_deg, abs_tol=1e-9))
        self.assertTrue(math.isclose(CONFIG.DRIVE_SPEED, CONFIG.mechanics.drive_speed, abs_tol=1e-9))
        self.assertTrue(math.isclose(CONFIG.TURN_SPEED, CONFIG.mechanics.turn_speed, abs_tol=1e-9))
        self.assertTrue(math.isclose(CONFIG.GYRO_NOISE_STD, CONFIG.fusion.gyro_noise_std, abs_tol=1e-9))
        self.assertTrue(math.isclose(CONFIG.ACCEL_NOISE_STD, CONFIG.fusion.accel_noise_std, abs_tol=1e-9))

        # 2. Kaskad PID & PI Donus Kontrolcu SSOT
        self.assertTrue(math.isclose(controller.KP_VOLT, CONFIG.control.kp_volt, abs_tol=1e-6))
        self.assertTrue(math.isclose(controller.KD_VOLT, CONFIG.control.kd_volt, abs_tol=1e-6))
        self.assertTrue(math.isclose(controller.KV_VEL, CONFIG.control.kv_vel, abs_tol=1e-6))
        self.assertTrue(math.isclose(controller.KP_POS, CONFIG.control.kp_pos, abs_tol=1e-6))
        self.assertTrue(math.isclose(controller.KI_POS, CONFIG.control.ki_pos, abs_tol=1e-6))
        self.assertTrue(math.isclose(controller.KP_VEL_DRIVE, CONFIG.control.kp_vel_drive, abs_tol=1e-6))
        self.assertTrue(math.isclose(controller.MAX_PITCH_TARGET, CONFIG.control.max_pitch_target, abs_tol=1e-6))
        self.assertTrue(math.isclose(controller.MAX_POS_INT, CONFIG.control.max_pos_int, abs_tol=1e-6))
        self.assertTrue(math.isclose(controller.KP_TURN, CONFIG.control.kp_turn, abs_tol=1e-6))
        self.assertTrue(math.isclose(controller.KI_TURN, CONFIG.control.ki_turn, abs_tol=1e-6))
        self.assertTrue(math.isclose(controller.MAX_TURN_VOLTS, CONFIG.control.max_turn_volts, abs_tol=1e-6))
        self.assertTrue(math.isclose(controller.back_emf_ff_ratio, CONFIG.control.back_emf_ff_ratio, abs_tol=1e-6))

        # 3. DC Motor & TB6612 Surucu SSOT
        self.assertTrue(math.isclose(motor.v_battery, CONFIG.motor.v_battery, abs_tol=1e-6))
        self.assertTrue(math.isclose(motor.r_armature, CONFIG.motor.r_armature, abs_tol=1e-6))
        self.assertTrue(math.isclose(motor.driver_resistance, CONFIG.motor.driver_resistance, abs_tol=1e-6))
        self.assertTrue(math.isclose(motor.gear_ratio, CONFIG.motor.gear_ratio, abs_tol=1e-6))
        self.assertTrue(math.isclose(motor.kt_motor, CONFIG.motor.kt_motor, abs_tol=1e-6))
        self.assertTrue(math.isclose(motor.ke_motor, CONFIG.motor.ke_motor, abs_tol=1e-6))
        self.assertTrue(math.isclose(motor.efficiency, CONFIG.motor.efficiency, abs_tol=1e-6))
        self.assertTrue(math.isclose(motor.coulomb_friction, CONFIG.motor.coulomb_friction, abs_tol=1e-6))
        self.assertTrue(math.isclose(motor.viscous_friction, CONFIG.motor.viscous_friction, abs_tol=1e-6))
        self.assertTrue(math.isclose(motor.deadband_volts, CONFIG.motor.deadband_volts, abs_tol=1e-6))
        self.assertTrue(math.isclose(motor.max_torque, CONFIG.motor.max_torque, abs_tol=1e-6))

        # 4. Tamamlayici Filtre (SensorFusion) SSOT
        self.assertTrue(math.isclose(fusion.alpha, CONFIG.fusion.alpha, abs_tol=1e-6))
        self.assertTrue(math.isclose(fusion.rate_ema_alpha, CONFIG.fusion.rate_ema_alpha, abs_tol=1e-6))
        self.assertTrue(math.isclose(fusion.min_accel_z_g, CONFIG.fusion.min_accel_z_g, abs_tol=1e-6))

    def test_every_config_field_is_read(self):
        """SSOT: her CONFIG alt veri sinifi alani uretim kodunda en az bir kez okunmalidir."""
        root = Path(__file__).resolve().parents[1]
        sources = []
        for pattern in ("self_balancing_robot/**/*.py", "webots/controllers/**/*.py", "analysis/**/*.py"):
            for path in root.glob(pattern):
                if path.name == "config.py":
                    continue
                sources.append(path.read_text(encoding="utf-8", errors="ignore"))
        blob = "\n".join(sources)

        cfg_text = (root / "self_balancing_robot" / "config.py").read_text(encoding="utf-8")
        unused = []
        section = None
        for line in cfg_text.splitlines():
            match = re.match(r"class (\w+)Config", line)
            if match:
                section_name = match.group(1)
                section = None if section_name == "Robot" else section_name.lower()
                continue
            field = re.match(r"\s+(\w+):\s", line)
            if field and section:
                name = field.group(1)
                if not re.search(r"\." + re.escape(name) + r"\b", blob):
                    unused.append(f"{section}.{name}")
        self.assertEqual(unused, [], f"Unused CONFIG fields: {unused}")


if __name__ == "__main__":
    unittest.main()
