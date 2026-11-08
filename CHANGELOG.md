# Changelog

All notable changes to the YTU ÇDTP Two-Wheeled Self-Balancing Patient Support Robot will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed
- **Chassis M4 rod standardization:** Standardized chassis through-cut holes to Ø4.4 mm (radius 2.2 mm) for M4 stainless steel threaded rods (gijon), updated CAD generator (`docs/cad/generate_chassis.py`), regenerated SVG/DXF templates, updated `tests/test_chassis_geometry.py`, and updated Webots cylinder radius to 0.0020 m.
- **E-stop threshold alignment:** Aligned fall protection tilt limit in `self_balancing_robot/config.py` and documentation to 36.0 degrees, matching system architecture and flowchart specifications.
- **Pytest simulation test discovery:** Added `@pytest.mark.simulation` markers, `has_webots()` detection, and explicit `test_simulation_*` test functions so all 34 tests run seamlessly under `pytest -v` or `-m "not simulation"`.
- **Webots wheel joint damping & test early exit:** Added viscous joint damping (`dampingConstant 0.001 N*m*s/rad`) to both wheels in Webots world, and enabled early test termination upon robot fall in `webots_main.py`.
- **Chassis fabrication geometry:** Generated matching millimetre SVG/DXF drawings; corrected pocket, fuse, LED, rod and strap dimensions, separated placement references from through cuts, and widened wheel track to 190 mm for 2.5 mm clearance per side. Matched Webots collision width and 80 x 26 mm inner cassette dimensions.
- **IR receiver hardware consistency:** Standardized CAD, architecture references, simulation descriptions and plot labels on the selected TSOP2238. Documented TSOP38438 pin/AGC/package differences, revision-specific supply compatibility, burst timing, and the gap between digital receiver outputs and the ideal optical/geometric simulation. Marked SVG receiver symbols as placement references and IR mounting geometry as unverified for fabrication; control behavior is unchanged.
- **Conditional Integration & Clamping Anti-Windup:** Implemented saturation-aware conditional integration and strict clamping limits (`MAX_POS_INT = 0.030 rad`) on the outer loop position integrator to eliminate cascade windup and prevent runaway backward drift.
- **Discrete 50 Hz Chatter Elimination:** Applied a low-pass filter on the gyro rate (`filtered_rate = 0.70 * filtered_rate + 0.30 * gyro_vals[0]`) to remove discrete Nyquist oscillation at 100 Hz sampling.
- **Simulation Fall Protection Deadlock:** Moved keyboard sampling and reset key ('R') evaluation before the fall threshold check in the voltage controller adapter, allowing physics recovery even when tilted beyond limit.
- **Medicine Box Drop Coordinate Reference:** Converted medicine box tracking from world frame to the robot local body frame via the transpose of the rotation matrix (R^T), eliminating false drop triggers during pitch tilt.
- **Telemetry Ground Truth Pitch Logging:** Separated true ground truth body roll (`pitch_deg`) from complementary filter estimation (`pitch_est_deg`), enabling precise sensor lag and dynamic acceleration error analysis.
- **Backward Driving Negative Velocity:** Corrected reverse driving velocity direction and assertion bounds to ensure negative linear speed is strictly verified.
- **Test Suite Early Exit Vulnerability:** Added process return code validation (`res.returncode == 0`) and strict time/step completion assertions (`time_s >= ...`, `step >= ...`) across all test runners.
- **Cross-Platform Test Execution:** Replaced hardcoded Windows Webots binary paths in all test runners with dynamic resolution (`WEBOTS_EXECUTABLE` environment variable, `PATH`, and standard platform defaults).

### Changed
- Unified per-wheel back-EMF feedforward in `PIDController` (`self_balancing_robot/pid_controller.py`), encoder-feedback PI differential turning, and direct Webots Optuna multi-seed evaluation in `analysis/controllers/pid/`.
- **Floor Environment Simplification:** Updated the arena to a compact 4x4m dark-gray planar indoor test room for high visual contrast with the 30 cm approach sector.
- **Controller Speed Configuration:** Parameterized nominal driving speed and all subsystem constants via frozen SSOT dataclasses in `config.py`.

### Added
- **Single Source of Truth (SSOT) Configuration (`self_balancing_robot/config.py`):** Structured frozen `MechanicsConfig`, `ControlConfig`, `MotorConfig`, `FusionConfig`, `HomingConfig`, and root `RobotConfig` (`CONFIG`) consumed across all core modules and verified by `tests/test_config_consistency.py`.
- **Autonomous IR Receiver-Transmitter Docking & Funnel Guide (`self_balancing_robot/ir_homing.py`):** Implemented differential IR intensity homing, in-place radar scan (`search_turn_rate`), `TCRT5000` proximity stop detection, dynamic Layer 4 status LED (`STATUS_LED_MAT`), and elevated frictionless funnel guide rails (`ir_dock_station`) in `webots/worlds/webots_self_balancing_robot.wbt`, verified by `tests/test_ir_homing.py` and `tests/test_simulation_ir_homing.py`.
- **300 DPI Academic Validation Figures (`analysis/comparison/plot_tuned_system.py`):** Added automated figure generation script producing `python_fig1` through `python_fig5` in `analysis/figures/python/`.
- **Physical DC Motor & TB6612 Voltage/Torque Model:** Implemented `DCMotorModel` (`self_balancing_robot/dc_motor_model.py`) and `pid_controller.py` modeling armature resistance, gear reduction, Back-EMF feedforward, Coulomb/viscous friction, and traction slip limits for direct ESP32 firmware parity.
- **Voltage Controller World & Test Runner:** Added dedicated Webots world `webots/worlds/webots_self_balancing_robot.wbt`, adapter `webots/controllers/webots_main/webots_main.py`, and test runner `tests/test_simulation_pid.py` verifying 30s standing equilibrium under physical torque control.
- **Disturbance Rejection & Push Recovery:** Added interactive impulse simulation (`P` key) delivering lateral perturbing forces via Webots Supervisor API (`addForceWithOffset`) and verified recovery.
- **Physical Dynamic Medicine Box (Spec Module E):** Converted the static medicine box shape into an independent dynamic `Solid` entity (25 g Aspirin box) seated inside a four-walled cassette holster on Layer 4, with realistic contact friction.
- **MPU-6050 Sensor Noise & Complementary Filter:** Injected realistic Gaussian sensor noise on gyro (0.002 rad/s) and accelerometer (0.012 m/s²) and implemented the 98% Gyro + 2% Accel complementary filter in the voltage controller adapter mirroring physical ESP32 firmware.
- **Webots R2025a Simulation Worlds:** Worlds with 4-deck golden ratio chassis (135x95 mm), 4x M3 tij support columns, 85 mm rubber wheels, MPU-6050 IMU, JGB37 motors, 3S battery, and medicine box holster.
- **Keyboard Teleoperation:** Teleoperation support (W/S/A/D/Space/P/R) for driving, turning, braking, external push perturbation, and physics reset.
- **Comprehensive Project Documentation:** Architecture documentation (`AGENTS.md`), project memory, power budget analysis for 3S 18650 Li-ion configuration with 10A BMS, and laser-cutting CAD templates (`docs/cad/`).
