# Changelog

All notable changes to the YTU ÇDTP Two-Wheeled Self-Balancing Patient Support Robot will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed
- **Simulation Fall Protection Deadlock:** Moved keyboard sampling and reset key ('R') evaluation before the fall threshold check in 'balance_controller.py', allowing physics recovery even when tilted beyond limit.
- **Medicine Box Drop Coordinate Reference:** Converted medicine box tracking from world frame to the robot local body frame via the transpose of the rotation matrix (R^T), eliminating false drop triggers during pitch tilt.
- **Telemetry Ground Truth Pitch Logging:** Separated true ground truth body roll ('pitch_deg') from complementary filter estimation ('pitch_est_deg'), enabling precise sensor lag and dynamic acceleration error analysis.
- **Backward Driving Negative Velocity:** Corrected reverse driving velocity direction and assertion bounds to ensure negative linear speed is strictly verified.
- **Test Suite Early Exit Vulnerability:** Added process return code validation ('res.returncode == 0') and strict time/step completion assertions ('time_s >= ...', 'step >= ...') across all test runners.

### Changed
- **Consolidated Test Architecture:** Merged standalone spec test scripts into two rigorous test suites: 'test_simulation_balance.py' (30s static balance, 4.0 N dual push, and medicine security) and 'test_simulation_driving.py' (0.8 - 1.2 m distance, 3s stop, reverse drive, and medicine security).
- **Floor Environment Simplification:** Removed floor cable conduit from 'cdtp_balance_world.wbt' to focus on standard planar indoor floor dynamics.
- **Controller Speed Configuration:** Parameterized nominal driving speed via 'DRIVE_SPEED' in 'config.py'.

### Added
- **Forward Runaway on Throttle Release (W Key):** Fixed forward pitch-over acceleration caused by lack of rolling speed feedforward when braking. Added base wheel angular velocity feedforward (`base_omega = - v / R`) and deceleration-based target pitch generation.
- **Conditional Integration & Clamping Anti-Windup:** Implemented saturation-aware conditional integration and strict clamping limits (`MAX_POS_INT = 0.040 rad`) on the outer loop position integrator to eliminate cascade windup and prevent runaway backward drift.
- **Discrete 50 Hz Chatter Elimination:** Applied a low-pass filter on the gyro rate (`filtered_rate = 0.70 * filtered_rate + 0.30 * gyro_vals[0]`) to remove discrete Nyquist oscillation at 100 Hz sampling.
- **30-Second Extended Equilibrium:** Verified zero-drift (0.000 m drift, 0.00 deg pitch) standing stability over 30 continuous seconds and fast recovery from a 3.0-degree external disturbance.

### Added
- **Disturbance Rejection & Push Recovery:** Added interactive impulse simulation (`P` key) delivering 4.0 N lateral perturbing forces via Webots Supervisor API (`addForceWithOffset`) and verified recovery within 1.5 seconds ([test_simulation_push.py](file:///c:/YTU/self-balancing-robot/tests/test_simulation_push.py)).
- **Physical Dynamic Medicine Box (Spec Module E):** Converted the static medicine box shape into an independent dynamic `Solid` entity (25 g Aspirin box) seated inside a four-walled cassette holster on Layer 4, with realistic contact friction ([test_simulation_medicine_box.py](file:///c:/YTU/self-balancing-robot/tests/test_simulation_medicine_box.py)).
- **MPU-6050 Sensor Noise & Complementary Filter:** Injected realistic Gaussian sensor noise on gyro (0.002 rad/s) and accelerometer (0.012 m/s²) and implemented the 98% Gyro + 2% Accel complementary filter in `balance_controller.py` mirroring physical ESP32 firmware.
- **JGB37 DC Motor Deadband & Stiction Compensation:** Modeled 0.60 rad/s gear friction deadband and implemented feedforward deadband compensation (`compensate_deadband`, 0.50 rad/s) for immediate motor response.
- **Indoor Floor Cable Conduit / Rug Obstacle:** Added a 4 mm tall, 45 mm wide floor cable runner at Y = 0.85 m in the arena and verified stable obstacle traversal without tipping over ([test_simulation_obstacle.py](file:///c:/YTU/self-balancing-robot/tests/test_simulation_obstacle.py)).
- Webots R2025a simulation world ([cdtp_balance_world.wbt](file:///c:/YTU/self-balancing-robot/webots/worlds/cdtp_balance_world.wbt)) with 4-deck golden ratio chassis (135x95 mm), 4x M3 tij support columns, 85 mm rubber wheels, MPU-6050 IMU, JGB37 motors, 3S battery, and medicine box holster.
- Dual-loop cascade PID controller ([balance_controller.py](file:///c:/YTU/self-balancing-robot/webots/controllers/balance_controller/balance_controller.py)) with 100 Hz inner loop pitch stabilization, outer velocity damping, fall protection cut-off, and real-time telemetry logging.
- Keyboard teleoperation support (W/S/A/D/Space/R) for driving, turning, braking, and physics reset.
- Automated headless simulation test suite ([test_simulation_balance.py](file:///c:/YTU/self-balancing-robot/tests/test_simulation_balance.py) and [test_simulation_driving.py](file:///c:/YTU/self-balancing-robot/tests/test_simulation_driving.py)).
- Standardized project directory structure matching `turret_fcs` specifications.
- Comprehensive project memory and architecture documentation ([AGENTS.md](file:///c:/YTU/self-balancing-robot/AGENTS.md)).
- Golden ratio (135 mm × 95 mm) 4-layer box-truss chassis CAD template ([pleksi_katmanlari.dxf](file:///c:/YTU/self-balancing-robot/docs/cad/pleksi_katmanlari.dxf)) and technical drawing ([pleksi_kesim_sablonu.svg](file:///c:/YTU/self-balancing-robot/docs/cad/pleksi_kesim_sablonu.svg)).
- Dedicated vertical cable riser (conduit shaft) aligned across all decks (32 mm × 12 mm).
- Low-EMI base deck architecture with TB6612FNG motor driver located on Layer 1.
- Complete power budget and battery design analysis for 3S 18650 Li-ion configuration with 10A BMS.
- Architecture flowcharts and system design assets organized under `docs/assets/`.
