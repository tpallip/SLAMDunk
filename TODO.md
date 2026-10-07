# Benchmarking TODO

Measure what the platform can actually do before relying on it in the final project.
Each item should end with a number (with spread), the method used, and where the raw data lives.

**Conventions**
- Raw data and scripts go in `bench/<item>/` (CSV + the script/sketch that produced it).
- Report distributions, not single values: median, p95, p99, max, and sample count.
- Record conditions: robot ID, battery state, WiFi distance/obstacles, firmware commit.
- Tick an item only when its result is written in the **Results** column.

---

## 1. Communication

| # | Item | Method | Results |
|---|------|--------|---------|
| [ ] 1.1 | **TCP round-trip latency** (laptop ↔ StampC3) | Laptop sends timestamped ping over TCP; StampC3 echoes immediately from `loop()`. Measure RTT distribution idle, and with the controller + telemetry running. One-way ≈ RTT/2. | |
| [ ] 1.2 | **TCP throughput, robot → laptop** | Stream fixed-size records as fast as possible; vary record size (CSV line vs packed binary via `sendBuffer`). Measure sustained bytes/s and records/s. | |
| [ ] 1.3 | **TCP throughput, laptop → robot** | Same, reversed; check the StampC3 keeps up without stalling `loop()`. | |
| [ ] 1.4 | **Cost of sending on the robot** | Time `server.printf` / `sendBuffer` calls with `micros()`. Does `write()` block when the laptop is slow or out of range? | |
| [ ] 1.5 | **WiFi range and dropouts** | Latency/throughput vs distance and with other robots' APs nearby; time to detect disconnect and reconnect. | |
| [ ] 1.6 | **USB Serial baseline** | Repeat 1.1–1.2 over Serial at 115200 (and higher baud) for comparison. | |
| [ ] 1.7 | **I2C transaction time** (StampC3 ↔ 3Pi+) | Time each `Robot_c` call (`getSurfaceSensors`, `getEncoders`, `getPose`, `setMotorPWM`) with `micros()`. Record failure rate at 400 kHz. | |
| [ ] 1.8 | **Maximum control-loop rate** | Remove the 10 ms `TaskTimer_c` gate; measure achievable cycle rate with a full sense → act cycle. | |

## 2. End-to-end latency and timing

| # | Item | Method | Results |
|---|------|--------|---------|
| [ ] 2.1 | **Remote command → wheel motion** | Laptop sends a motor command over TCP; robot timestamps receipt, I2C send, and first encoder change. Split into: network, StampC3 processing, I2C, motor/encoder response. | |
| [ ] 2.2 | **Sensor event → motor command** (on-board) | Move a black/white edge under a surface sensor; timestamp first reading change and resulting `setMotorPWM`. | |
| [ ] 2.3 | **Control-loop period and jitter** | Log `micros()` at each accepted `Controller_c::update`; histogram of actual period vs nominal 10 ms, with and without WiFi client. | |
| [ ] 2.4 | **Data freshness from the middleware** | Compare `surface.timestamp_us` across consecutive reads: how often does the 3Pi+ publish new data vs return a repeated snapshot? | |
| [ ] 2.5 | **Motor step response** | Step PWM 0 → N; log encoder counts at max rate. Extract dead time and time constant per wheel. | |
| [ ] 2.6 | **PWM → wheel speed curve** | Sweep PWM; steady-state speed per wheel. Find deadband (minimum PWM that moves), linearity, left/right asymmetry. | |
| [ ] 2.7 | **Motion-helper accuracy** | `startMoveDistance` / `startRotateAngle` for a range of targets; measure physically. Also time until `MOTION_STATUS_COMPLETE`. | |
| [ ] 2.8 | **Motor PWM deadband** | Ramp PWM up slowly from 0 per wheel, forward and reverse; record the PWM at first encoder change (breakaway) and, ramping down, where motion stops. Repeat from rest and on different surfaces. | |
| [ ] 2.9 | **Maximum line-following speed** | Run the line follower at increasing base speeds on a fixed course; log robot odometry velocity (from encoder deltas). Report the highest sustained velocity that completes the course without losing the line, plus velocity on straights vs curves. | |
| [ ] 2.10 | **Latency added by the logger (robot → logger → DigitalTwin)** | The robot accepts one TCP client, so the twin must get data through the logger when both run. Timestamp each record on arrival at the logger and again at the twin (same laptop clock); compare against the twin connected directly to the robot. Split into logger parsing, Rerun/dashboard logging, and forwarding; repeat with `USE_RERUN` on and off. | |

## 3. Sensors

### Surface (line) sensors DN1–DN5
| # | Item | Method | Results |
|---|------|--------|---------|
| [ ] 3.1 | **Static noise** | Robot stationary on white, then on black: mean and std dev per sensor over ≥ 1000 samples. | |
| [ ] 3.2 | **Range and per-sensor offsets** | Min/max per sensor on the course surface; derive per-sensor normalisation. | |
| [ ] 3.3 | **Line threshold validity** | Is `LINE_THRESHOLD = 1400` separable for every sensor, under different room lighting? | |
| [ ] 3.4 | **Spatial response** | Slide an edge across each sensor in known mm steps; sensor footprint and effective lateral resolution. | |
| [ ] 3.5 | **Sim vs real** | Compare real readings with `SurfaceSensorModel` in DigitalTwin for the same positions. | |

### Encoders and odometry
| # | Item | Method | Results |
|---|------|--------|---------|
| [ ] 3.6 | **Counts per revolution** | Rotate a wheel exactly N turns by hand; verify 358.3 counts/rev. | |
| [ ] 3.7 | **Encoder counts → distance (mm/count)** | Drive straight over a measured distance (e.g. 1 m, tape measure) at several speeds, forward and reverse; mm/count = distance / counts, per wheel. Compare with nominal 2π·16 mm / 358.3 ≈ 0.281 mm/count (`OdometryModel_c` defaults) and check for wheel slip at higher PWM. | |
| [ ] 3.8 | **Wheel radius and separation calibration** | Straight-line runs (radius) and in-place rotations (separation) against physical measurement. | |
| [ ] 3.9 | **Odometry drift** | Repeated square paths (UMBmark-style), clockwise and anticlockwise; end-point error distribution. | |
| [ ] 3.10 | **Local vs middleware pose** | Log `controller.odometry.pose` and `robot.pose` side by side; quantify divergence over a run. | |
| [ ] 3.11 | **Integration interval effect** | Odometry error vs update interval (10, 20, 50 ms) on curved paths. | |
| [ ] 3.12 | **Update frequency effect on encoder change** | At fixed PWMs (slow to fast), log encoder deltas per update at 100, 50, 20, 10 Hz. Report counts per update, how often the delta is 0 or repeats (stale middleware data, see 2.4), and the noise of the velocity estimate (counts/s) at each rate. Find the rate above which 1-count quantisation dominates (fewer counts per shorter interval). | |

### IMU (LSM6DS33 accel/gyro, LIS3MDL magnetometer)
| # | Item | Method | Results |
|---|------|--------|---------|
| [ ] 3.13 | **Achievable sample rate** | Time `imu.read()`; how often `isSampleReady()` is true; effect on the control loop. | |
| [ ] 3.14 | **Gyro bias and noise** | Stationary for several minutes: bias, noise density, bias drift (Allan variance if time allows). | |
| [ ] 3.15 | **Gyro scale factor** | Known rotations (e.g. 10 × 360°); integrated gyro heading vs truth. | |
| [ ] 3.16 | **Gyro vs encoder heading** | Heading from gyro vs odometry during turns and line following; which drifts faster? | |
| [ ] 3.17 | **Accelerometer noise and vibration** | Stationary noise; noise while driving at different PWMs. | |
| [ ] 3.18 | **Magnetometer usefulness** | Heading while rotating on the spot, motors off vs on; interference from motors and the surroundings. | |

### Clocks and power
| # | Item | Method | Results |
|---|------|--------|---------|
| [ ] 3.19 | **Clock alignment** | Drift between StampC3 `millis()`, 3Pi+ `timestamp_us`, and the laptop clock over a long run. | |
| [ ] 3.20 | **Battery effect** | Battery voltage isn't exposed over I2C. Track speed at fixed PWM across a session as a proxy (see 2.6). | |

## 4. Software

| # | Item | Method | Results |
|---|------|--------|---------|
| [ ] 4.1 | **Function dependency map** | Document which functions call which in the firmware (`loop()` → `Controller_c::update` → `publishTelemetry`, `Robot_c`, `RobotWifiAP_c`) and in `tools/logger.py`. Update it whenever a change adds, removes or rewires a function. | |
