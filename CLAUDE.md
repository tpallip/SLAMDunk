# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

Teaching platform for Robotics Science and Systems (UoB coursework, team "SLAMdunk"). A Pololu 3Pi+ 32U4 robot is driven by an M5Stack StampC3 (ESP32-C3) that talks to closed robot middleware over I2C. There are three parts:

- `StampC3_Template/`: Arduino firmware for the StampC3 (C++).
- `DigitalTwin/`: Processing (Java) simulator of the same robot and controller, with a live-comparison mode against the real robot over TCP.
- `tests/`: Python TCP logger for collecting sensor trials from the robot, plus its unit tests.

## Commands

Python logger (connect the PC to the robot's Wi-Fi AP first; defaults to `192.168.4.1:80`):

```powershell
python tests\tcp_logger.py [--config config\robot_params.json] [--watch] [--log-dir logs] [--host H] [--port P] [--telemetry-csv path]
```

The full command list and wire protocol are in `tests/README.md`.

Unit tests. They import `tcp_logger` as a top-level module, so run them from inside `tests/`:

```powershell
cd tests; python -m unittest                  # all tests (test_tcp_logger, test_calibrate_sensors)
cd tests; python -m unittest test_tcp_logger.TcpLoggerTests.test_builds_trial_telemetry_and_control_commands
```

Firmware: open `StampC3_Template/StampC3_Template.ino` in the Arduino IDE with the M5Stack ESP32 board package (board `M5StampC3`) and the Pololu `LSM6` and `LIS3MDL` libraries installed. Serial runs at 115200 baud. The WS2812 LED driver is vendored into `Robot.cpp`, so it needs no separate library. To check that it compiles without the IDE, use the IDE's bundled CLI:

```powershell
& "$env:LOCALAPPDATA\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn m5stack:esp32:m5stack_stamp_c3 StampC3_Template
```

Digital Twin: open `DigitalTwin/DigitalTwin.pde` in Processing; it needs the bundled `processing.net` library. Keys: `g` graphs, `t` track robot, `m` toggles sandbox/live-comparison, `+`/`-` zoom, arrow keys pan.

## Architecture

**Firmware control flow.** `loop()` calls `server.update()` (WiFi AP/TCP connection management) and then `controller.update(robot, server)`. `Controller_c::update` is gated by a `TaskTimer_c` (`update_interval_ms`, 10 ms by default). Each cycle it does the following:
1. refreshes surface sensors, encoders and pose over I2C, and records which reads succeeded in `read_ok_mask`;
2. updates local odometry;
3. handles Serial and TCP commands;
4. polls helper-motion and self-test completion;
5. sends `DATA` telemetry and `LOG` records when those streams are on.

`runLineFollower()` (WAITING → FOLLOWING_LINE ↔ SEARCHING_FOR_LINE → STOPPED) exists but is **not currently called** from `update()`.

**Runtime parameters.** Every tunable value is a field of `RobotParams_t` in `RobotParams.h`. That covers timing intervals, trial sample count, line-follower gains and thresholds, the motor PWM limit, and wheel geometry. A descriptor table in `RobotParams.cpp` drives set, get, listing, range checks and NVS save/load. NVS stores a blob plus a layout hash, so a stale blob is ignored after the struct changes.

`Controller_c::begin()` loads the stored values at boot. `applyParams()` pushes them into the timers, into `odometry` (re-baselining only when the geometry changed), and into the middleware motion-helper geometry.

**Sensor normalisation.** `Controller_c::normaliseSurface()` runs after each successful surface read and fills `surface_norm[5]` with `clamp((raw - white_dnN) / (black_dnN - white_dnN), 0, 1)`. A sensor whose black limit is not above its white limit reads 0.
- `lineDetected()` and the steering error in `runLineFollower()` both use `surface_norm`. That makes `line_threshold` a 0–1 value and `line_follow_gain` a PWM per unit of normalised error.
- `normalise_readings()` in `tcp_logger.py` mirrors the firmware formula for the `watch` view.
- The default limits are per-sensor means at 0% and 100% blackness. `tests/calibrate_sensors.py` regenerates them from trial CSVs, and a test checks that `config/robot_params.json` still matches `telemetry.csv`.

To add a parameter, update all of these together:
- the struct field and its table row in the firmware;
- `KNOWN_PARAMS` in `tests/tcp_logger.py`;
- `config/robot_params.json`. A unit test checks that this file and `KNOWN_PARAMS` match.

**Hardware layering.**
- `Robot_c` is the only I2C client of the 32U4 middleware (address `0x08`). Its public members are *caches*: a failed transaction leaves them unchanged, and cached PWM is the last value requested, not a measurement. It owns `Wire.begin()`.
- `RobotIMU_c` reads the LSM6DS33 and LIS3MDL directly on the same bus and must be initialised *after* `Robot_c`. It deliberately returns raw counts with no filtering or bias removal.
- `i2c_datatypes.h` holds the packed binary protocol shared with the middleware firmware, which is not in this repo. Do not reorder fields or change their widths.
- `OdometryModel_c` is a student-editable Euler odometry model, separate from the middleware's `getPose()`. Telemetry x/y/theta come from this local model, not from `robot.pose`.

**TCP protocol (firmware ↔ `tcp_logger.py`).** Commands and replies are newline-terminated text; the full spec is in `tests/README.md`. The commands fall into four groups:
- telemetry and trials;
- parameters (`SET`, `GET`, `PARAMS`, `SAVE`, `DEFAULTS`);
- the full-state `LOG` stream, with 26 fields (raw DN1..DN5 plus normalised `dn*_norm`);
- motion (`MOVE`, `ROTATE`, `SELFTEST`, `ODOM_RESET`).

Firmware implementation details:
- `handleTcpCommand` first tries `handleParamCommand` and `handleMotionCommand`.
- New code sends replies with `Controller_c::sendFormatted`, which refuses to send anything over 255 bytes. Avoid `RobotWifiAP_c::printlnf`, which misbehaves on payloads over about 253 bytes.
- `tcp_control_active` means "stop the motors if the client disconnects". MOTOR, helper motions and the self-test set it.

Logger structure:
- `RobotSession` in `tcp_logger.py` owns the socket, the receiver thread and all output files: the trial CSV, the LOG CSV and an always-on session transcript in `logs/`.
- Request/reply commands (`load`, `save-config`) route replies through a queue while `collecting` is set.
- `watch` redraws `format_watch_table()` in place with ANSI codes, using only the standard library.

A separate Serial path accepts `trial_no,percentage` lines and writes one CSV record per line.

When you change the protocol, update these together:
- `Controller.cpp`;
- `tests/tcp_logger.py` (`build_command`, `parse_response`, the `*_COLUMNS` lists);
- `tests/README.md`.

**Digital Twin mirroring.** The `.pde` files reuse the firmware's class names (`Robot_c`, `Controller_c`, `TaskTimer_c`, `OdometryModel_c`) so controller code can move between the two. `Controller.pde` constants must stay aligned with the defaults in `StampC3_Template/RobotParams.h`, except the sensor threshold. The Digital Twin does not read the JSON config, and it does not normalise: it still uses a raw `LINE_THRESHOLD` of 1400 on simulated sensors, whereas the firmware uses a normalised 0.5. `Simulator` steps the model at a fixed 10 ms with catch-up, renders `surface.png` at 1 px/mm, and in live mode connects to `192.168.4.1:80` and parses telemetry in `TCPTelemetry_c`. On connect it sends `START TELEMETRY`, and `TCPTelemetry_c.parse` accepts `DATA,`-prefixed or bare 14-field records. If the `DATA` record layout changes, update this parser too.

**Sensor experiments (course exercises 4–7).**
- `RobotSession.collect_batch(n, pct)` in `tcp_logger.py` runs a firmware `TRIAL` and returns the samples in memory. While a batch is active, samples go to `batch_rows` instead of the trial CSV. It needs no firmware changes.
- `tests/experiments.py` builds a list of `Step`s per protocol (`sweep`, `noise`, `footprint`) and prompts for each one. It keeps the most recent batch in memory so that `r` (redo) never leaves bad rows in the file. It writes long-format CSVs and metadata JSON to `data/experiments/`.
- `tests/reflectance_analysis.py` holds the pure maths functions (fits, golden-section γ, noise statistics, FWHM) and per-exercise report writers. They output `summary.md` and PNG plots to `reports/`, which is gitignored.
- Only the analysis needs numpy and matplotlib (`requirements.txt`); the logger and the experiment runner use only the standard library.

## Data files

`telemetry.csv` (trial output) is tracked in git and is append-only by design. `sensor_test.csv` and `logs/` (session transcripts and LOG CSVs) are gitignored. `config/robot_params.json` is the parameter set to load. Its geometry comes from `3pi-plus-32u4-oled-robot-dimensions.pdf`: wheel radius 16.0 mm and wheel separation 89.2 mm, against a compiled default of 90. Don't rewrite existing rows; earlier trials may use an older schema.
