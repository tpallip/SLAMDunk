# TCP logger and controller

The logger connects to the StampC3's Wi-Fi access point and TCP server. It does
four things:

- collects sensor trials;
- records a full-state log;
- changes the robot's parameters at runtime, so you don't have to reflash;
- shows a live telemetry table.

Connect the computer to the robot's access point first, then run this from the
repository root:

```powershell
python tests\tcp_logger.py --config config\robot_params.json --watch
```

Command-line options:

| Option | Default | Purpose |
|---|---|---|
| `--host` / `--port` | `192.168.4.1` / `80` | Robot address |
| `--config file.json` | none | Load parameters as soon as it connects |
| `--watch` | off | Open the live table as soon as it connects |
| `--telemetry-csv` | `telemetry.csv` | Trial CSV, appended to and never overwritten |
| `--log-dir` | `logs` | Session transcripts and LOG CSVs |

## Commands

Type `help` in the logger to list these.

| Command | What it does |
|---|---|
| `start telemetry` / `stop telemetry` | Periodic `DATA` stream, every `telemetry_interval_ms` |
| `status` | Send one `DATA` record now |
| `trial <n>, <pct>` | Collect `trial_sample_count` fresh readings and add them to the trial CSV |
| `motor <l> <r>` | Set motor PWM, within ±`motor_pwm_limit` |
| `stop` | Stop both motors (also cancels a helper motion) |
| `move <mm>` / `rotate <deg>` | Middleware helper motion; prints `MOTION_DONE` when finished |
| `selftest` | 2 s hardware self-test. **The robot spins on the spot.** Prints `SELFTEST_DONE` |
| `odom-reset [x y theta]` | Reset the local odometry pose (mm, mm, rad) |
| `params` | List every parameter on the robot |
| `get <name>` / `set <name> <value>` | Read or change one parameter (changes take effect at once) |
| `load <file.json>` | Send every key in a JSON file and print a summary of which keys were accepted |
| `save-config <file.json>` | Write the robot's current parameters to JSON |
| `save` | Store the current parameters in flash, so they are reloaded at boot |
| `defaults` | Restore the compiled defaults and erase the parameters stored in flash |
| `log start [file.csv]` / `log stop` | Full-state `LOG` stream to a CSV file |
| `watch` | Live table that updates in place. Press `n` to switch DN1–DN5 between raw counts and normalised 0–1 values. Press Enter or Ctrl+C to return to the prompt |
| `quit` | Stop the motors and exit |

Every line sent or received is also written, with a PC timestamp, to
`logs/session_<date>_<time>.txt`.

## Parameters

Parameters are held in RAM on the robot (`StampC3_Template/RobotParams.h`).
The full list is in `config/robot_params.json`. In that file, keys that start
with `_` are treated as comments.

When the robot boots it loads the parameters saved with `save`. If nothing has
been saved, it uses the compiled defaults. Changing a geometry parameter
restarts the odometry count baseline but keeps the current pose.

The geometry values in `config/robot_params.json` come from the Pololu
dimensions drawing:

- wheel diameter 32.0 mm, so the radius is 16.0 mm;
- wheel separation 89.2 mm, which is the 96.0 mm outer width minus the 6.8 mm tyre width.

## Sensor calibration (normalised 0–1 readings)

Each surface sensor has its own pair of limits, `white_dn1..5` and `black_dn1..5`. The robot converts each reading like this:

```
norm = clamp((raw - white) / (black - white), 0, 1)     # 0 = white, 1 = black
```

The robot calculates this on every cycle. The results go into the `LOG` record (`dn1_norm..dn5_norm`) and into the line follower, whose `line_threshold` is now a 0–1 value (default 0.5). Raw readings are still in `DATA`, in `LOG` and in the trial CSV.

The default limits are the **mean** raw readings at 0% and 100% blackness over trials 1–6, about 3000 samples per surface. These differ from the extreme values in `additional_info/Reflectance_Sensor_Calibration_Report.xlsx`. With the extremes, a typical white reading would come out at about 0.08 and a typical black at about 0.92. With the means they come out at 0 and 1, and noise beyond the limits is clamped.

To recalibrate after collecting new 0% and 100% trials:

```powershell
python tests\calibrate_sensors.py telemetry.csv      # prints a table plus JSON for robot_params.json
```

Paste the JSON into `config/robot_params.json`, then run `load` and, if you want the values kept after a reboot, `save`.

## Sensor experiments (course exercises 4–7)

These follow https://semtm0043-42.github.io/reflectance.html. They need a one-time install for the analysis:

```powershell
python -m pip install -r requirements.txt     # numpy, matplotlib
```

`tests\experiments.py` walks you through every step, one prompt at a time:

- **Enter** collects a stationary batch, after a settling wait of `--settle`, default 0.5 s.
- **r** throws away the previous batch and redoes it.
- **q** stops. Every batch collected so far is already saved.

After each batch it prints the median and SD of all five sensors, so you can spot a misplaced robot straight away.

Each run writes two files to `data/experiments/`:

- `<protocol>_<time>.csv`, in the course's long format: `test_name, surface_value, trial_num, sample_num, sample_time, sensor_num, sensor_reading`, plus the extra columns `blackness_pct, role, position_mm, rectangle_width_mm, direction`. `surface_value` is the course fill value, 255 × (1 − blackness/100), where 0 is black. The file can be loaded straight into the course Colab notebooks.
- `<same>.meta.json`, with the robot's parameters, the arguments, the shuffle seed and the sensor-index mapping.

Run `tests\reflectance_analysis.py`, then open `reports/<analysis>/summary.md` together with its PNG plots.

### Gain and non-linearity (Ex 4 and 5): `sweep`

Print grey patches at **0, 10, 20, 40, 50, 60, 80, 90 and 100 % blackness**. These are the fit conditions 0/20/40/60/80/100 and the held-back conditions 10/50/90; you can change both lists with `--fit` and `--holdout`. Each patch must be large enough to cover all five sensors.

```powershell
python tests\experiments.py sweep --config config\robot_params.json
python tests\reflectance_analysis.py gain data\experiments\sweep_*.csv --fit-range 20 80
python tests\reflectance_analysis.py nonlinearity data\experiments\sweep_*.csv
```

By default there are 3 placement passes (`--placements`) of 100 samples each (`--samples`). The condition order is shuffled on every pass. For each placement, lift the robot and put it back down.

- **gain:** choose `--fit-range` from the plot, covering the region that looks linear. Don't choose it by chasing R². Held-back conditions are predicted and compared with the measurements.
- **nonlinearity:** compares a straight line `v̂=u`, a quadratic, and a power law `u^γ`, both per sensor and with one shared γ. The comparison is by held-back MAE.

### Measurement noise (Ex 6): `noise`

```powershell
python tests\experiments.py noise --points 20,60
python tests\reflectance_analysis.py noise data\experiments\noise_*.csv
```

This takes 3 placements at each operating point. Each placement is 1000 samples, about 10 s, and the third placement is held back. Keep the robot still and avoid shadows while it samples.

The report gives:
- per-trial statistics, including the number of distinct levels and the lag-1 autocorrelation;
- time series plots, then histograms with Gaussian and uniform models;
- the within-placement and between-placement spread, kept separate.

### Spatial footprint (Ex 7): `footprint`

Print the course strip sheet `printable_materials/testsheet_2.svg` at **100 % scale**. Measure the strip widths with a ruler before you start.

```powershell
python tests\experiments.py footprint --sensor 3 --widths 14,10,6,3,1 --from -20 --to 20 --step 1 --direction both
python tests\reflectance_analysis.py footprint data\experiments\footprint_*.csv --sensor 3
```

- It first takes white and black reference batches. For each strip it then prompts you to put the chosen sensor at each signed position from the strip centre; move the robot by hand against a ruler.
- Never move the robot while it is sampling. At each new traverse, realign the robot square to the strip.
- `--direction both` traverses each strip in both directions. `--repeats N` adds traverses after realignment.

The report gives:
- the profile z(position) for each strip width, with SD error bars;
- the peak z and the FWHM;
- the narrowest strip that still reaches z = 0.5;
- a repeatability table comparing the repeated traverses.

## Protocol

Each command is one line, ending with a newline.

- **Original:** `START TELEMETRY`, `STOP TELEMETRY`, `TRIAL,<n>,<pct>`, `STATUS`, `MOTOR,<l>,<r>`, `STOP`
- **Parameters:** `SET,<name>,<value>`, `GET,<name>`, `PARAMS`, `SAVE`, `DEFAULTS`
- **Logging:** `LOG START`, `LOG STOP`
- **Motion:** `MOVE,<mm>`, `ROTATE,<rad>`, `SELFTEST`, `ODOM_RESET[,x,y,theta]`

Responses:

- `DATA,...` has 14 fields: `timestamp_ms, x, y, theta, left_pwm, right_pwm,
  left_count, right_count, DN1..DN5, signal`.
- `LOG,...` has 26 fields: `timestamp_ms, read_ok_mask, DN1..DN5, left_count,
  right_count, left_pwm, right_pwm, odom_x, odom_y, odom_theta, mw_x, mw_y,
  mw_theta, mode, signal, motion_state, cfg_version, dn1_norm..dn5_norm`.
  The DN values are raw counts, and the `dn*_norm` values are calibrated to 0–1.
  - `read_ok_mask` shows which reads succeeded in that cycle: bit0 surface,
    bit1 encoders, bit2 middleware pose. 7 means all three succeeded.
  - `cfg_version` goes up on every parameter change, so each row can be
    matched to the parameters that were in use.
- `TRIAL_STARTED,<n>,<pct>,<count>`, then `TRIAL_SAMPLE,...` rows, then `TRIAL_DONE,<n>,<count>`.
- `PARAM,<name>,<value>`, and after a full listing `PARAMS_END,<version>`.
- `MOTION_DONE,<state>,<elapsed_ms>` and `SELFTEST_DONE,<state>,<masks...>,<deltas...>,<ranges...>`.
- `ACK,...` acknowledges a command. `ERROR,...` reports a problem, for example
  `ERROR,unknown_param,<name>` or `ERROR,out_of_range,<name>`.

Telemetry and logging are off by default. Both stop when the TCP client
disconnects. The firmware also stops the motors on disconnect if a non-zero
MOTOR command, a helper motion or the self-test is still running.

## Tests

The tests import `tcp_logger` directly, so run them from inside `tests/`:

```powershell
cd tests; python -m unittest
```
