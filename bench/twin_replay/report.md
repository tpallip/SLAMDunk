# Does the twin run the same controller as the robot?

2026-11-17. Step 0 of matching the twin to the robot: before comparing the
twin's models with the real world, check that the controller logic is the
same, so any difference found later can be put down to the models.

**Result:** yes. Given the robot's own inputs, the twin's controller sets
exactly the same motor PWMs on every one of 4,386 following cycles over three
runs, and its odometry agrees to 0.013 mm. The logic difference between the
robot and the twin is zero.

## Method

`tools/twin_replay/` is a headless Processing sketch. It loads the twin's
own `Controller.pde`, `WheelSpeedController.pde`, `OdometryModel.pde`,
`TaskTimer.pde` and the robot model files through symlinks into
`DigitalTwin/`, so it always tests the twin code as it stands.

For each row of a robot log it hands the twin's `Controller_c` what the
robot's controller saw in that cycle:

- the time (`t_ms`), so the twin's 10 ms update timer, speed windows and
  `dt` match the robot's real cycle times
- the encoder counts
- the five raw line sensor readings
- the start signal, set before the update as on the robot

No physics runs. The twin's PWM output for the cycle is then compared with
the PWM the robot logged in the same row. The odometry starts from the
robot's first logged pose and is compared on every row.

```
Processing cli --sketch=$PWD/tools/twin_replay --run [--config=x.json] logs/run_*.csv
```

It prints a summary per log and writes each compared row to
`logs/replay_<log>.csv`.

## Data

The three 120 mm/s template follower runs from 2026-11-12, after the speed
was set to 120 mm/s (`b5b307f`), with the logger sending `config/lokum.json`
on connect. The firmware's follower last changed in `7e8684f`. Copies of the
logs are in `robot_logs/`.

| Run | Rows | Not 10 ms apart | Following cycles | of which searching |
|---|---|---|---|---|
| 162434 | 3886 | 36 | 2089 | 578 |
| 163737 | 2474 | 18 | 978 | 450 |
| 163827 | 2835 | 25 | 1319 | 688 |

(A fourth log, 163349, never started: signal 0 throughout.)

## Results

| Run | PWM exact | Max PWM diff | Max position diff | Max heading diff |
|---|---|---|---|---|
| 162434 | 2089 / 2089 | 0 | 0.013 mm | 1e-5 rad |
| 163737 | 978 / 978 | 0 | 0.011 mm | 1e-5 rad |
| 163827 | 1319 / 1319 | 0 | 0.009 mm | 1e-5 rad |

Raw output: `summary.txt`.

Every PWM matched, including the parts most likely to differ:

- each switch from following to the search spin and back
- the spin direction picked from DN1/DN5 and the last steering error
- the feedforward-only window after a spin
- the speed controllers' encoder windows, kick and anti-windup

The odometry differences are single-precision rounding (the robot uses
`cosf` and a double `TWO_PI`, Processing a float one).

## Can the test fail?

To check it isn't passing trivially, the same logs were replayed with one
setting changed slightly:

| Change | PWM exact | Max PWM diff | Max position diff |
|---|---|---|---|
| none | 100 % | 0 | 0.013 mm |
| `line_gain_mm_s` 55 → 56 (+2 %) | 48-70 % | 4 | 0.013 mm |
| `wheel_separation_mm` 84.43 → 84.50 (+0.08 %) | 100 % | 0 | 1.3-5.3 mm |

A 2 % change in steering gain shows up in up to half the cycles, and a
0.07 mm change in wheel separation shows up in the odometry. So the match is
real, and it also confirms the robot ran with the config in the repo.

## What it doesn't cover

- **Late cycles while following.** All 79 rows that weren't 10 ms apart
  (11 ms up to 2.8 s, the long ones logger pauses) fell before the start
  signal. Every following cycle in these runs was exactly 10 ms, so a late
  cycle's `dt` going through the speed controllers is not tested here.

- **Only the line follower.** Manual driving, `move`, `turn` and the bench
  tests run only on the robot and have no twin version.
- **The encoder loss guard.** The robot stops following if the encoders
  haven't read for `ENCODER_LOSS_MS`. It never fired in these runs, and the
  twin has no such failure to guard against.
- **Failed I2C reads and writes.** On the robot a failed encoder read skips
  the speed controllers' update, and a failed motor write leaves the old PWM
  in the log. Neither happened in a way that changed a PWM here.

## What it means for matching the twin

Since the logic is identical, every difference between robot and twin runs
comes from the twin's world: sensor positions, sensor spot size, motor dead
time, wheel slip, motor speed against battery, and the delays in reading
sensors and sending commands. Those are the next steps.
