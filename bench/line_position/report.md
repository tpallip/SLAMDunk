# Line offset from the five surface sensors, and a line follower on it

TODO.md 3.2 (per-sensor normalisation) and the line position estimate.
Built and checked on the line spin run 2026-10-29, then run on the robot.

**Question.** Where is the line, in mm sideways, from DN1-DN5, and does
steering on it follow the course better than the template follower?
**Answer.** Normalise each sensor with its own white and black levels, then
fit the line's offset to the five readings with the measured footprints. On
the spin run it agrees with the spin's own geometry to +0.5 mm median, 2.2 mm
rms, over about ±40 mm. On the robot it takes 120 µs a cycle and gives the
same numbers as the Python mirror. A pure pursuit follower on it laps the
square in 8.2-8.7 s at 150 mm/s in either direction and ran 14 laps
(123 s) without stopping; the template follower turned round and went the
other way within 20 s, searching for 5.8 s of it.

## Line offset

- **Normalised reading**: n = (raw - white) / (black - white), held to 0..1,
  per sensor. Levels on the course tape from the line spin run
  (`bench/3.4_spatial_response/spread_cmd70_01`), now `dn<k>_white`,
  `dn<k>_black` in `config/lokum.json`.
- **Offset**: sensors at y = 31.8, 10.9, 0, -10.9, -31.8 mm (45 mm circle,
  45/14/0/-14/-45 deg). A line of width w (16.5 mm) with its middle at e
  reads n(e) = Phi((y - e + w/2)/sigma) - Phi((y - e - w/2)/sigma) with each
  sensor's footprint sigma (`dn<k>_sigma_mm`). The offset is the e that
  matches the five readings best (least squares), searched every 2 mm over
  ±50 mm, then every 0.5 mm within 2 mm of the best, and refined with a
  parabola. A weak pull towards the previous offset picks the side when only
  DN1 or DN5 sees the line. The line counts as seen when any sensor reads
  ≥ 0.5 (`line_detect_level`).
- A weighted average of the readings (centroid) was the alternative. It
  reads 9.9 mm for a line 13 mm off and 10.8 mm for 16 mm, and pins at 31.8 mm
  once only DN1 sees the line; the fit has no such flat spots.
- Firmware: `StampC3_Template/LineSensor.*`, run every cycle, answered by
  the `line` command (`#L offset_mm=.. residual=.. detected=.. norm=..
  us=.. max_us=..`). Python mirror: `tools/line_position.py`.

### Check on the spin run (`check_spin.py`)

Truth from the spin: for each turn, the spin centre's distance from the
line's middle (c) and the line's direction are fitted to all the readings
(fit rms 0.06-0.08). Where the robot faces along the line (within 3 deg) the
line's true offset at the sensors is about ±c; the estimate is compared with
that (143 records): median error +0.46 mm, rms 2.21 mm, p95 |error| 4.5 mm.

The spin centre walked from 5.5 to 18.5 mm off the line over the 10 turns
(about 1.3 mm per turn), so a single c for the run isn't the truth for any
one moment. The error grows a little with c (about +1.5 mm at 16-18.5 mm),
where the line is between DN1 and DN2. The rms includes up to 3 deg of
misalignment (±2 mm at 42 mm ahead) and drift within a turn.

### On the robot

Robot still on the line, `line` asked every 100 ms (50 replies each,
`follower_20261029/raw/line_replies_*.txt`):

| Search | Time per update, median | Longest |
|---|---|---|
| float, interpolated table | 2716 µs | 3896 µs |
| integer table on the search grid (flashed) | 120 µs | 1673 µs |

The first version took over a quarter of the 10 ms cycle: the ESP32-C3 has
no FPU, and each update did about 300 float divisions and interpolations.
Tabulating each sensor's n(e) on the search grid as integers (n × 1000) at
configure time makes the search integer sums. The occasional long update is
probably an interrupt (WiFi, I2C) landing in it. The robot's offsets match
the Python mirror on the same logged readings (4.04 mm, sd 0.06, same
normalised readings).

## Line follower (`line_mode` 1)

- **Seen / lost**: the line sensor's `detected` (any sensor ≥ 0.5), not DN3
  alone over `line_threshold`.
- **Steering**: pure pursuit on the point the sensors see, the line's middle
  at (L, e) with L = 42 mm ahead of the wheels: curvature
  `line_kp` × 2e / (L² + e²), plus `line_kd` (deg/s per mm/s) on the
  offset's rate of change.
- **Speed**: through the calibrated wheel speed controllers (not raw PWM), the
  outer wheel held at `line_speed_mm_s`, so the robot slows on curves; wheel
  targets ramp at `line_accel_mm_s2` (1000).
- **Lost line**: spins at `line_search_deg_s` towards the side the line was
  last seen on (the template always spins left), stops after
  `recovery_timeout_ms`.
- `line_mode` 0 keeps the template follower (raw PWM, DN2 - DN4, DN3 for
  detection), for comparison.

Settings were picked in a simulation of the twin's course (centreline about
301 × 212 mm, sharp corners, 17 mm tape, measured footprints, levels, noise
and wheel lags): speed 100 / kp 1.5 / kd 0 first, then 150 / 1.5 / 0.5
(predicted 12.8 and 8.9 s a lap).

### Runs on the square (`follower_20261029/`)

Same robot and course, one session; `analyse.py` rebuilds the table and
`paths.png` from `raw/runs.csv`. Lost and |offset| come from the line sensor
re-run on the logged readings.

| Run | Follower | Duration | Laps | Speed | s / lap | Line lost | Searching | Longest |
|---|---|---|---|---|---|---|---|---|
| `new_100` | offset, 100 / kp 1.5 / kd 0 | 20 s | 1.52 cw | 78 mm/s | 13.2 | 12 | 0.9 s | 0.11 s |
| `old` | template, base_pwm 45 | 20 s | turned round | 58 mm/s | — | 19 | 5.8 s | 1.16 s |
| `new_150` | offset, 150 / 1.5 / 0.5 | 20 s | 2.44 acw | 106 mm/s | 8.2 | 11 | 1.5 s | 0.22 s |
| `new_150_button` | offset, 150 / 1.5 / 0.5 | 123 s | 14.08 acw | 108 mm/s | 8.7 | 57 | 8.0 s | 0.24 s |

- **Template follower**: started clockwise, spun in loops at the first
  corners (DN3 leaves the line at every corner and the search always turns
  left), then followed the line anticlockwise: the way it came.
- **Offset follower**: clean laps that land on top of each other, in the
  direction it starts in. It loses the line for 0.1-0.2 s at each 90 deg corner
  (about 4 a lap) and finds it again turning the right way. The long run
  (started with the button, stopped by hand) held 2.25-2.47 turns per 20 s
  throughout and was on the line when stopped.
- The first 3 s run (`new_100_3s`): offset +6 mm crossed 0 within 0.3 s,
  overshot once to -3 mm and settled by 0.9 s, 100 mm/s on both wheels on the
  straight, inner wheel reversing through the corner. Its first cycle sent
  -26/-19 PWM for 10 ms (the speed ramp hadn't moved yet); fixed since by
  skipping the motors on that cycle (not in these runs).

### Odometry on the long run

The laps drawn from odometry rotate: the straights' heading (mod 90 deg) goes
+1.6 deg on the first lap to +16.8 deg on the 14th, a steady +1.2 deg a lap
anticlockwise, about 0.33 % of the 360 deg turned. The robot follows the same
square every lap, so this is odometry heading error, the same kind as the
arc runs' (`bench/twist_test/arc20_*_end_pose_truth.md`).

## Still to do

- Straight passes across the line at known angles (`cmd 60 0`) against the
  grid: the direct check of the offset, without the spin geometry.
- Corners: the 0.1-0.2 s losses could go with a longer search pull or by
  using the residual (it jumps to about 0.2 at a corner) to turn early.
- Same follower in the DigitalTwin (TODO 3.5).
