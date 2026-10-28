# Wheel geometry calibration: lokum (plan Phase 3)

2026-10-23, floor, printed mm grid. Ground truth is two points on the robot,
its front and back, 96 mm apart on its centreline, read off the grid by hand
(about ±1 mm). Axes: +x along the robot's starting forward direction, +y to
its left, seen from above. Heading = direction of the back→front vector;
centre = midpoint of the two points. Encoder numbers are from
`tools/move_test.py report` on the logger CSVs in `logs/motors/geometry/`
(`motions.csv` in each folder here).

**Result:** wheel radius 16.26 mm both wheels (was 16.0), wheel separation
84.43 mm (was 84.0). Set in `config/lokum.json`.

## Wheel separation: 10 turns on the spot (`turn ±3600 40`)

Firmware arc per wheel = angle × b/2, with b = 84.0 and r = 16.0 at the time.
After 10 turns the robot's offset from its starting heading gives the true
angle, and b = 84 × encoder angle / true angle.

| Run | Encoder angle | Front end | Back end | Heading past start | True angle | b (r = 16.0) |
|---|---|---|---|---|---|---|
| spin10_ccw_01 | +3599.7° | (−17, 12) | (−91, −53) | +41.3° | +3641.3° | 83.04 |
| spin10_ccw_02 | +3601.2° | (−16, 19.5) | (−90, −44) | +40.6° | +3640.6° | 83.09 |
| spin10_cw_01 | −3600.4° | (−1, −26) | (−75, 35) | −39.5° | −3639.5° | 83.10 |

Front started at (0, 0), back at (−96, 0). b = **83.08 ± 0.03 mm** at
r = 16.0, the same both ways, so turns overshot by 1.1 % with b = 84. The
centre drifted 12-55 mm over 10 turns (it doesn't affect the angle).
`spin10_ccw_interrupted.csv` is a first attempt stopped after 7 turns.

## Wheel radius: 250 mm straight (`move ±250 60`)

Forward runs: front from (−269, 0), back from (−365, 0), facing +x.
Reverse runs: back of the robot at (−269, 0), front at (−365, 0), facing −x,
reversing towards +x. Scale = centre's true travel / encoder travel (mean
of both wheels, r = 16.0).

| Run | Encoder L / R mm | Front end | Back end | True travel | Scale | Encoder heading | True heading |
|---|---|---|---|---|---|---|---|
| fwd250_02 | 249.4 / 250.0 | (−15, 6) | not read | about 254 | about 1.017 | +0.39° | — |
| fwd250_03 | 250.6 / 249.7 | (−15, 1.5) | (−111, 1.5) | 254.0 | 1.0154 | −0.58° | 0.0° |
| fwd250_04 | 252.5 / 249.7 | (−15, −6) | (−110, −1) | 254.5 | 1.0135 | −1.94° | −3.0° |
| fwd250_05 | 249.4 / 251.1 | (−15, −6) | (−111, −4.5) | 254.05 | 1.0152 | +1.16° | −0.9° |
| back250_01 | 251.4 / 249.4 | (−108, 2) | (−13, 6) | 256.5 | 1.0245 | +1.35° | +2.4° |
| back250_02 | 250.8 / 250.6 | (−110, 0) | (−14, 0) | 255.0 | 1.0172 | +0.19° | 0.0° |
| back250_03 | 252.0 / 250.3 | (−109, −3) | (−14, −2.5) | 255.5 | 1.0174 | +1.16° | +0.3° |

(fwd250_01 has encoder data only: no reading was taken.)

- Mean scale forward 1.0147, reverse 1.0173 without back250_01 (likely a
  start-position error). Median of all six full readings **1.0163**, so
  r = 16 × 1.0163 = **16.26 mm**; forward and reverse then within about
  ±0.15 %.
- Left/right radius difference: unequal radii would make true − encoder
  heading flip sign between forward and reverse. Forward mean −0.85°,
  reverse 0.0°: −0.4 ± 0.5° per 250 mm, or a 0.25 ± 0.3 % radius
  difference. Not resolvable; both radii kept equal.
- The ±1-2° heading change per run shows on the encoders too: uneven
  wheel starts and stops, not wheel size.

## Config change

- `left_wheel_radius_mm`, `right_wheel_radius_mm`: 16.0 → 16.26.
- `wheel_separation_mm`: 84.43 = 83.08 × 1.0163. The spin test measured
  b / r, so b scales with r and turn angles stay as calibrated.
- `*_ff_mm_s_per_pwm`: × 1.0163 (3.60 → 3.66, 3.75 → 3.81), since the same
  counts now read as 1.63 % more mm/s. Offsets (PWM) unchanged.

For comparison: ruler about 84 mm; Pololu's CAD drawing gives 85.2 mm
between tyre centres and 32 mm wheels (r = 16).
