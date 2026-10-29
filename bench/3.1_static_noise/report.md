# Surface sensors: static noise and room lighting

TODO.md 3.1, 3.3 (partial). Logged 2026-10-07, analysed 2026-10-29.

**Question.** How noisy are DN1-DN5 with the robot still, and how much does
room lighting move them?
**Answer.** About 14 raw counts sd on white and 17-22 on black, white noise
(no drift), about 1 % of each sensor's white-to-black span: under 0.1 mm of
line position. Dimming the room raises white readings by 11-37 counts, 2-3 %
of the span. Fixed per-sensor levels are enough; `LINE_THRESHOLD = 1400`
clears white in every lighting tested.

## Method

Robot still (encoders didn't move, PWM 0) on the course sheet, logger
recording, 50 ms telemetry (firmware of the time). Runs, in `raw/`:

| Run | Surface | Lighting | Records |
|---|---|---|---|
| `all_white` | all sensors on white | normal | 330 |
| `all_white_low_light` | white | room lights low | 274 |
| `all_white_low_light_blinds` | white | low, blinds down | 608 |
| `dn1_black` | dn1 on black | normal | 188 |
| `dn234_black` | dn2-dn4 on black | normal | 166 |
| `dn5_black` | dn5 on black | normal | 230 |

`analyse.py` writes `summary.csv` (every run and sensor). Battery, robot
height and the black surface used weren't recorded. Fewer than the 1000
samples per case TODO 3.1 asks for.

## Results (raw counts)

| Sensor | White | sd | Low light | Blinds | Black (static) | sd | Tape white | Tape black | Tape 50 % | Noise at edge |
|---|---|---|---|---|---|---|---|---|---|---|
| dn1 | 638 | 13.9 | +12 | +33 | 2022 | 21.8 | 633 | 1962 | 1298 | 0.08 mm |
| dn2 | 672 | 13.7 | +21 | +37 | 2942 (cap) | 0 | 666 | 2629 | 1648 | 0.06 mm |
| dn3 | 468 | 13.2 | +11 | +20 | 2472 | 16.8 | 470 | 1963 | 1216 | 0.08 mm |
| dn4 | 649 | 13.6 | +15 | +36 | 2796 | 21.3 | 643 | 2186 | 1414 | 0.07 mm |
| dn5 | 866 | 14.5 | +11 | +29 | 2942 (cap) | 0 | 840 | 2942 | 1891 | 0.04 mm |

Low light, blinds: change from normal light, on white. Tape: levels on the
course line from the line spin run (`bench/3.4_spatial_response/spread_cmd70_01`).
Noise at edge: white sd divided by the reading's slope at the tape edge's
50 % point, span / (sigma sqrt(2 pi)) per mm with the fitted footprint sigma.

- **Noise is white.** The sd from successive differences matches the sd
  (within about 2 counts), and the drift over each run is within ±6 counts.
- **Lighting.** Darker raises the reading (towards black), as expected of
  RC timing with less ambient light. The worst white, 937 (dn5, blinds), is
  far below 1400. The largest shift, +37, moves a sensor's 50 % point about
  0.3 mm. Black wasn't logged in low light.
- **Static black is not the tape.** dn3 and dn4 read 500-600 higher on the
  static black than on the tape, and dn2 hits the 2942 cap there but not on
  the tape. The static runs were on a darker or wider black. Calibrate from
  the tape; on it only dn5 hits the cap.
- **One threshold, five edges.** The tape 50 % points run from 1216 (dn3) to
  1891 (dn5). 1400 detects the line on every sensor but marks the edge at a
  different distance on each, so normalise per sensor for a position estimate.

## Still to do

One session at 10 ms telemetry, about 15 s each (about 1500 samples): robot
still on white and on the tape, normal and low light. That gives TODO 3.1
its sample count and black under low light for 3.3.
