# Line offset from the five surface sensors

TODO.md 3.2 (per-sensor normalisation) and the line position estimate.
Built and checked 2026-10-29 on the line spin run.

**Question.** Where is the line, in mm sideways, from DN1-DN5?
**Answer.** Normalise each sensor with its own white and black levels, then
fit the line's offset to the five readings with the measured footprints.
On the spin run it agrees with the spin's own geometry to +0.5 mm median,
2.2 mm rms, over the whole range the sensors cover (about ±40 mm).

## Method

- **Normalised reading**: n = (raw - white) / (black - white), held to 0..1,
  per sensor. Levels on the course tape from the line spin run
  (`bench/3.4_spatial_response/spread_cmd70_01`), now `dn<k>_white`,
  `dn<k>_black` in `config/lokum.json`.
- **Offset**: sensors at y = 31.8, 10.9, 0, -10.9, -31.8 mm (45 mm circle,
  45/14/0/-14/-45 deg). A line of width w (16.5 mm) with its middle at e
  reads n(e) = Phi((y - e + w/2)/sigma) - Phi((y - e - w/2)/sigma) with each
  sensor's footprint sigma (`dn<k>_sigma_mm`). The offset is the e that
  matches the five readings best (least squares), searched every 0.5 mm over
  ±50 mm and refined with a parabola. The line counts as seen when any sensor
  reads ≥ 0.5 (`line_detect_level`).
- A weighted average of the readings (centroid) was the alternative. It
  reads 9.9 mm for a line 13 mm off and 10.8 mm for 16 mm, and pins at 31.8 mm
  once only DN1 sees the line; the fit has no such flat spots.
- Firmware: `StampC3_Template/LineSensor.*`, run every cycle, answered by
  the `line` command (`#L offset_mm=.. residual=.. detected=.. norm=..`).
  Each sensor's n(e) is tabulated when the config changes, so a cycle costs
  no erf calls (the ESP32-C3 has no FPU). Python mirror:
  `tools/line_position.py`, which writes `line.csv` (and `line.png`).

## Check on the spin run (`check_spin.py`)

Truth from the spin: for each turn, the spin centre's distance from the
line's middle (c) and the line's direction are fitted to all the readings
(fit rms 0.06-0.08). Where the robot faces along the line (within 3 deg) the
line's true offset at the sensors is about ±c; the estimate is compared with
that (143 records).

| | |
|---|---|
| median error | +0.46 mm |
| rms error | 2.21 mm |
| p95 \|error\| | 4.5 mm |

The spin centre walked from 5.5 to 18.5 mm off the line over the 10 turns
(about 1.3 mm per turn), so a single c for the run (14 mm, from
`sensor_spread`) isn't the truth for any one moment. The error grows a
little with c (+0.0 mm at 5.5, about +1.5 mm at 16-18.5), where the line is
between DN1 and DN2 and fewer sensors see it. The rms includes up to 3 deg
of misalignment (±2 mm at 42 mm ahead) and drift within a turn.

## Still to do

Straight passes across the line at known angles (`cmd 60 0`) against the
grid: the direct check, without the spin geometry in between.
