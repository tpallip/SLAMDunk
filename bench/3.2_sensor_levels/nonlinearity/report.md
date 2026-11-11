# Surface sensors: is the response linear? (Paul's reflectance 05)

2026-11-11, from runs we already had; no new test.
Script: `../nonlinearity.py`. Output: `output.txt`, `summary.csv`, `residuals.png`.

**Question.** Does each sensor's reading rise in a straight line with how
much of its spot sees black, or does it curve?

**Answer.** It is close enough to straight. The linear model's leftover bend
is at most 9-45 raw counts (0.5-2 % of each sensor's white-to-black range),
or about 0.3 mm of line position at the tape edge. A power law or a
quadratic doesn't predict held-back crossings better: rms changes by at most
4 counts. The shape that is left over is an S, the same on four sensors in
both runs. That looks like the spot not being quite Gaussian rather than a
curved response, so the linear model stays.

## Data

Two spin runs over the course tape:
- `logs/spread_profile_test2.csv`: 10-12 crossings per sensor.
- `logs/spread_cmd70_01.csv`: 19-20 crossings per sensor.

Readings at the 2942 cap are left out, which matters for dn5.

Why not use `readings_greyscale.csv`, the existing raw-vs-grey plot? Its
greyscale was worked out with a spot size fitted *assuming* a linear
response, so it is partly circular: any bend gets absorbed into the spot
size.

## Method

- **Crossing centre:** for each tape crossing, the centre is the midpoint of
  the two 50 % crossings. A reading lag shifts both crossings the same way,
  so it doesn't move the centre.
- **Coverage:** around that centre, the share c of a Gaussian spot over the
  tape.
- **Models:** three, fitted on equal terms:
  - linear: `raw = W + S·c`
  - power law: `raw = W + S·c^γ`
  - quadratic: `raw = W + S1·c + S2·c²`
- **Fitted with each model:** the spot size (σ, given the ruler's 18.5 mm
  tape width) and the tape's true length along the track, κ times the
  distance between the 50 % crossings. A linear response with a symmetric
  spot puts the 50 % crossings on the real edges, so κ ≈ 1.
- **Held-back test:** fit on every other crossing, test on the rest, then
  swap. The residuals of the held-back readings are averaged in 10 coverage
  bins.

## Results (held-back rms, raw counts)

| Run | Sensor | Linear | Power | Quadratic | Linear worst bin | Range |
|---|---|---|---|---|---|---|
| profile_test2 | dn1 | 59.7 | 58.4 | 58.8 | 31 | 1329 |
| | dn2 | 175.2 | 175.6 | 176.1 | 9 | 1963 |
| | dn3 | 43.3 | 42.6 | 42.5 | 16 | 1493 |
| | dn4 | 61.9 | 60.9 | 61.0 | 25 | 1543 |
| | dn5 | 48.4 | 45.7 | 46.4 | 42 | 2102 |
| cmd70_01 | dn1 | 37.0 | 36.9 | 36.9 | 14 | 1329 |
| | dn2 | 58.2 | 59.6 | 60.2 | 25 | 1963 |
| | dn3 | 45.1 | 44.4 | 44.1 | 16 | 1493 |
| | dn4 | 80.2 | 79.9 | 79.8 | 23 | 1543 |
| | dn5 | 52.4 | 48.3 | 50.1 | 45 | 2102 |

## What this says

- **Curved models buy nothing.** Held-back rms improves by 0-4 counts at
  most, and gets worse on dn2.
- **The power law can't be pinned down.** Its γ ran to the limits allowed
  (0.2-0.3, or 3-5 on dn2), with κ also at its limit. Response curvature,
  spot size and tape length trade off against each other, and these crossings
  can't tell them apart.
- **The leftover bend is an S, not a one-way curve.** Under the linear model
  readings sit 15-45 counts high at 20-50 % coverage and low at 70-90 %, on
  dn1, dn3, dn4 and dn5 in both runs. A curved response would bend one way
  only. An S that is symmetric about half coverage fits a spot with a
  sharper middle and wider tails than a Gaussian.
- **The scatter isn't sensor noise.** Residual rms (37-175) is 2-10 times
  the static noise (14-22 counts, `bench/3.1_static_noise`). It is
  crossing-to-crossing variation: odometry position, crossing angle, the
  tape itself. dn2 in the first run is the worst.

## What it means for the robot and the twin

- **Normalisation:** `n = (raw − white)/(black − white)` is fine as it is.
  No curve correction is needed.
- **Threshold:** with the linear model, a sensor reads 0.3 when its spot
  is 1.4-2.3 mm outside the tape's edge (dn5 1.4 mm, dn3 2.3 mm). With the
  quadratic it's about 0.5 mm. Either way DN3's "line seen" strip at 0.3 is
  about ±10-11.5 mm.
- **Tape width (16.5 fitted vs 18.5 by ruler):** the quadratic puts the 50 %
  crossings 4-6 % inside the real edges (κ 1.08-1.12), which would explain
  the mismatch exactly (18.5 / 16.5 = 1.12). But it doesn't predict
  held-back crossings any better, so this is a hint, not a result.
- **For the twin:** a linear response is right. If anything is worth
  modelling next it is the spot shape (wider tails), not a response curve.

## Limits

- Coverage comes from odometry along a spin, not from known printed greys.
  So the response curve and the spot shape can't be fully separated.
  Printed grey patches at known levels (the exercise's own method) would
  separate them; we chose not to run more tests.
- dn5 has fewer usable readings: its black is at the cap.
