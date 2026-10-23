# Motor characterisation: lokum (Phase 1 summary)

Open-loop model of each wheel of lokum (3pi+ 32U4 Standard Edition, 30:1
motors), measured on the floor with the robot spinning on the spot, so both
wheels carry its weight. Tests follow Paul's motor exercises 2-8
(<https://semtm0043-42.github.io/motors.html>), plus our own keep-turning,
coast-down and encoder checks. Firmware `pranay/motor_characteristics`,
10 ms control and telemetry. Battery voltage is not exposed, so sessions are
not corrected for it.

**Verdict:** a per-wheel model with a gain line, start and keep-turning
thresholds, and a dead time plus first-order lag predicts an unseen
sequence (with a reversal and a one-wheel stall) correctly in shape, timing
and every start/stall decision. Its steady speeds were 3-15 % low on a later
day, so the feedforward is a starting point and the speed loop must close
the rest.

## The model (floor, spin)

| | Left fwd | Left rev | Right fwd | Right rev | Source |
|---|---|---|---|---|---|
| Steady speed, mm/s | 3.419·u − 62.0 | 3.762·u − 75.2 | 3.852·u − 47.2 | 3.811·u − 45.1 | `1.5_gain`, floor 2026-10-14 |
| Fit / held-back error, mm/s | 6.1 / 3.8 | 4.2 / 2.5 | 6.2 / 2.7 | 4.9 / 2.8 | same |
| 50 % start PWM (always starts at) | 32.5 (40) | 32.5 (35) | 28.7 (35) | 32.5 (40) | `1.2_deadband`, floor |
| Keep-turning stop PWM, median [range] | 25 [23-27] | 23 [22-26] | 18 [17-19] | 18 [18-20] | `1.3_keepturn`, floor |

| Timing (dead time / tau, ms, median) | Left | Right | Source |
|---|---|---|---|
| From rest | 98 / 74 | 98 / 54 | `1.7_step_response/step_floor`, 330 steps, 4 runs |
| Up while moving | 59 / 67 | 56 / 56 | same |
| Down while moving | 40 / 110 | 50 / 50 | same |
| To PWM 0 | 68 / 44 | 62 / 42 | same |

- Up and down steps are both first-order (no slew limit). PWM 0 brakes:
  it stops the wheel faster than any lower PWM does.
- The left wheel is slower than the right, mostly when slowing down.
- **Coast to a stop** after PWM 0: 0.074·v + 0.00024·v² mm (left),
  0.066·v + 0.00021·v² mm (right); a stop lead of 0.075-0.089 s at
  40-60 mm/s and 0.11-0.12 s at 200 mm/s.
- **Encoders** (`0.4_encoder_quality`): signs right, 358.3 counts/rev kept,
  no stale reads. Speed over 10 ms has 11-21 % sd; use ≥ 30-50 ms windows
  and the real dt.

## Exercise by exercise

| # | Test | Result |
|---|---|---|
| 1.1 | Speed estimate (Ex 2) | From 0.4: ≥ 30-50 ms windows, actual elapsed time |
| 1.2 | Deadband (Ex 3) | Table above; breakaway set to 40 / 42 |
| 1.3 | Keep-turning (ours) | Table above; floor stops 6 PWM higher than the stand |
| 1.4 | Saturation (Ex 4) | **None in the usable range.** Gain is linear to PWM 100 (the firmware limit); the top step's slope is 101-114 % of the median. PWM 100 gives 290-340 mm/s, above the 250 mm/s command limit |
| 1.5 | Gain (Ex 5) | Table above; feedforward in `config/lokum.json` is the mean of both directions |
| 1.6 | Non-linearity (Ex 6) | Quadratic no better than linear on held-back levels (2.2-3.8 vs 2.5-3.8 mm/s). Power law not tried: nothing left to explain |
| 1.7 | Dynamics (Ex 7) | Table above; held-back 40/70 steps predicted within 8-9 mm/s (count noise) |
| 1.8 | Coast-down (ours) | Above |
| 1.9 | Ripple (ours) | Not done separately. Wobble at 50-80 mm/s is 3-10 % over 100 ms, at the encoder count limit at 80 mm/s, and doesn't change with the PI gains, so there's nothing for a filter to remove |
| 1.10 | Session drift (ours) | **Seen in the validation:** on 2026-10-23 the left wheel ran 14-24 mm/s (10-15 %) and the right 3-14 mm/s (3-8 %) faster than the 2026-10-14 gain lines, in both directions. Likely battery charge |
| 1.11 | Validation (Ex 8) | Below |

## Validation (Ex 8)

`bench/1.7_step_response/validation/`. The model was frozen in
`tools/motor_validate.py` and its prediction saved (`model.json`,
`prediction_nominal.csv`) before the run. The sequence used levels no test
had fitted (45, 75, 35, 21, 55, ±60, ±90, ±40, 65), a reversal, and PWM 21,
where the model said the left wheel would stall and the right keep turning.

- **Start/stall decisions: all 12 levels as predicted**, including the
  one-wheel stall at 21.
- **Timing and shape match** (`validate_floor_01/plot.png`), the reversal
  included.
- **Steady speed:** measured faster than predicted at every level, left by
  14-24 mm/s, right by 3-14 mm/s (the session drift above). Settled rms
  error 17.5 (L) and 9.8 (R) mm/s.

## Limits

- **No steady motion below about 35-40 mm/s.** At the lowest PWM that keeps
  a wheel turning it already runs at about 35 mm/s, so slower targets can
  only be averaged by stick-slip (65-75 % wobble at 20 mm/s whatever the
  run-min or gains). Command ≥ 40 mm/s, ideally ≥ 50.
- Steady speed per PWM moves by up to about 15 % between sessions, so the
  feedforward can't be trusted alone.
- Floor only. Stand timing wasn't measured.

## Closed-loop checks (Phase 2, for reference)

`bench/2.6_speed_control/floor_pi_*`, `floor_rm*`, 3-6 runs each:
kp 0.3 / ki 3 holds 30-80 mm/s to ±1 % mean; halving the gains halves PWM
jitter but not speed wobble and slows the rise by about 55 ms; a shorter
speed window changes nothing above 30 mm/s; run-min 28/21 pins 20-30 mm/s
at 35-42 mm/s. Config kept at kp 0.3, ki 3, run-min 19/14.
