# Encoder check (plan 0.4)

Run 2026-10-13, robot on its stand, wheels clear. Firmware
`pranay/motor_characteristics` (ping/USB build). Data:
`logs/encoder_check_20261013.csv` (logger, every 10 ms cycle, 471 s). Script:
`analyse.py` (full output in `output.txt`). Battery charge and robot ID not
recorded.

**Verdict: the encoders are fit for characterisation.** Signs are right,
counts per wheel revolution match the Standard Edition 358.3, and every
reading is fresh. The 10 ms speed estimate is too noisy to use raw; use
≥ 50 ms windows.

## 1. Sign

Hand-turning a wheel forward and positive PWM both give rising counts, on
both wheels.

## 2. Counts per wheel revolution (hand turns, forward only)

| Wheel | Sets of 10 turns | Counts | Per revolution | vs 358.3 |
|---|---|---|---|---|
| Left | 1 (357 + 359 + 2861) | 3577 | 357.7 | −0.17 % |
| Right | 2 (3224 + 363; 2512 + 1077) | 3587, 3589 | 358.7, 358.9 | +0.13, +0.18 % |

Both are within the error of lining up a pen mark by eye over 10 turns
(±5°, about ±5 counts). Both wheels use the same 29.86:1 gearbox, so the
left/right difference is most likely reading error rather than physics.
**Keep 358.3.** The Phase 3 calibration measures mm per count from driven
distance, which also takes in each wheel's effective radius.

## 3. Freshness and speed precision (both wheels at constant PWM)

| PWM | Wheel | Speed (mm/s) | Counts / cycle | Repeated readings | Speed sd, 10 ms | 20 ms | 50 ms | 100 ms |
|---|---|---|---|---|---|---|---|---|
| 30 | L | 69.6 | 2.5 | 0 % | 21 % | 8.4 % | 4.3 % | 2.2 % |
| 30 | R | 93.9 | 3.4 | 0 % | 16 % | 8.5 % | 3.4 % | 2.0 % |
| 40 | L | 101.5 | 3.6 | 0 % | 16 % | 7.7 % | 3.9 % | 1.9 % |
| 40 | R | 134.5 | 4.8 | 0 % | 13 % | 7.5 % | 3.5 % | 1.8 % |
| 80 | L | 243.6 | 8.7 | 0 % | 11 % | 6.2 % | 1.8 % | 1.1 % |
| 80 | R | 287.1 | 10.2 | 0 % | 11 % | 6.0 % | 1.5 % | 1.0 % |

- **No stale readings.** No reading repeated the previous count while a
  wheel was turning. The count changes per cycle sit at the two whole numbers
  around the expected value, which is what 1-count quantisation alone would
  give.
- **Late cycles carry more counts, as they should.** 4 % of cycles are
  11–54 ms instead of 10 ms. Their counts scale with the longer interval, so
  speed must use the robot's actual elapsed time, never an assumed 10 ms.
- **Spread beyond quantisation at PWM 80**: 8–9 % of on-time cycles are off
  by more than 1.5 counts, by up to 5–6. The likely cause is the reading
  being taken at a varying point inside the cycle (after the surface-sensor
  I2C read), while `t_ms` is the cycle start. The `#B` rows should
  timestamp the encoder read itself with `micros()` (0.3). Real speed ripple
  (1.9) is the other candidate.

## 4. Seen in passing (inputs to Phase 1, not results)

- **Deadband on the stand, from rest:** neither wheel started at PWM 20.
  Left didn't start at 25 (twice) but did at 30. Right started at 25. This
  matches the earlier logs; to be measured properly in 1.2.
- **Left/right mismatch:** at the same PWM the right wheel is 18–35 %
  faster (30: 70 vs 94; 40: 102 vs 135; 80: 244 vs 287 mm/s). A shared
  gain won't do; the model must be per wheel.
- **Coast after `stop`:** from PWM 40, 24/31 counts (7/9 mm); from PWM 80,
  98/109 counts (27/31 mm) for L/R, on the stand with no load. Input to 1.8.

## Not done

Reverse hand turns and repeat sets (the plan asked for 3 sets per wheel per
direction); the reading error above makes more sets of little value. No
sign check by hand on the right wheel; it was confirmed with the motor.
