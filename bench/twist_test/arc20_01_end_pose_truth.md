# arc20_01: hand readings vs odometry (2026-10-28)

Robot centre started at the origin, facing +x (y to the robot's left).

## After the right-hand arcs (holds 0-49)

Each (v, -w) followed by (-v, +w), 5 repeats of (110, 77), (130, 92),
(150, 105), (150, 125), (120, 90).

| | x mm | y mm |
|---|---|---|
| front point | 70 | -84 |
| back point | -6 | -23 |
| centre (midpoint) | 32 | -53.5 |

Front to back 97.5 mm.

| | fwd mm | left mm | distance mm | heading deg |
|---|---|---|---|---|
| measured | 32 | -53.5 | 62.3 | -38.8 |
| odometry (encoders) | 9.1 | -49.3 | 50.2 | -2.1 |
| odometry - measured | -22.9 | +4.2 | | +36.7 |
