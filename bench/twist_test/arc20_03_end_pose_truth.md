# arc20_03: hand readings vs odometry (2026-10-28)

Robot centre started at the origin, facing +x (y to the robot's left).

## After the right-hand arcs (holds 0-49)

Each (v, -w) followed by (-v, +w), 5 repeats of (110, 77), (130, 92),
(150, 105), (150, 125), (120, 90).

| | x mm | y mm |
|---|---|---|
| front point | -2 | 32 |
| back point | -88 | 73 |
| centre (midpoint) | -45 | 52.5 |

Front to back 95.3 mm.

| | fwd mm | left mm | distance mm | heading deg |
|---|---|---|---|---|
| measured | -45 | 52.5 | 69.1 | -25.5 |
| odometry (encoders) | -86.3 | 22.5 | 89.2 | -2.3 |
| odometry - measured | -41.3 | -30.0 | | +23.2 |

Odometry is the last logged pose in the frame of the pose just before
hold 0 (logs/motors/twist/arc20_03.csv).
