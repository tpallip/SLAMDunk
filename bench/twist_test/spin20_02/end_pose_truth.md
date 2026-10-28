# spin20_02: end pose, hand reading vs odometry (2026-10-28)

Robot centre started at the origin, facing +x (y to the robot's left).
Read by hand after the 50 holds (about 105 turns, half each way):

| | x mm | y mm |
|---|---|---|
| front point | 105 | -125 |
| back point | 51 | -47 |
| centre (midpoint) | 78 | -86 |

Front to back 94.9 mm (robot diameter about 97 mm).

| | fwd mm | left mm | distance mm | heading deg |
|---|---|---|---|---|
| measured | 78 | -86 | 116 | -55.3 |
| odometry (encoders) | 75.5 | -67.7 | 101 | -38.1 |
| odometry - measured | -2.5 | +18.3 | | +17.2 |

Assumes the frame axes match the start heading; one end reading only, so
the error can't be split between spin rates.
