# Path reproduction: record a line-follower run, replay it with pure pursuit

2026-11-19. The group task: the robot follows a printed line from a start
pad to an end pad while its odometry is recorded, then drives the recorded
path again by odometry alone, with the line follower off. The line sensors
only watch during the replay, so they tell us how far the reproduction
drifted from the real line.

**Result:** after three fixes, a replay stays within **4.4 mm rms (8.9 mm
max)** of the line, against 2.6 mm rms for the follower itself on the same
recording. Before the last fix it was 18.8 mm rms (29.7 mm max).

| | Line offset rms | Max | Line seen |
|---|---|---|---|
| Follower, recording `run_20261119_234542` | 2.6 mm | 7.6 mm | 100 % |
| Replay before the S-curve start (`replay_01`, `replay_02`) | 18.8, 20.2 mm | 29.7, 29.4 mm | 81, 73 % |
| **Replay with the S-curve start (`replay_03`)** | **4.4 mm** | **8.9 mm** | **100 %** |

One replay with the final setup so far; repeats are still to do (see the
end).

## Why

The question for the twin project is how well a path driven by odometry
reproduces the real one. Recording with the follower gives a path that
really lies on the line; replaying it without the follower tests the
odometry and the path tracking, and the line is a free ground truth all the
way along. Every piece below is there to make that comparison fair: the same
course, the same start, the same speed, and the same way of starting and
stopping in both runs.

## What we built

| Piece | Where | What it does |
|---|---|---|
| Course | `tools/path_course.py`, `course_a4x6/` | Start pad, S-shaped line (curves of 90-110 mm radius), end pad. The pads are arcs round the axle at the sensors' radius, so all five sensors read black on a pad at any heading, and never on the line. |
| Curve fix | `line_ki_mm_s` (firmware, twin, `config/lokum.json`) | Integral term on the follower's turn, so it can hold the tight curves. |
| Start and stop | firmware `runLineFollower`, twin `Controller.pde` | Leaves the start pad, stops on the end pad (all five sensors black for 3 cycles while following). |
| S-curve start | `line_start_ramp_ms` 600 (firmware, twin, replay) | Speeds up from rest smoothly instead of jumping to full speed. |
| Replay | `tools/trajectory_controller.py` | Pure pursuit on the recorded odometry, through the logger's teleop and hub ports. |
| Ground truth | `tools/path_error.py` | Line offset at the sensors from the logged readings, for the recording and each replay. |

### Replay (`trajectory_controller.py`)

- Takes the recorded run's odometry (rows with `signal` 1: start pad to end
  pad), resamples it every 2 mm, and places it at the robot's pose when the
  replay starts.
- Puts the robot in manual mode, so the follower is off. Every telemetry
  record (10 ms) it finds the closest path point (forwards only), takes the
  point 50 mm further on, and sends `cmd v omega` with omega = v·2y/d².
  The inner wheel never goes below 40 mm/s, as in the follower.
- Speeds up on the same S-curve as the follower and sends `stop` at the end
  of the path, which cuts the PWM just as the follower does on the end pad.
- Wheel targets have no ramp (`accel_mm_s2 0` for the run), as in the
  follower.
- Saves every telemetry record with its own values to
  `logs/path_reproduction/replay_NN.csv`.

### Ground truth (`path_error.py`)

The line's sideways offset is fitted from all five readings with the
sensors' measured spots (the model in `line_position.py`, good to about
2 mm over ±40 mm). It is measured at the sensors, about 40 mm ahead of the
axle, so it includes heading error. The follower's own DN2 - DN4 is no use
as a ruler: it saturates about 8 mm off the line.

## What broke, and how we fixed it

### 1. The follower jerked through the curves

**Seen:** in `run_20261119_224828`, 19 search spins in 20 s, every one on a
curve, and a wobble of about 0.6 s on the 180 deg curve.

**Why:** the follower steers by DN2 - DN4 × `line_gain_mm_s` (55). That
signal saturates about 8 mm off the line (DN2 - DN4 ≈ 0.95), so the most turn
it can ask for is about ±52 mm/s. A 90 mm curve at 120 mm/s needs ±56 mm/s.
The robot drifted to the outside of every curve until DN3 lost the line
(DN3 at 0.30 and DN2 - DN4 at 0.84-0.89 just before each loss), spun to
find it again, and went on. The square course never needed this much turn.

**Tried:** slowing to 70 mm/s didn't help (5-13 spins). The inner wheel's
floor `line_min_mm_s` 40 then leaves only ±30 mm/s of turn, still short of
the 33 mm/s the curve needs at that speed.

**Fix:** an integral term, `turn = 55 × error + I`, with `I` growing by
`line_ki_mm_s × error × dt` and held within the same turn limit. On a long
curve it builds up the turn the curve needs, so the robot can ride near the
middle of the line. Tuned live with `set` (`run_20261119_225644`):

| `line_ki_mm_s` | Search spins to the end pad | Mean \|DN2 - DN4\| | Time saturated |
|---|---|---|---|
| 0 | 7-8 | 0.41-0.42 | 25-26 % |
| 150 | 6-10 | 0.34-0.45 | 15-21 % |
| **300** | **0, 0** | **0.23-0.27** | **0-1 %** |

The `ki` values weren't logged; they were worked out from the steering, and
the twin replay confirmed them (each run matches the robot exactly only at
its own value). 300 is in `lokum.json`.

### 2. The robot didn't stop at the end

**Seen:** it reached the end pad, lost the line, spun round and followed the
line back to the start.

**Fix:** a pad is black under all five sensors at once. Once the robot has
left the start pad, 3 cycles in a row on a pad stop the motors and send the
follower back to waiting; the robot reports `#I path: left the start pad`
and `#I path: end pad, stopped`. Picking the robot up reads the same, so it
stops then too.

**Watch out:** facing straight across the line during a search spin, all
five sensors can sit on it for a moment (they are only 13.5 mm deep and the
line is 18.5 mm wide), so the pad only counts while following. The
replay of older logs showed no other false trigger: every all-black stretch
was a pick-up (raw readings 2942).

**Check:** `run_20261119_231317`: started on the pad, off it after 0.18 s,
863 mm with no line loss, stopped on the end pad, coasted 12 mm.

### 3. The replay ran up to 30 mm off the line

**Seen:** `replay_01`: 18.8 mm rms, 29.7 mm max, +26 mm in the middle
turning to -23 mm, although pure pursuit kept the odometry within 2.3 mm of
the recorded path. So the robot really drove somewhere other than where its
odometry said.

**Why:** the whole offset curve is explained by one error at the start. A
fit of a single rotation and shift to the replay's offsets gives -3.5 deg
and 5 mm, and leaves 4.1 mm rms (from 18.8). The sign flips halfway because
the S-bend turns the line back across the rotation. The robot was placed
exactly on the start pad, so the rotation came from the take-off: from rest
both runs jumped to 120 mm/s, the speed controllers kicked both wheels to
PWM 90-100, the right wheel started first and the heading swung by several
degrees within 0.4 s (recording +3.9 then -4.8 deg, replay +2.5 then
-7.2 deg). At that PWM the wheels slip for a moment, which the encoders
can't see, so each start turns the robot by a different, unseen amount.

**Fix:** an S-curve start in both the follower and the replay: for the first
`line_start_ramp_ms` (600 ms) the speed is v = 120 × k²(3 - 2k), k going
0 to 1. Acceleration starts and ends at zero and peaks at 300 mm/s², the
same as the manual-mode ramp; it covers about 36 mm. There is no gentle way
to stop on the pad, so the stop stays as it is in both.

**Check:** start PWM now peaks at about 50. `replay_03` (new recording
`run_20261119_234542`): 4.4 mm rms, 8.9 mm max, line seen all the way.

## Is the replay really not the follower?

- `signal` is 0 on every replay row: the follower only runs in auto mode
  with signal 1, and the replay is in manual mode.
- The steering follows pure pursuit, not the line (wheel speed difference
  60 ms later):

| | Steering vs DN2 - DN4 | Steering vs pure pursuit's omega |
|---|---|---|
| Recording (follower) | R² = 0.27 | |
| `replay_03` | R² = 0.00 | R² = 0.95 |

- A physical check still to do: start the replay 20 mm to the side of the
  pad. The follower would steer back onto the line; the replay should run
  parallel to it, about 20 mm off all the way.

## The twin

Every follower change (integral, pads, S-curve) is mirrored in the twin's
`Controller.pde`. Replayed through `tools/twin_replay/` with the robot's
inputs, the twin sets exactly the same PWM as the robot: 100 % of the 736
following cycles of `run_20261119_234542` (integral, S-curve and end pad all
active), and of both follower runs in `run_20261119_231317`.

## How the tools were checked

Before robot time, the replay and `path_error.py` were run against a
simulated robot (unicycle, 60 ms command delay, sensors read off the
recorded path) with known errors put in:

| Put in | Line offset rms |
|---|---|
| Nothing (lookahead 50 mm) | 2.9 mm |
| Start 2 deg rotated | 9.5 mm |
| Wheel separation 1.3 % off | 5.2 mm, drifting to +7.9 mm at the end |

Lookahead: 30-40 mm 2.5-2.7 mm rms, 50 mm 2.9, 70 mm 4.3, 90 mm 7.1 (longer
cuts the 90 mm curves). The default is 50 for margin on the real robot.

## Watch out

- **Course scale.** The recorded path is about 864 mm from pad to pad; the
  a4x6 course file is about 950 mm, and the shapes don't match. The A4 tiles
  were probably printed with "fit to page" (about 91 %). Measure the 100 mm
  scale bar on the print before quoting course dimensions. The replay itself
  doesn't depend on it.
- **One replay with the final setup.** The 4.4 mm is one run.
- The offset is at the sensors, 40 mm ahead of the axle, so it mixes
  position and heading error.

## Still to do

- 3-5 replays of the same recording (and a second recording) for a spread.
- The 20 mm side-start check above.
- Lookahead 40 mm on the robot.
- Measure the course scale bar.

## Data

| File | What |
|---|---|
| `logs/run_20261119_224828.csv` | Follower jerking on the curves (before the fix) |
| `logs/run_20261119_225644.csv` | `line_ki_mm_s` 0 / 150 / 300 tuning runs |
| `logs/run_20261119_230047.csv` | 70 mm/s runs |
| `logs/run_20261119_231317.csv` | Pad start/stop, hard start; also holds `replay_01`/`02`'s telemetry |
| `logs/run_20261119_234542.csv` | Recording with the S-curve start, and `replay_03` |
| `logs/path_reproduction/replay_0*.csv` | Replays with the controller's values |
| `run_20261119_234542/` | `path_error.py` output: `offsets.csv`, `offsets.png`, `summary.txt` |
