# Bench results in plain words

What we have measured on our robot (lokum) so far, one short answer per
test. Each heading links to the detailed report and raw data. Numbers are
medians unless it says otherwise.

**The short version**

- The **link to the robot** is fast enough: commands take about 10 ms there
  and back, and telemetry arrives every 10 ms.
- The **motors** behave predictably: speed rises in a straight line with
  PWM, each wheel needs about 30 PWM to start, and each reacts about 0.1 s
  late. But the same PWM gives up to 15 % more or less speed on different
  days (battery), so the robot must measure and correct its speed, not
  trust a fixed PWM.
- The **wheels** are 16.26 mm in radius and 84.43 mm apart (calibrated).
  Driving straight and turning on the spot are now accurate to well under
  1 %.
- **Odometry** is good while the wheels grip, and goes wrong when they slip:
  after many fast arcs or spins the robot's heading was 17-37 deg off. On the line course it drifts about 1 deg a lap.
- The five **line sensors** are quiet (noise about 1 % of their range) and
  barely affected by room lighting. Each sees a spot about 3.5 mm across,
  and together they tell where the line is to about 2 mm.
- The **line follower** built on that laps the square without stopping. Its
  weak point is the corners, where the wheels jerked; a fix is in the
  firmware but not yet tested.

## Communication

### Command round trip ([`1.1_network_latency`](1.1_network_latency/report.md))
- **Question:** how long does a command take to reach the robot and get an
  answer, over WiFi or USB?
- **Answer:** about 10 ms over WiFi (17 ms for the slowest 5 %), 7 ms over
  USB. Half of it is waiting for the robot's next 10 ms cycle. The logger
  adds almost nothing (0.2 ms). We use WiFi.
- **Watch out:** the robot's 10 ms cycle sometimes runs up to 32 ms late, so
  anything that measures speed must use the real time between readings.

## Motors ([`motors_summary.md`](motors_summary.md))

### Encoders ([`0.4_encoder_quality`](0.4_encoder_quality/report.md))
- **Question:** can we trust the wheel encoders?
- **Answer:** yes. They count the right way, 358.3 counts per wheel turn as
  the datasheet says, and never repeat a stale reading. Speed worked out
  over a single 10 ms cycle is too jumpy (11-21 % noise); over 50 ms it is
  2-4 %.

### Starting and stopping (`1.2_deadband`, `1.3_keepturn`)
- **Question:** what is the smallest PWM that moves a wheel, and that keeps
  it moving?
- **Answer:** from rest a wheel needs about 29-33 PWM to start (always
  starts at 35-40). Once turning it keeps going down to about 18 (right) or
  23-25 (left). Below about 35-40 mm/s the wheels can't run smoothly at
  all: they stick and slip.

### Speed for a given PWM (`1.5_gain`)
- **Question:** how fast does each wheel go at each PWM?
- **Answer:** in a straight line, about 3.4-3.9 mm/s per PWM above the
  start point, with no limit up to PWM 100 (about 300 mm/s). The right
  wheel is faster than the left at the same PWM, so each wheel needs its
  own numbers.

### Reaction time (`1.7_step_response`)
- **Question:** how quickly does a wheel respond to a new PWM?
- **Answer:** nothing happens for about 0.1 s from rest (0.04-0.06 s when
  already moving), then the speed settles with a time constant of about
  0.04-0.11 s.
- **Check on unseen data:** the model predicted a new test sequence
  correctly in timing and in every start and stall, but its speeds were
  3-15 % low that day. That day-to-day drift is the main reason the robot
  uses closed-loop speed control.

### Speed control (`2.6_speed_control`)
- **Answer:** with the controller in `config/lokum.json` (kp 0.3, ki 3), the
  wheels hold 30-80 mm/s to within 1 % on average.

## Wheel geometry and odometry

### Wheel size and spacing ([`geometry`](geometry/geometry.md))
- **Question:** what wheel radius and wheel separation make the robot's
  odometry match reality?
- **Method:** drove 250 mm straight (7 runs) and spun 10 full turns (3 runs)
  on a printed millimetre grid, and read the robot's real position by hand.
- **Answer:** radius 16.26 mm (not 16.0) and separation 84.43 mm. With the
  old values the robot under-read distance by 1.6 % and over-turned by
  1.1 %. Now forward and reverse agree to about ±0.15 %.

### Odometry when the wheels slip (`twist_test`)
- **Question:** after a long run of fast curves or spins, does the robot
  still know where it is?
- **Answer:** not its heading. After 50 fast arcs it was 23-37 deg off, and
  after about 105 spins 17 deg off; its position was up to about 50 mm off.
  The likely cause is wheel slip, which the encoders can't see. Hand readings in
  [`arc20_01`](twist_test/arc20_01_end_pose_truth.md),
  [`arc20_03`](twist_test/arc20_03_end_pose_truth.md),
  [`spin20_02`](twist_test/spin20_02/end_pose_truth.md).

### Odometry on the line course ([`line_position`](line_position/report.md#odometry-on-the-long-run))
- **Answer:** over 14 laps the odometry heading crept by about 1.2 deg a lap
  (0.33 % of each 360 deg). The robot really follows the same square every
  lap, so this is odometry error.

## Line sensors

### Noise and room lighting ([`3.1_static_noise`](3.1_static_noise/report.md))
- **Question:** how much do the readings wobble with the robot still, and
  does room lighting matter?
- **Answer:** the wobble is about 14 counts on white and 17-22 on black,
  about 1 % of the range: under 0.1 mm of line position. Dimming the room
  shifts white by 11-37 counts (2-3 %). One fixed set of levels per sensor
  is enough.

### White and black levels (`3.2_sensor_levels`, `3.4_spatial_response`)
- **Answer:** each sensor reads white and black differently (white 470-840,
  black 1960-2940 on the course tape), so one threshold for all five puts
  the line's edge in a different place for each. The robot now scales each
  sensor to 0 (white) to 1 (black) with its own levels.

### How big a spot each sensor sees ([`3.4_spatial_response`](3.4_spatial_response/spread_cmd70_01/))
- **Method:** spun the robot over the course line and fitted how sharply
  each reading changes at the tape's edge.
- **Answer:** each sensor sees a blurred spot with a spread (sigma) of
  2.3-3.6 mm; going from white to black takes 6-9 mm of travel. The fit
  puts the tape at about 16.5 mm wide.
- **Open question:** a ruler says the tape is about 18.5 mm. The fit comes
  out 2 mm narrow, so the model is missing something: maybe the spot isn't
  really Gaussian, or the readings don't scale in a straight line between
  white and black. The robot still uses 16.5 (`line_width_mm`) because
  that's what fits its readings. The twin's map is 19 mm; moving it to 18.5
  made no clear difference in one run, so it's left until last.

## Line following ([`line_position`](line_position/report.md))

### Where the line is
- **Answer:** from all five scaled readings and each sensor's spot size,
  the robot works out how far the line is to its left or right. Checked on
  the spin run, it is right to about 2 mm (rms), over ±40 mm either side.
  It takes 0.12 ms a cycle on the robot.

### Following it
- **Answer:** steering on that position, the robot laps the square in
  8.2-8.7 s at 150 mm/s and ran 14 laps without stopping. The template
  follower turned round within 20 s.
- **Corners (2026-11-10, 90 mm/s):** the inner wheel's motor flipped between
  forward and reverse 7 times a second, and the robot lost the line about 5
  times a lap for 0.2 s. A fix is in the firmware (commit 559c237), not yet
  run on the robot.

## Not measured yet

Taken from [`TODO.md`](../TODO.md): readings at 10 ms with at least 1000
samples (3.1), black under low light (3.3), sensor readings in the twin vs
the robot (3.5), the odometry square test (3.9), the IMU (3.13-3.18) and
battery drift (3.20).
