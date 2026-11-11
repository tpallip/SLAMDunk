/**
 * @brief Both motors. Each wheel uses the speed line measured for the robot's
 * feedforward (ff_offset_pwm, ff_mm_s_per_pwm in lokum.json) and stops below
 * run_min_pwm.
 */
class MotorsModel_c {
  MotorModel_c left;
  MotorModel_c right;

  MotorsModel_c(JSONObject cfg) {
    left = new MotorModel_c(cfg.getFloat("left_ff_offset_pwm", 0), cfg.getFloat("left_ff_mm_s_per_pwm", 1.25),
      cfg.getFloat("left_run_min_pwm", 0));
    right = new MotorModel_c(cfg.getFloat("right_ff_offset_pwm", 0), cfg.getFloat("right_ff_mm_s_per_pwm", 1.25),
      cfg.getFloat("right_run_min_pwm", 0));
  }

  void setPWM(float leftPwm, float rightPwm) {
    left.setPWM(leftPwm);
    right.setPWM(rightPwm);
  }

  void update(int dtMs) {
    left.update(dtMs);
    right.update(dtMs);
  }

  void reset() {
    left.reset();
    right.reset();
  }
}

class MotorModel_c {
  // Wheels take roughly this long to get most of the way to a new speed
  // (bench/1.7_step_response: 40-110 ms depending on the case).
  final float LAG_MS = 70;

  float offsetPwm, mmPerSecPerPwm, minPwm;
  int pwm = 0;
  float speedMmS = 0;

  MotorModel_c(float offsetPwmIn, float mmPerSecPerPwmIn, float minPwmIn) {
    offsetPwm = offsetPwmIn;
    mmPerSecPerPwm = mmPerSecPerPwmIn;
    minPwm = minPwmIn;
  }

  // the robot truncates PWM to an int
  void setPWM(float value) { pwm = (int) constrain(value, -255, 255); }

  void reset() {
    pwm = 0;
    speedMmS = 0;
  }

  float steadySpeed() {
    if (abs(pwm) < minPwm) return 0;
    float v = max(0, (abs(pwm) - offsetPwm) * mmPerSecPerPwm);
    return pwm > 0 ? v : -v;
  }

  void update(int dtMs) {
    speedMmS += (steadySpeed() - speedMmS) * (1 - exp(-dtMs / LAG_MS));
  }
}
