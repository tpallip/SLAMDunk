/**
 * @brief One wheel's speed controller, line for line the same as
 * StampC3_Template/WheelSpeedController.cpp: feedforward from the measured
 * speed line, PI on the speed error, a kick from breakaway_pwm when the wheel
 * is stuck, a run_min_pwm floor and anti-windup.
 * @details Speed comes from the encoder counts over a window of about
 * min_counts counts (30 ms .. window_ms), as on the robot.
 */
class WheelSpeedController_c {
  float kp, ki;
  float ffOffsetPwm, ffMmSPerPwm;
  float maxPwm, breakawayPwm, runMinPwm;
  float kickRampPwmPerS = 200;
  long windowMs = 80;
  float minCounts = 6;
  long stallMinMs = 30;
  float accelMmS2 = 0;

  final int HISTORY = 16;    // encoder samples kept; covers windowMs at 10 ms per read

  float targetMmS = 0, setpointMmS = 0, speedMmS = 0, integralMm = 0, mmPerCount = 0;
  boolean kicking = false;
  float kickPwm = 0;
  int[] counts = new int[HISTORY];
  long[] times = new long[HISTORY];
  int samples = 0, newest = 0;
  long lastChangeMs = 0;

  WheelSpeedController_c(float kp, float ki, float ffOffsetPwm, float ffMmSPerPwm,
                         float maxPwm, float breakawayPwm, float runMinPwm) {
    this.kp = kp;
    this.ki = ki;
    this.ffOffsetPwm = ffOffsetPwm;
    this.ffMmSPerPwm = ffMmSPerPwm;
    this.maxPwm = maxPwm;
    this.breakawayPwm = breakawayPwm;
    this.runMinPwm = runMinPwm;
  }

  void setTarget(float newTargetMmS) {
    // An integral built up driving one way would push the wrong way after a
    // reversal, so start it again when the direction changes.
    if (newTargetMmS * targetMmS <= 0) integralMm = 0;
    targetMmS = newTargetMmS;
  }

  float getTarget() { return targetMmS; }
  float getSpeed() { return speedMmS; }

  // The feedforward PWM alone for a speed, without the PI or the kick.
  float feedforward(float speed) {
    if (speed == 0) return 0;
    float direction = speed > 0 ? 1 : -1;
    return direction * (ffOffsetPwm + abs(speed) / ffMmSPerPwm);
  }

  // Clears the target and integral. The speed estimate is kept.
  void reset() {
    targetMmS = 0;
    setpointMmS = 0;
    integralMm = 0;
  }

  void measure(int count, long nowMs, float mmPerCountIn) {
    mmPerCount = mmPerCountIn;
    if (samples > 0 && nowMs == times[newest]) return;
    if (samples == 0 || count != counts[newest]) lastChangeMs = nowMs;
    newest = (newest + 1) % HISTORY;
    counts[newest] = count;
    times[newest] = nowMs;
    if (samples < HISTORY) samples++;
    if (samples < 2) return;

    // Window long enough for about minCounts counts at the previous speed.
    long wantMs = windowMs;
    if (abs(speedMmS) > 0) {
      float msForCounts = 1000.0 * minCounts * mmPerCount / abs(speedMmS);
      wantMs = (long) constrain(msForCounts, 30, windowMs);
    }
    int delta = 0;
    long spanMs = 0;
    for (int k = 1; k < samples; k++) {
      int j = (newest - k + HISTORY) % HISTORY;
      delta = count - counts[j];
      spanMs = nowMs - times[j];
      if (spanMs >= wantMs) break;
    }
    float speed = spanMs > 0 ? delta * mmPerCount * 1000.0 / spanMs : 0;

    // No count for a while: the wheel is at most one count per that time.
    long quietMs = nowMs - lastChangeMs;
    if (quietMs > 0) {
      float bound = mmPerCount * 1000.0 / quietMs;
      if (abs(speed) > bound) speed = speed > 0 ? bound : -bound;
    }
    speedMmS = speed;
  }

  void stepSetpoint(float dtS) {
    float direction = targetMmS > 0 ? 1 : -1;
    if (accelMmS2 <= 0) {
      setpointMmS = targetMmS;
    } else if (setpointMmS * direction <= 0) {
      setpointMmS = direction * min(abs(targetMmS), accelMmS2 * dtS);
    } else if (abs(targetMmS) <= abs(setpointMmS)) {
      setpointMmS = targetMmS;
    } else {
      setpointMmS += direction * min(abs(targetMmS) - abs(setpointMmS), accelMmS2 * dtS);
    }
  }

  float update(float dtS) {
    if (targetMmS == 0) {
      kicking = false;
      integralMm = 0;
      setpointMmS = 0;
      return 0;
    }
    stepSetpoint(dtS);

    float direction = setpointMmS > 0 ? 1 : -1;
    float feedforward = direction * (ffOffsetPwm + abs(setpointMmS) / ffMmSPerPwm);
    float error = setpointMmS - speedMmS;

    // Stuck: push from the breakaway PWM upwards until the wheel moves.
    if (breakawayPwm > 0 && mmPerCount > 0 && samples > 1) {
      long countPeriodMs = (long) (1000.0 * mmPerCount / abs(setpointMmS));
      long allowedMs = Math.max(stallMinMs, countPeriodMs * 5 / 2);
      if (times[newest] - lastChangeMs > allowedMs) {
        float pi = abs(feedforward + kp * error + ki * integralMm);
        kickPwm = kicking ? kickPwm + kickRampPwmPerS * dtS : max(breakawayPwm, pi);
        kicking = true;
        return direction * min(kickPwm, maxPwm);
      }
    }
    kicking = false;

    float newIntegral = integralMm + error * dtS;
    float pwm = feedforward + kp * error + ki * newIntegral;

    // Floor: a wheel meant to be turning keeps at least runMinPwm.
    if (runMinPwm > 0 && pwm * direction < runMinPwm) {
      pwm = direction * runMinPwm;
      if (error * direction < 0) newIntegral = integralMm;
    }

    // Anti-windup.
    if (pwm > maxPwm) {
      pwm = maxPwm;
      if (error < 0) integralMm = newIntegral;
    } else if (pwm < -maxPwm) {
      pwm = -maxPwm;
      if (error > 0) integralMm = newIntegral;
    } else {
      integralMm = newIntegral;
    }
    return pwm;
  }
}
