/**
 * @brief The line follower, same as runLineFollower on the StampC3: raw PWM
 * when line_speed_mm_s is 0, otherwise wheel speeds through the speed controllers.
 * @details Settings come from config/lokum.json so the twin and the robot
 * run with the same numbers. Defaults match StampC3_Template/RobotConfig.h.
 */
class Controller_c {
  final int WAITING = 0, FOLLOWING = 1, SEARCHING = 2, STOPPED = 3;
  int signal = 0, mode = WAITING;
  long recoveryStartMs = 0;
  boolean searchLeft = true;   // side the line was last seen on, so the search turns that way

  float lineLevel, basePwm, lineGain;
  float[] lineWhite = new float[5], lineBlack = new float[5];
  float[] lineNorm = new float[5];   // latest readings, 0 white to 1 black
  float recoveryLeftPwm, recoveryRightPwm, recoveryTimeoutMs;
  float searchBrakeMs, cornerSlow;
  float lineSpeedMmS, lineGainMmS, lineMinMmS;
  float searchSpeedMmS, searchDecelMmS2;
  long lastLineMs = 0;   // previous follower cycle, for the speed controllers' dt
  long followStartMs = 0;   // when following last (re)started
  float speedWindowMs;

  WheelSpeedController_c leftSpeed, rightSpeed;

  // the robot's own pose estimate, from encoder counts only
  OdometryModel_c odometry;

  TaskTimer_c update_timer;
  TaskTimer_c telemetry_timer;
  String telemetryLine = null;  // set on cycles where the robot would send telemetry

  Controller_c(JSONObject cfg) {
    float[] whites = {633, 666, 470, 643, 840};
    float[] blacks = {1962, 2629, 1963, 2186, 2942};
    for (int i = 0; i < 5; i++) {
      lineWhite[i] = cfg.getFloat("dn" + (i + 1) + "_white", whites[i]);
      lineBlack[i] = cfg.getFloat("dn" + (i + 1) + "_black", blacks[i]);
    }
    lineLevel = cfg.getFloat("line_level", 0.3);
    basePwm = cfg.getFloat("base_pwm", 45);
    lineGain = cfg.getFloat("line_gain", 17.5);
    recoveryLeftPwm = cfg.getFloat("recovery_left_pwm", -35);
    recoveryRightPwm = cfg.getFloat("recovery_right_pwm", 35);
    recoveryTimeoutMs = cfg.getFloat("recovery_timeout_ms", 5000);
    searchBrakeMs = cfg.getFloat("search_brake_ms", 0);
    cornerSlow = cfg.getFloat("corner_slow", 0.5);
    lineSpeedMmS = cfg.getFloat("line_speed_mm_s", 120);
    lineGainMmS = cfg.getFloat("line_gain_mm_s", 55);
    lineMinMmS = cfg.getFloat("line_min_mm_s", 40);
    searchSpeedMmS = cfg.getFloat("search_speed_mm_s", 0);
    searchDecelMmS2 = cfg.getFloat("search_decel_mm_s2", 1000);
    speedWindowMs = cfg.getFloat("speed_window_ms", 80);
    leftSpeed = speedController(cfg, "left");
    rightSpeed = speedController(cfg, "right");
    update_timer = new TaskTimer_c(cfg.getInt("update_interval_ms", 10));
    telemetry_timer = new TaskTimer_c(cfg.getInt("telemetry_interval_ms", 10));
    odometry = new OdometryModel_c(cfg.getFloat("left_wheel_radius_mm", 16.0),
      cfg.getFloat("right_wheel_radius_mm", 16.0), cfg.getFloat("wheel_separation_mm", 90.0));
  }

  // The same settings Controller_c::applyConfig gives the robot's speed controllers.
  WheelSpeedController_c speedController(JSONObject cfg, String side) {
    WheelSpeedController_c w = new WheelSpeedController_c(cfg.getFloat("speed_kp", 0.3), cfg.getFloat("speed_ki", 3.0),
      cfg.getFloat(side + "_ff_offset_pwm", 0), cfg.getFloat(side + "_ff_mm_s_per_pwm", 1.25),
      cfg.getFloat("manual_max_pwm", 100), cfg.getFloat(side + "_breakaway_pwm", 45),
      cfg.getFloat(side + "_run_min_pwm", 20));
    w.kickRampPwmPerS = cfg.getFloat("kick_ramp_pwm_per_s", 200);
    w.windowMs = (long) cfg.getFloat("speed_window_ms", 80);
    w.minCounts = cfg.getFloat("speed_min_counts", 6);
    w.stallMinMs = (long) cfg.getFloat("stall_min_ms", 30);
    return w;
  }

  void reset() {
    signal = 0;
    mode = WAITING;
    recoveryStartMs = 0;
    searchLeft = true;
  }

  int getSignal() {
    return signal;
  }

  void setSignal(int requested) {
    if (requested == 0) {
      reset();
      return;
    }
    if (signal == 0) {
      signal = 1;
      mode = FOLLOWING;
      recoveryStartMs = 0;
      leftSpeed.reset();
      rightSpeed.reset();
      lastLineMs = 0;
      followStartMs = 0;
    }
  }

  void update(Robot_c robot) {
    telemetryLine = null;
    long now = robot.getMillis();
    if (!update_timer.isReady(now)) return;
    update_timer.resetTimer(now);

    robot.getSurfaceSensors();
    odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
    leftSpeed.measure(robot.getLeftEncoderCount(), now, TWO_PI * odometry.leftRadiusMm / odometry.COUNTS_PER_REV);
    rightSpeed.measure(robot.getRightEncoderCount(), now, TWO_PI * odometry.rightRadiusMm / odometry.COUNTS_PER_REV);

    if (signal == 1) runLineFollower(robot, now);

    if (telemetry_timer.isReady(now)) {
      telemetry_timer.resetTimer(now);
      publishTelemetry(robot, now);
    }
  }

  void normaliseSurface(Robot_c robot) {
    for (int i = 0; i < 5; i++) {
      float span = max(lineBlack[i] - lineWhite[i], 1.0);
      lineNorm[i] = constrain((robot.surface.reading[i] - lineWhite[i]) / span, 0, 1);
    }
  }

  boolean lineDetected() {
    // Only the central sensor decides whether the line is there.
    return lineNorm[2] >= lineLevel;
  }

  void runLineFollower(Robot_c robot, long now) {
    normaliseSurface(robot);
    boolean detected = lineDetected();

    boolean speedMode = lineSpeedMmS > 0;
    float dtS = lastLineMs != 0 ? (now - lastLineMs) / 1000.0 : 0;
    lastLineMs = now;

    if (!detected && mode == FOLLOWING) {
      mode = SEARCHING;
      recoveryStartMs = now;
      // The speed controllers aren't updated during the spin, so their
      // integrals (the trim the feedforward misses) are kept for after it.
      // An outer sensor may still see where the line went.
      if (lineNorm[0] >= lineLevel) {
        searchLeft = true;
      } else if (lineNorm[4] >= lineLevel) {
        searchLeft = false;
      }
    }

    if (mode == SEARCHING) {
      if (detected) {
        mode = FOLLOWING;
        followStartMs = now;
      } else if (now - recoveryStartMs >= recoveryTimeoutMs) {
        mode = STOPPED;
      }
    }

    if (mode == STOPPED) {
      robot.setMotorPWM(0, 0);
      return;
    }

    if (mode == SEARCHING) {
      // Brake first (search_brake_ms, 0 by default).
      if (now - recoveryStartMs < searchBrakeMs) {
        robot.setMotorPWM(0, 0);
        return;
      }
      if (speedMode && searchSpeedMmS > 0) {
        // Spin at wheel speeds; the wheels slow into it at search_decel_mm_s2.
        leftSpeed.accelMmS2 = rightSpeed.accelMmS2 = 0;
        leftSpeed.decelMmS2 = rightSpeed.decelMmS2 = searchDecelMmS2;
        leftSpeed.setTarget(searchLeft ? -searchSpeedMmS : searchSpeedMmS);
        rightSpeed.setTarget(searchLeft ? searchSpeedMmS : -searchSpeedMmS);
        robot.setMotorPWM(leftSpeed.update(dtS), rightSpeed.update(dtS));
        return;
      }
      // recovery left/right PWMs spin left; swapped they spin right
      if (searchLeft) {
        robot.setMotorPWM(recoveryLeftPwm, recoveryRightPwm);
      } else {
        robot.setMotorPWM(recoveryRightPwm, recoveryLeftPwm);
      }
      return;
    }

    // Same coverage under DN2 and DN4 gives 0, whichever sensor reads higher.
    float error = lineNorm[1] - lineNorm[3];
    // DN2 darker: line on the left
    if (error != 0) {
      searchLeft = error > 0;
    }
    float turn = error * lineGain;
    // Slow down while DN1 or DN5 sees a corner's new line (corner_slow).
    float corner = max(lineNorm[0], lineNorm[4]);
    float slow = 1 - cornerSlow * corner;

    if (!speedMode) {
      float base = basePwm * slow;
      robot.setMotorPWM(base - turn, base + turn);
      return;
    }

    // Wheel speeds: each wheel's speed controller finds the PWM.
    float v = lineSpeedMmS * slow;
    float turnMmS = error * lineGainMmS;
    float maxTurn = max(v - lineMinMmS, 0);
    turnMmS = constrain(turnMmS, -maxTurn, maxTurn);
    // No setpoint ramps: they would delay every steering change.
    leftSpeed.accelMmS2 = rightSpeed.accelMmS2 = 0;
    leftSpeed.decelMmS2 = rightSpeed.decelMmS2 = 0;
    leftSpeed.setTarget(v - turnMmS);
    rightSpeed.setTarget(v + turnMmS);
    // Just after a search spin the speed estimate still shows the spin, and
    // the PI would drive both wheels hard forward: feedforward only until the
    // estimate's window has refreshed twice (one window still left a lurch).
    if (now - followStartMs < 2 * speedWindowMs) {
      robot.setMotorPWM(leftSpeed.feedforward(v - turnMmS), rightSpeed.feedforward(v + turnMmS));
      return;
    }
    robot.setMotorPWM(leftSpeed.update(dtS), rightSpeed.update(dtS));
  }

  // Same 14 fields as the robot's telemetry, so both logs go through the same tools.
  void publishTelemetry(Robot_c robot, long now) {
    int[] r = robot.surface.reading;
    telemetryLine = now + "," + String.format(java.util.Locale.ROOT, "%.2f,%.2f,%.5f",
      odometry.pose.x, odometry.pose.y, odometry.pose.theta)
      + "," + robot.motors.left.pwm + "," + robot.motors.right.pwm
      + "," + robot.getLeftEncoderCount() + "," + robot.getRightEncoderCount()
      + "," + r[0] + "," + r[1] + "," + r[2] + "," + r[3] + "," + r[4] + "," + signal;
  }
}
