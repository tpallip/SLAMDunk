/**
 * @brief The template line follower, same as runLineFollower on the StampC3.
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
    update_timer = new TaskTimer_c(cfg.getInt("update_interval_ms", 10));
    telemetry_timer = new TaskTimer_c(cfg.getInt("telemetry_interval_ms", 10));
    odometry = new OdometryModel_c(cfg.getFloat("left_wheel_radius_mm", 16.0),
      cfg.getFloat("right_wheel_radius_mm", 16.0), cfg.getFloat("wheel_separation_mm", 90.0));
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
    }
  }

  void update(Robot_c robot) {
    telemetryLine = null;
    long now = robot.getMillis();
    if (!update_timer.isReady(now)) return;
    update_timer.resetTimer(now);

    robot.getSurfaceSensors();
    odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());

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

    if (!detected && mode == FOLLOWING) {
      mode = SEARCHING;
      recoveryStartMs = now;
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
      } else if (now - recoveryStartMs >= recoveryTimeoutMs) {
        mode = STOPPED;
      }
    }

    if (mode == STOPPED) {
      robot.setMotorPWM(0, 0);
      return;
    }

    if (mode == SEARCHING) {
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
    robot.setMotorPWM(basePwm - turn, basePwm + turn);
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
