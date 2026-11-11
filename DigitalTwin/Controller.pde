/**
 * @brief The template line follower, same as runLineFollower on the StampC3.
 * @details Settings come from config/lokum.json so the twin and the robot
 * run with the same numbers. Defaults match StampC3_Template/RobotConfig.h.
 */
class Controller_c {
  final int WAITING = 0, FOLLOWING = 1, SEARCHING = 2, STOPPED = 3;
  int signal = 0, mode = WAITING;
  long recoveryStartMs = 0;

  float lineThreshold, basePwm, lineFollowGain;
  float recoveryLeftPwm, recoveryRightPwm, recoveryTimeoutMs;

  // the robot's own pose estimate, from encoder counts only
  OdometryModel_c odometry;

  TaskTimer_c update_timer;
  TaskTimer_c telemetry_timer;
  String telemetryLine = null;  // set on cycles where the robot would send telemetry

  Controller_c(JSONObject cfg) {
    // the twin still follows on raw counts; its settings sit under "_twin"
    JSONObject twin = cfg.hasKey("_twin") ? cfg.getJSONObject("_twin") : cfg;
    lineThreshold = twin.getFloat("line_threshold", 1400);
    basePwm = cfg.getFloat("base_pwm", 45);
    lineFollowGain = twin.getFloat("line_follow_gain", 0.01);
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

  boolean lineDetected(Robot_c robot) {
    return robot.surface.reading[2] >= lineThreshold;
  }

  void runLineFollower(Robot_c robot, long now) {
    boolean detected = lineDetected(robot);
    if (!detected && mode == FOLLOWING) {
      mode = SEARCHING;
      recoveryStartMs = now;
    }
    if (mode == SEARCHING && detected) {
      mode = FOLLOWING;
    }
    if (mode == SEARCHING && now - recoveryStartMs >= recoveryTimeoutMs) {
      mode = STOPPED;
    }

    if (mode == STOPPED) {
      robot.setMotorPWM(0, 0);
      return;
    }
    if (mode == SEARCHING) {
      robot.setMotorPWM(recoveryLeftPwm, recoveryRightPwm);
      return;
    }

    float turn = (robot.surface.reading[1] - robot.surface.reading[3]) * lineFollowGain;
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
