#include "Controller.h"

const unsigned long Controller_c::UPDATE_INTERVAL_MS;
const unsigned long Controller_c::TELEMETRY_INTERVAL_MS;
const unsigned long Controller_c::RECOVERY_TIMEOUT_MS;
const uint16_t Controller_c::LINE_THRESHOLD;
constexpr float Controller_c::BASE_PWM;
constexpr float Controller_c::LINE_FOLLOW_GAIN;
constexpr float Controller_c::RECOVERY_LEFT_PWM;
constexpr float Controller_c::RECOVERY_RIGHT_PWM;
constexpr float Controller_c::MANUAL_MAX_PWM;
const unsigned long Controller_c::MANUAL_TIMEOUT_MS;

Controller_c::Controller_c()
  : update_timer(UPDATE_INTERVAL_MS),
    telemetry_timer(TELEMETRY_INTERVAL_MS) {
  reset();
}

void Controller_c::reset() {
  signal = 0;
  mode = WAITING;
  usr_mode = AUTO;
  recovery_start_ms = 0;
}

uint8_t Controller_c::getSignal() const {
  return signal;
}

void Controller_c::setSignal(uint8_t requested_signal) {
  if (requested_signal == 0) {
    reset();
    return;
  }

  if (signal == 0) {
    signal = 1;
    mode = FOLLOWING_LINE;
    recovery_start_ms = 0;
  }
}

void Controller_c::update(Robot_c &robot, RobotWifiAP_c &server) {

  // Get the current count of milliseconds since the StampC3 was powered on.
  unsigned long now = robot.getMillis();

  // Check if our TaskTimer_c for the general controller update indicates
  // that it is the correct time to operate.  Note, we give the TaskTimer_c
  // the current count in milliseconds.
  // We do this because we don't want to query the robot faster than at 10ms
  // intervals.
  if (!update_timer.isReady(now)) {

    // TaskTimer_c says it is not ready, so we simply end this call to
    // the controller here by calling return.
    return;
  }

  // The TaskTimer_c was ready, so we now reset the TaskTimer_c so it
  // begins counting again.  We now run the remaining controller code.
  update_timer.resetTimer(now);

  // Act on every complete command the client has sent since the last cycle.
  // readClient() never waits, so this costs nothing when no data has arrived.
  while(readClient(server) > 0) {
    handleCommand(robot, now);
  }

  // Ask the robot for latest information on surface reflectance sensors.
  robot.getSurfaceSensors();

  // Ask the robot for the latest encoder information, and then use this
  // to update the odometry on board the StampC3 (see OdometryModel.h).
  if (robot.getEncoders()) {
    odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
  }

  // Pose estimation from the Pololu 3Pi robot itself.
  robot.getPose();

  // While waiting, a button press sets signal to 1 and starts the physical
  // robot's line follower, even without Processing connected. The transmitted
  // signal also starts an armed Digital Twin live comparison. This is a start
  // control, not a start/stop toggle; further presses are ignored while signal is 1.
  if(usr_mode == AUTO) {
    if (signal == 0) {
      if (robot.isButtonPressed()) {
        setSignal(1);
      }
    }

    // The user has started the demonstration, so we run the line following
    // controller code.
    if (signal == 1) {
      runLineFollower(robot, now);

    }
  }
  else if(usr_mode == MANUAL_CTRL) {
    // Stop if the client has gone, or (when enabled) if "pwm" commands stopped
    // arriving, so a lost connection can't leave the robot driving.
    bool timed_out = MANUAL_TIMEOUT_MS > 0 && now - manual_cmd_ms > MANUAL_TIMEOUT_MS;
    if(!server.clientConnected() || timed_out) {
      manual_left_pwm = 0.0f;
      manual_right_pwm = 0.0f;
    }
    robot.setMotorPWM(manual_left_pwm, manual_right_pwm);
  }
  // Use another TaskTimer_c to limit how often we transmit telemetry data
  // on WiFi and Serial.
  if (telemetry_timer.isReady(now)) {
    telemetry_timer.resetTimer(now);
    publishTelemetry(robot, server, now);

  }
}

bool Controller_c::lineDetected(const Robot_c &robot) const {
  // Note: only using the central sensor to detect a line.
  if (robot.surface.reading[2] >= LINE_THRESHOLD) {
    return true;
  }

  return false;
}

void Controller_c::runLineFollower(Robot_c &robot, unsigned long now) {
  bool line_detected = lineDetected(robot);

  if (!line_detected && mode == FOLLOWING_LINE) {
    mode = SEARCHING_FOR_LINE;
    recovery_start_ms = now;
  }

  if (mode == SEARCHING_FOR_LINE) {
    if (line_detected) {
      mode = FOLLOWING_LINE;
    } else if (now - recovery_start_ms >= RECOVERY_TIMEOUT_MS) {
      mode = STOPPED;
    }
  }

  if (mode == STOPPED) {
    robot.setMotorPWM(0, 0);
    return;
  }

  if (mode == SEARCHING_FOR_LINE) {
    robot.setMotorPWM(RECOVERY_LEFT_PWM, RECOVERY_RIGHT_PWM);
    return;
  }

  float error = (float)robot.surface.reading[1] - (float)robot.surface.reading[3];
  float turn = error * LINE_FOLLOW_GAIN;
  robot.setMotorPWM(BASE_PWM - turn, BASE_PWM + turn);
}

void Controller_c::publishTelemetry(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms) {
  server.printf(
    "%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",
    timestamp_ms,
    odometry.pose.x,
    odometry.pose.y,
    odometry.pose.theta,
    robot.getLeftMotorPWM(),
    robot.getRightMotorPWM(),
    (long)robot.getLeftEncoderCount(),
    (long)robot.getRightEncoderCount(),
    robot.surface.reading[0],
    robot.surface.reading[1],
    robot.surface.reading[2],
    robot.surface.reading[3],
    robot.surface.reading[4],
    signal
  );
  if ( Serial ) { // If serial is connected
    Serial.printf(
      "%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",
      timestamp_ms,
      odometry.pose.x,
      odometry.pose.y,
      odometry.pose.theta,
      robot.getLeftMotorPWM(),
      robot.getRightMotorPWM(),
      (long)robot.getLeftEncoderCount(),
      (long)robot.getRightEncoderCount(),
      robot.surface.reading[0],
      robot.surface.reading[1],
      robot.surface.reading[2],
      robot.surface.reading[3],
      robot.surface.reading[4],
      signal
    );
  }
}

int Controller_c::readClient(RobotWifiAP_c &client) {
  if(!client.clientConnected()) {
    cmd_len = 0;
    return 0;
  }

  while(client.available() > 0) {
    int recvd_byte = client.read();
    if(recvd_byte < 0){
      return 0;
    }

    if(recvd_byte == '\n') {
      // Command complete: terminate it and keep a copy in mode_message.
      // An empty line leaves the previous mode_message in place.
      cmd_buf[cmd_len] = '\0';
      if(cmd_len > 0) {
        memcpy(mode_message, cmd_buf, cmd_len + 1);  // +1 copies the '\0'
      }
      int len = cmd_len;
      cmd_len = 0;
      return len;
    }
    else if(recvd_byte == '\r') {
      // Ignore the '\r' of "\r\n" line endings.
    }
    else if(cmd_len < sizeof(cmd_buf) - 1) {
      // Leave the last slot free for the '\0'.
      cmd_buf[cmd_len++] = (char)recvd_byte;
    }
  }

  return 0;
}

void Controller_c::handleCommand(Robot_c &robot, unsigned long now) {
  float left, right;

  if(strcmp(cmd_buf, "auto") == 0) {
    // reset() returns to waiting in AUTO; start with the button or "start".
    setSignal(0);
    robot.setMotorPWM(0, 0);
  }
  else if(strcmp(cmd_buf, "manual") == 0) {
    setSignal(0);  // leaves the line follower; also sets usr_mode to AUTO
    usr_mode = MANUAL_CTRL;
    manual_left_pwm = 0.0f;
    manual_right_pwm = 0.0f;
    manual_cmd_ms = now;
    robot.setMotorPWM(0, 0);
  }
  else if(strcmp(cmd_buf, "start") == 0) {
    if(usr_mode == AUTO) {
      setSignal(1);
    }
  }
  else if(strcmp(cmd_buf, "stop") == 0) {
    if(usr_mode == AUTO) {
      setSignal(0);
    }
    manual_left_pwm = 0.0f;
    manual_right_pwm = 0.0f;
    robot.setMotorPWM(0, 0);
  }
  else if(sscanf(cmd_buf, "pwm %f %f", &left, &right) == 2) {
    // isfinite rejects "pwm nan 0", which would reach the motors unclamped.
    if(usr_mode == MANUAL_CTRL && isfinite(left) && isfinite(right)) {
      manual_left_pwm = constrain(left, -MANUAL_MAX_PWM, MANUAL_MAX_PWM);
      manual_right_pwm = constrain(right, -MANUAL_MAX_PWM, MANUAL_MAX_PWM);
      manual_cmd_ms = now;
    }
  }
  else {
    Serial.printf("unknown command: %s\n", cmd_buf);
  }
}
