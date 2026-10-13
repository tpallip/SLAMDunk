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
constexpr float Controller_c::MANUAL_MAX_SPEED_MM_S;
constexpr float Controller_c::SPEED_KP;
constexpr float Controller_c::SPEED_KI;
constexpr float Controller_c::LEFT_BREAKAWAY_PWM;
constexpr float Controller_c::RIGHT_BREAKAWAY_PWM;
constexpr float Controller_c::LEFT_RUN_MIN_PWM;
constexpr float Controller_c::RIGHT_RUN_MIN_PWM;
constexpr float Controller_c::LEFT_FF_OFFSET_PWM;
constexpr float Controller_c::LEFT_FF_MM_S_PER_PWM;
constexpr float Controller_c::RIGHT_FF_OFFSET_PWM;
constexpr float Controller_c::RIGHT_FF_MM_S_PER_PWM;
constexpr float Controller_c::MOVE_KP;
constexpr float Controller_c::MOVE_MAX_SPEED_MM_S;
constexpr float Controller_c::MOVE_MIN_SPEED_MM_S;
constexpr float Controller_c::MOVE_TOLERANCE_MM;
constexpr float Controller_c::MOVE_STOP_LEAD_S;
constexpr float Controller_c::MOVE_MAX_DISTANCE_MM;

Controller_c::Controller_c()
  : left_speed(SPEED_KP, SPEED_KI, LEFT_FF_OFFSET_PWM, LEFT_FF_MM_S_PER_PWM,
               MANUAL_MAX_PWM, LEFT_BREAKAWAY_PWM, LEFT_RUN_MIN_PWM),
    right_speed(SPEED_KP, SPEED_KI, RIGHT_FF_OFFSET_PWM, RIGHT_FF_MM_S_PER_PWM,
                MANUAL_MAX_PWM, RIGHT_BREAKAWAY_PWM, RIGHT_RUN_MIN_PWM),
    mover(MOVE_KP, MOVE_MAX_SPEED_MM_S, MOVE_MIN_SPEED_MM_S, MOVE_TOLERANCE_MM, MOVE_STOP_LEAD_S),
    update_timer(UPDATE_INTERVAL_MS),
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

  // Act on every complete command the client has sent since the last cycle,
  // over WiFi or typed on USB Serial. The readers never wait, so this costs
  // nothing when no data has arrived.
  while(readClient(server) > 0) {
    handleCommand(robot, server, cmd_buf, FROM_WIFI, now);
  }
  while(readSerial() > 0) {
    handleCommand(robot, server, serial_buf, FROM_SERIAL, now);
  }

  // Ask the robot for latest information on surface reflectance sensors.
  robot.getSurfaceSensors();

  // Ask the robot for the latest encoder information, and then use this
  // to update the odometry on board the StampC3 (see OdometryModel.h).
  // timestamp the middle of the read for the bench tests
  unsigned long enc_start_us = micros();
  enc_ok = robot.getEncoders();
  enc_us = enc_start_us + (micros() - enc_start_us) / 2;
  if (enc_ok) {
    odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());

    // Wheel speeds for the "vel" command, using the odometry's wheel geometry.
    float counts_per_rev = odometry.encoderCountsPerRevolution;
    left_speed.measure(robot.getLeftEncoderCount(), now,
                       TWO_PI * odometry.leftWheelRadiusMm / counts_per_rev);
    right_speed.measure(robot.getRightEncoderCount(), now,
                        TWO_PI * odometry.rightWheelRadiusMm / counts_per_rev);
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
  else if(usr_mode == MOTOR_TEST) {
    runMotorTest(robot, server, now);
  }
  else if(usr_mode == MANUAL_CTRL) {
    // Stop if the client has gone, or (when enabled) if "pwm"/"vel" commands
    // stopped arriving, so a lost connection can't leave the robot driving.
    // A "move" ends by itself (or at its own deadline), so it isn't resent
    // and the command timeout doesn't apply while it runs. Serial can't tell
    // whether anyone is listening, so when driven over USB only the timeout
    // applies.
    bool timed_out = MANUAL_TIMEOUT_MS > 0 && !mover.isActive()
                     && now - manual_cmd_ms > MANUAL_TIMEOUT_MS;
    bool link_lost = manual_source == FROM_WIFI && !server.clientConnected();
    if(link_lost || timed_out) {
      clearManual();
    }
    if(mover.isActive()) {
      float left_target, right_target;
      if(!mover.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount(), now,
                       left_target, right_target)) {
        reply(server, manual_source, "#I move %s: left %+.2f mm, right %+.2f mm still to go\n",
              mover.timedOut() ? "timed out" : "done",
              mover.getLeftError(), mover.getRightError());
      }
      left_speed.setTarget(left_target);
      right_speed.setTarget(right_target);
    }
    if(manual_vel) {
      float dt_s = (now - last_manual_ms) / 1000.0f;
      manual_left_pwm = left_speed.update(dt_s);
      manual_right_pwm = right_speed.update(dt_s);
    }
    last_manual_ms = now;
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

void Controller_c::clearManual() {
  manual_left_pwm = 0.0f;
  manual_right_pwm = 0.0f;
  left_speed.reset();
  right_speed.reset();
  mover.cancel();
}

void Controller_c::startDeadbandTest(Robot_c &robot, RobotWifiAP_c &server, const char *cmd,
                                     CmdSource source, unsigned long now) {
  DeadbandTest_c::Params p;
  int rest_ms = (int)p.rest_ms, run_ms = (int)p.run_ms;
  if(sscanf(cmd, "test deadband %c %d %d %d %d %d %d %d", &p.wheel, &p.dir, &p.pwm_min,
            &p.pwm_max, &p.pwm_step, &p.repeats, &rest_ms, &run_ms) < 1) {
    reply(server, source, "#E usage: test deadband <L|R> [dir min max step repeats rest_ms run_ms]\n");
    return;
  }
  p.rest_ms = rest_ms < 0 ? 0 : rest_ms;
  p.run_ms = run_ms < 0 ? 0 : run_ms;
  int32_t count = p.wheel == 'L' ? robot.getLeftEncoderCount() : robot.getRightEncoderCount();
  const char *problem = deadband.start(p, now, count);
  if(problem != nullptr) {
    reply(server, source, "#E test deadband: %s\n", problem);
    return;
  }

  setSignal(0);
  clearManual();
  robot.setMotorPWM(0, 0);
  usr_mode = MOTOR_TEST;
  test_source = source;
  test_rows = 0;
  reply(server, source, "#BH deadband wheel=%c dir=%d min=%d max=%d step=%d repeats=%d "
        "rest_ms=%lu run_ms=%lu still_ms=%lu trials=%d\n",
        p.wheel, p.dir, p.pwm_min, p.pwm_max, p.pwm_step, p.repeats, p.rest_ms, p.run_ms,
        DeadbandTest_c::STILL_MS, p.repeats * deadband.levelsPerSweep());
  reply(server, source, "#BC trial,repeat,phase,wheel,pwm,t_ms,enc_us,enc_ok,enc_l,enc_r,rest_ok\n");
}

void Controller_c::runMotorTest(Robot_c &robot, RobotWifiAP_c &server, unsigned long now) {
  // stop if nobody is listening, or on the button
  if(test_source == FROM_WIFI && !server.clientConnected()) {
    endMotorTest(robot, server, now, "link lost");
    return;
  }
  if(robot.isButtonPressed()) {
    endMotorTest(robot, server, now, "button");
    return;
  }

  const DeadbandTest_c::Params &p = deadband.getParams();
  int32_t count = p.wheel == 'L' ? robot.getLeftEncoderCount() : robot.getRightEncoderCount();
  deadband.update(now, count);
  if(!deadband.active()) {
    endMotorTest(robot, server, now, "done");
    return;
  }

  int pwm = deadband.getPwm();
  robot.setMotorPWM(p.wheel == 'L' ? pwm : 0, p.wheel == 'R' ? pwm : 0);
  bool running = deadband.getPhase() == DeadbandTest_c::RUN;
  reply(server, test_source, "#B,%d,%d,%s,%c,%d,%lu,%lu,%d,%ld,%ld,%d\n",
        deadband.getTrial(), deadband.getRepeat(), running ? "run" : "rest", p.wheel, pwm,
        now, enc_us, enc_ok ? 1 : 0, (long)robot.getLeftEncoderCount(),
        (long)robot.getRightEncoderCount(), running && deadband.getRestOk() ? 1 : 0);
  test_rows++;
}

void Controller_c::endMotorTest(Robot_c &robot, RobotWifiAP_c &server, unsigned long now,
                                const char *why) {
  if(deadband.active()) {
    deadband.abort();
  }
  robot.setMotorPWM(0, 0);
  reply(server, test_source, "#BE deadband %s rows=%lu trials=%d\n", why, test_rows,
        deadband.getTrial());
  usr_mode = MANUAL_CTRL;
  manual_vel = false;
  clearManual();
  manual_source = test_source;
  manual_cmd_ms = now;
  last_manual_ms = now;
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

int Controller_c::readSerial() {
  while(Serial.available() > 0) {
    int recvd_byte = Serial.read();
    if(recvd_byte < 0) {
      return 0;
    }

    if(recvd_byte == '\n') {
      serial_buf[serial_len] = '\0';
      if(serial_len > 0) {
        memcpy(mode_message, serial_buf, serial_len + 1);  // +1 copies the '\0'
      }
      int len = serial_len;
      serial_len = 0;
      return len;
    }
    else if(recvd_byte == '\r') {
      // Ignore the '\r' of "\r\n" line endings (the Serial Monitor sends them).
    }
    else if(serial_len < sizeof(serial_buf) - 1) {
      serial_buf[serial_len++] = (char)recvd_byte;
    }
  }

  return 0;
}

void Controller_c::reply(RobotWifiAP_c &server, CmdSource to, const char *format, ...) {
  char buffer[192];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if(to == FROM_SERIAL) {
    Serial.print(buffer);
  } else {
    server.printf("%s", buffer);
  }
}

void Controller_c::handleCommand(Robot_c &robot, RobotWifiAP_c &server, const char *cmd,
                                 CmdSource source, unsigned long now) {
  float left, right;
  unsigned long ping_id;

  // don't let other motion commands fight a running test
  if(usr_mode == MOTOR_TEST) {
    if(strcmp(cmd, "stop") == 0 || strcmp(cmd, "manual") == 0 || strcmp(cmd, "auto") == 0) {
      endMotorTest(robot, server, now, cmd);
      if(strcmp(cmd, "auto") == 0) {
        setSignal(0);
      }
      return;
    }
    if(strncmp(cmd, "pwm ", 4) == 0 || strncmp(cmd, "vel ", 4) == 0 ||
       strncmp(cmd, "move ", 5) == 0 || strncmp(cmd, "test ", 5) == 0 || strcmp(cmd, "start") == 0) {
      reply(server, source, "#E a test is running; send stop first\n");
      return;
    }
  }

  if(strcmp(cmd, "auto") == 0) {
    // reset() returns to waiting in AUTO; start with the button or "start".
    setSignal(0);
    robot.setMotorPWM(0, 0);
  }
  else if(strcmp(cmd, "manual") == 0) {
    setSignal(0);  // leaves the line follower; also sets usr_mode to AUTO
    usr_mode = MANUAL_CTRL;
    manual_vel = false;
    clearManual();
    manual_cmd_ms = now;
    manual_source = source;
    last_manual_ms = now;
    robot.setMotorPWM(0, 0);
  }
  else if(strcmp(cmd, "start") == 0) {
    if(usr_mode == AUTO) {
      setSignal(1);
    }
  }
  else if(strcmp(cmd, "stop") == 0) {
    if(usr_mode == AUTO) {
      setSignal(0);
    }
    clearManual();
    robot.setMotorPWM(0, 0);
  }
  else if(sscanf(cmd, "pwm %f %f", &left, &right) == 2) {
    // isfinite rejects "pwm nan 0", which would reach the motors unclamped.
    if(usr_mode == MANUAL_CTRL && isfinite(left) && isfinite(right)) {
      manual_vel = false;
      mover.cancel();
      left_speed.reset();
      right_speed.reset();
      manual_left_pwm = constrain(left, -MANUAL_MAX_PWM, MANUAL_MAX_PWM);
      manual_right_pwm = constrain(right, -MANUAL_MAX_PWM, MANUAL_MAX_PWM);
      manual_cmd_ms = now;
      manual_source = source;
    }
  }
  else if(sscanf(cmd, "vel %f %f", &left, &right) == 2) {
    if(usr_mode == MANUAL_CTRL && isfinite(left) && isfinite(right)) {
      manual_vel = true;
      mover.cancel();
      left_speed.setTarget(constrain(left, -MANUAL_MAX_SPEED_MM_S, MANUAL_MAX_SPEED_MM_S));
      right_speed.setTarget(constrain(right, -MANUAL_MAX_SPEED_MM_S, MANUAL_MAX_SPEED_MM_S));
      manual_cmd_ms = now;
      manual_source = source;
    }
  }
  else if(sscanf(cmd, "move %f", &left) == 1) {
    if(usr_mode == MANUAL_CTRL && isfinite(left)) {
      float counts_per_rev = odometry.encoderCountsPerRevolution;
      manual_vel = true;
      mover.start(constrain(left, -MOVE_MAX_DISTANCE_MM, MOVE_MAX_DISTANCE_MM),
                  robot.getLeftEncoderCount(), robot.getRightEncoderCount(),
                  TWO_PI * odometry.leftWheelRadiusMm / counts_per_rev,
                  TWO_PI * odometry.rightWheelRadiusMm / counts_per_rev, now);
      manual_cmd_ms = now;
      manual_source = source;
    }
  }
  else if(strncmp(cmd, "test deadband", 13) == 0) {
    startDeadbandTest(robot, server, cmd, source, now);
  }
  else if(sscanf(cmd, "ping %lu", &ping_id) == 1) {
    // Answered in the same cycle, so the round trip includes the wait for the
    // next 10 ms update (as every other command does) but nothing else.
    reply(server, source, "#P %lu %lu\n", ping_id, now);
  }
  else {
    reply(server, source, "#E unknown command: %s\n", cmd);
  }
}
