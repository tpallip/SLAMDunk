#include "Controller.h"


Controller_c::Controller_c()
  : left_speed(0, 0, 0, 1, 0, 0, 0),
    right_speed(0, 0, 0, 1, 0, 0, 0),
    mover(0, 0, 0, 0, 0) {
  applyConfig();
  reset();
}

void Controller_c::begin(const char *id) {
  robot_id = id;
  bool loaded = config.load();
  applyConfig();
  Serial.printf("%s: %s\n", robot_id, loaded ? "loaded saved config" : "no saved config, using defaults");
}

void Controller_c::applyConfig() {
  const RobotConfigValues_t &c = config.values;

  update_timer.setIntervalMS((unsigned long)c.update_interval_ms);
  telemetry_timer.setIntervalMS((unsigned long)c.telemetry_interval_ms);

  odometry.leftWheelRadiusMm = c.left_wheel_radius_mm;
  odometry.rightWheelRadiusMm = c.right_wheel_radius_mm;
  odometry.wheelSeparationMm = c.wheel_separation_mm;

  WheelSpeedController_c *wheels[2] = {&left_speed, &right_speed};
  for (int i = 0; i < 2; i++) {
    bool left = i == 0;
    WheelSpeedController_c &w = *wheels[i];
    w.kp = c.speed_kp;
    w.ki = c.speed_ki;
    w.ff_offset_pwm = left ? c.left_ff_offset_pwm : c.right_ff_offset_pwm;
    w.ff_mm_s_per_pwm = left ? c.left_ff_mm_s_per_pwm : c.right_ff_mm_s_per_pwm;
    w.max_pwm = c.manual_max_pwm;
    w.breakaway_pwm = left ? c.left_breakaway_pwm : c.right_breakaway_pwm;
    w.run_min_pwm = left ? c.left_run_min_pwm : c.right_run_min_pwm;
    w.kick_ramp_pwm_per_s = c.kick_ramp_pwm_per_s;
    w.window_ms = (unsigned long)c.speed_window_ms;
    w.min_counts = c.speed_min_counts;
    w.stall_min_ms = (unsigned long)c.stall_min_ms;
    w.accel_mm_s2 = c.accel_mm_s2;
  }

  mover.kp = c.move_kp;
  mover.max_speed_mm_s = c.move_max_speed_mm_s;
  mover.min_speed_mm_s = c.move_min_speed_mm_s;
  mover.tolerance_mm = c.move_tolerance_mm;
  mover.stop_lead_s = c.move_stop_lead_s;
  mover.turn_stop_lead_s = c.turn_stop_lead_s;
  mover.sync_kp = c.move_sync_kp;

  twist.kp = c.twist_kp;
  twist.max_error_mm = c.twist_max_error_mm;
  twist.max_speed_mm_s = c.manual_max_speed_mm_s;
  twist.accel_mm_s2 = c.accel_mm_s2;
  twist.stop_speed_mm_s = c.move_min_speed_mm_s;
  twist.left_stop_lead_s = c.twist_left_lead_s;
  twist.right_stop_lead_s = c.twist_right_lead_s;

  const float whites[LineSensor_c::N] = {c.dn1_white, c.dn2_white, c.dn3_white, c.dn4_white, c.dn5_white};
  const float blacks[LineSensor_c::N] = {c.dn1_black, c.dn2_black, c.dn3_black, c.dn4_black, c.dn5_black};
  const float sigmas[LineSensor_c::N] = {c.dn1_sigma_mm, c.dn2_sigma_mm, c.dn3_sigma_mm, c.dn4_sigma_mm, c.dn5_sigma_mm};
  for (int i = 0; i < LineSensor_c::N; i++) {
    line.white[i] = whites[i];
    line.black[i] = blacks[i];
    line.sigma_mm[i] = sigmas[i];
  }
  line.line_width_mm = c.line_width_mm;
  line.detect_level = c.line_detect_level;
  line.configure();
}

void Controller_c::reset() {
  signal = 0;
  mode = WAITING;
  usr_mode = AUTO;
  recovery_start_ms = 0;
  left_speed.accel_mm_s2 = config.values.accel_mm_s2;   // the line follower uses its own
  right_speed.accel_mm_s2 = config.values.accel_mm_s2;
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
    left_speed.reset();
    right_speed.reset();
    left_speed.accel_mm_s2 = config.values.line_accel_mm_s2;
    right_speed.accel_mm_s2 = config.values.line_accel_mm_s2;
    last_line_ms = 0;   // no previous cycle yet
    last_offset_mm = line.offset_mm;
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
  line.update(robot.surface.reading);

  // Ask the robot for the latest encoder information, and then use this
  // to update the odometry on board the StampC3 (see OdometryModel.h).
  // timestamp the middle of the read for the bench tests
  unsigned long enc_start_us = micros();
  enc_ok = robot.getEncoders();
  enc_us = enc_start_us + (micros() - enc_start_us) / 2;
  if (enc_ok) {
    last_enc_ok_ms = now;
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
    // and the command timeout doesn't apply while it runs; nor while "cmd 0 0"
    // slows down to a stop, which takes under a second. Serial can't tell
    // whether anyone is listening, so when driven over USB only the timeout
    // applies.
    unsigned long timeout_ms = (unsigned long)config.values.manual_timeout_ms;
    bool timed_out = !mover.isActive() && !twist.isStopping() && now - manual_cmd_ms > timeout_ms;
    bool link_lost = manual_source == FROM_WIFI && !server.clientConnected();
    if(link_lost || timed_out) {
      clearManual();
    }
    // Checked after the commands above, so a "vel" that arrives while the
    // encoders are out stops before it drives.
    bool driving = manual_vel && (mover.isActive() || twist.isActive() || left_speed.getTarget() != 0.0f
                                  || right_speed.getTarget() != 0.0f);
    if(driving && now - last_enc_ok_ms > ENCODER_LOSS_MS) {
      clearManual();
      reply(server, manual_source, "#E no encoder reading for %lu ms, stopped\n", now - last_enc_ok_ms);
    }
    if(mover.isActive()) {
      float left_target, right_target;
      if(!mover.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount(), now,
                       left_target, right_target)) {
        reply(server, manual_source, "#I %s %s: left %+.2f mm, right %+.2f mm still to go\n",
              move_name, mover.timedOut() ? "timed out" : "done",
              mover.getLeftError(), mover.getRightError());
      }
      left_speed.setTarget(left_target);
      right_speed.setTarget(right_target);
    }
    if(twist.isActive()) {
      float left_target, right_target;
      twist.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount(),
                   (now - last_manual_ms) / 1000.0f, left_speed.getSpeed(), right_speed.getSpeed(),
                   left_target, right_target);
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
  if (robot.surface.reading[2] >= config.values.line_threshold) {
    return true;
  }

  return false;
}

void Controller_c::startMove(Robot_c &robot, const char *name, float left_mm, float right_mm,
                             float speed_mm_s, CmdSource source, unsigned long now) {
  float counts_per_rev = odometry.encoderCountsPerRevolution;
  manual_vel = true;
  move_name = name;
  float max_speed = config.values.manual_max_speed_mm_s;
  mover.start(left_mm, right_mm, constrain(fabsf(speed_mm_s), 0.0f, max_speed),
              robot.getLeftEncoderCount(), robot.getRightEncoderCount(),
              TWO_PI * odometry.leftWheelRadiusMm / counts_per_rev,
              TWO_PI * odometry.rightWheelRadiusMm / counts_per_rev, now, strcmp(name, "turn") == 0);
  twist.stop();
  manual_cmd_ms = now;
  manual_source = source;
}

void Controller_c::clearManual() {
  manual_left_pwm = 0.0f;
  manual_right_pwm = 0.0f;
  left_speed.reset();
  right_speed.reset();
  mover.cancel();
  twist.stop();
}

void Controller_c::startDeadbandTest(Robot_c &robot, RobotWifiAP_c &server, const char *cmd,
                                     CmdSource source, unsigned long now) {
  DeadbandTest_c::Params p;
  int rest_ms = (int)p.rest_ms, run_ms = (int)p.run_ms;
  if(sscanf(cmd, "test deadband %c %d %d %d %d %d %d %d", &p.wheel, &p.dir, &p.pwm_min,
            &p.pwm_max, &p.pwm_step, &p.repeats, &rest_ms, &run_ms) < 1) {
    reply(server, source, "#E usage: test deadband <L|R|S> [dir min max step repeats rest_ms run_ms]\n");
    return;
  }
  p.rest_ms = rest_ms < 0 ? 0 : rest_ms;
  p.run_ms = run_ms < 0 ? 0 : run_ms;
  int32_t count = testCount(p.wheel, robot.getLeftEncoderCount(), robot.getRightEncoderCount());
  const char *problem = deadband.start(p, now, count);
  if(problem != nullptr) {
    reply(server, source, "#E test deadband: %s\n", problem);
    return;
  }

  beginMotorTest(robot, source, DEADBAND_TEST);
  reply(server, source, "#BH deadband wheel=%c dir=%d min=%d max=%d step=%d repeats=%d "
        "rest_ms=%lu run_ms=%lu still_ms=%lu trials=%d\n",
        p.wheel, p.dir, p.pwm_min, p.pwm_max, p.pwm_step, p.repeats, p.rest_ms, p.run_ms,
        DeadbandTest_c::STILL_MS, p.repeats * deadband.levelsPerSweep());
  reply(server, source, "#BC trial,repeat,phase,wheel,pwm,t_ms,enc_us,enc_ok,enc_l,enc_r,rest_ok\n");
}

void Controller_c::startKeepTurnTest(Robot_c &robot, RobotWifiAP_c &server, const char *cmd,
                                     CmdSource source, unsigned long now) {
  KeepTurnTest_c::Params p;
  int step_ms = (int)p.step_ms, spin_ms = (int)p.spin_ms, rest_ms = (int)p.rest_ms;
  if(sscanf(cmd, "test keepturn %c %d %d %d %d %d %d", &p.wheel, &p.dir, &p.start_pwm,
            &step_ms, &p.repeats, &spin_ms, &rest_ms) < 1) {
    reply(server, source, "#E usage: test keepturn <L|R|S> [dir start step_ms repeats spin_ms rest_ms]\n");
    return;
  }
  p.step_ms = step_ms < 0 ? 0 : step_ms;
  p.spin_ms = spin_ms < 0 ? 0 : spin_ms;
  p.rest_ms = rest_ms < 0 ? 0 : rest_ms;
  int32_t count = testCount(p.wheel, robot.getLeftEncoderCount(), robot.getRightEncoderCount());
  const char *problem = keepturn.start(p, now, count);
  if(problem != nullptr) {
    reply(server, source, "#E test keepturn: %s\n", problem);
    return;
  }

  beginMotorTest(robot, source, KEEPTURN_TEST);
  reply(server, source, "#BH keepturn wheel=%c dir=%d start=%d step_ms=%lu repeats=%d "
        "spin_ms=%lu rest_ms=%lu still_ms=%lu trials=%d\n",
        p.wheel, p.dir, p.start_pwm, p.step_ms, p.repeats, p.spin_ms, p.rest_ms,
        KeepTurnTest_c::STILL_MS, p.repeats);
  reply(server, source, "#BC trial,phase,wheel,pwm,t_ms,enc_us,enc_ok,enc_l,enc_r\n");
}

void Controller_c::startGainTest(Robot_c &robot, RobotWifiAP_c &server, const char *cmd,
                                 CmdSource source, unsigned long now) {
  GainTest_c::Params p;
  int settle_ms = (int)p.settle_ms, hold_ms = (int)p.hold_ms, rest_ms = (int)p.rest_ms;
  if(sscanf(cmd, "test gain %c %d %d %d %d %d %d %d %d", &p.wheel, &p.dir, &p.pwm_min,
            &p.pwm_max, &p.pwm_step, &p.repeats, &settle_ms, &hold_ms, &rest_ms) < 1) {
    reply(server, source, "#E usage: test gain <L|R|S> [dir min max step repeats settle_ms hold_ms rest_ms]\n");
    return;
  }
  p.settle_ms = settle_ms < 0 ? 0 : settle_ms;
  p.hold_ms = hold_ms < 0 ? 0 : hold_ms;
  p.rest_ms = rest_ms < 0 ? 0 : rest_ms;
  int32_t count = testCount(p.wheel, robot.getLeftEncoderCount(), robot.getRightEncoderCount());
  const char *problem = gain.start(p, now, count);
  if(problem != nullptr) {
    reply(server, source, "#E test gain: %s\n", problem);
    return;
  }

  beginMotorTest(robot, source, GAIN_TEST);
  reply(server, source, "#BH gain wheel=%c dir=%d min=%d max=%d step=%d repeats=%d settle_ms=%lu "
        "hold_ms=%lu rest_ms=%lu still_ms=%lu trials=%d\n",
        p.wheel, p.dir, p.pwm_min, p.pwm_max, p.pwm_step, p.repeats, p.settle_ms, p.hold_ms,
        p.rest_ms, GainTest_c::STILL_MS, p.repeats);
  reply(server, source, "#BC trial,leg,phase,wheel,pwm,t_ms,enc_us,enc_ok,enc_l,enc_r\n");
}

// S (spin) turns the robot on the spot: right wheel opposite to the left.
// Its count changes whenever either wheel moves.
int32_t Controller_c::testCount(char wheel, int32_t left, int32_t right) {
  if(wheel == 'L') return left;
  if(wheel == 'R') return right;
  return left - right;
}

void Controller_c::setTestPWM(Robot_c &robot, char wheel, int pwm) {
  if(wheel == 'S') {
    robot.setMotorPWM(pwm, -pwm);
  } else {
    robot.setMotorPWM(wheel == 'L' ? pwm : 0, wheel == 'R' ? pwm : 0);
  }
}

void Controller_c::beginMotorTest(Robot_c &robot, CmdSource source, TestKind kind) {
  setSignal(0);
  clearManual();
  robot.setMotorPWM(0, 0);
  usr_mode = MOTOR_TEST;
  test_kind = kind;
  test_source = source;
  test_rows = 0;
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

  long enc_l = robot.getLeftEncoderCount();
  long enc_r = robot.getRightEncoderCount();

  if(test_kind == GAIN_TEST) {
    const GainTest_c::Params &p = gain.getParams();
    gain.update(now, testCount(p.wheel, enc_l, enc_r));
    if(!gain.active()) {
      endMotorTest(robot, server, now, "done");
      return;
    }
    int pwm = gain.getPwm();
    setTestPWM(robot, p.wheel, pwm);
    reply(server, test_source, "#B,%d,%s,%s,%c,%d,%lu,%lu,%d,%ld,%ld\n",
          gain.getTrial(), gain.getLegName(), gain.getPhaseName(), p.wheel, pwm, now, enc_us,
          enc_ok ? 1 : 0, enc_l, enc_r);
    test_rows++;
    return;
  }

  if(test_kind == KEEPTURN_TEST) {
    const KeepTurnTest_c::Params &p = keepturn.getParams();
    keepturn.update(now, testCount(p.wheel, enc_l, enc_r));
    if(!keepturn.active()) {
      endMotorTest(robot, server, now, "done");
      return;
    }
    int pwm = keepturn.getPwm();
    setTestPWM(robot, p.wheel, pwm);
    reply(server, test_source, "#B,%d,%s,%c,%d,%lu,%lu,%d,%ld,%ld\n",
          keepturn.getTrial(), keepturn.getPhaseName(), p.wheel, pwm, now, enc_us,
          enc_ok ? 1 : 0, enc_l, enc_r);
    test_rows++;
    return;
  }

  const DeadbandTest_c::Params &p = deadband.getParams();
  deadband.update(now, testCount(p.wheel, enc_l, enc_r));
  if(!deadband.active()) {
    endMotorTest(robot, server, now, "done");
    return;
  }

  int pwm = deadband.getPwm();
  setTestPWM(robot, p.wheel, pwm);
  bool running = deadband.getPhase() == DeadbandTest_c::RUN;
  reply(server, test_source, "#B,%d,%d,%s,%c,%d,%lu,%lu,%d,%ld,%ld,%d\n",
        deadband.getTrial(), deadband.getRepeat(), running ? "run" : "rest", p.wheel, pwm,
        now, enc_us, enc_ok ? 1 : 0, enc_l, enc_r, running && deadband.getRestOk() ? 1 : 0);
  test_rows++;
}

void Controller_c::endMotorTest(Robot_c &robot, RobotWifiAP_c &server, unsigned long now,
                                const char *why) {
  const char *name = "deadband";
  int trials = deadband.getTrial();
  if(test_kind == KEEPTURN_TEST) {
    keepturn.abort();
    name = "keepturn";
    trials = keepturn.getTrial();
  } else if(test_kind == GAIN_TEST) {
    gain.abort();
    name = "gain";
    trials = gain.getTrial();
  } else if(deadband.active()) {
    deadband.abort();
  }
  robot.setMotorPWM(0, 0);
  reply(server, test_source, "#BE %s %s rows=%lu trials=%d\n", name, why, test_rows, trials);
  usr_mode = MANUAL_CTRL;
  manual_vel = false;
  clearManual();
  manual_source = test_source;
  manual_cmd_ms = now;
  last_manual_ms = now;
}

void Controller_c::handleConfigCommand(RobotWifiAP_c &server, const char *cmd, CmdSource source) {
  char key[40];
  float value;

  if(strcmp(cmd, "id") == 0) {
    reply(server, source, "#ID %s\n", robot_id);
  }
  else if(strncmp(cmd, "name ", 5) == 0) {
    const char *name = cmd + 5;
    if(!RobotConfig_c::validName(name)) {
      reply(server, source, "#E name: 1 to %d letters, digits, - or _\n", RobotConfig_c::MAX_NAME_LEN);
    } else if(!RobotConfig_c::saveName(name)) {
      reply(server, source, "#E name: couldn't write to flash\n");
    } else {
      reply(server, source, "#I name saved: %s (reboot to use it)\n", name);
    }
  }
  else if(strcmp(cmd, "get") == 0) {
    for(int i = 0; i < config.size(); i++) {
      reply(server, source, "#C %s %g\n", config.entry(i).key, *config.entry(i).value);
    }
  }
  else if(sscanf(cmd, "get %39s", key) == 1) {
    const RobotConfig_c::Entry *e = config.find(key);
    if(e == nullptr) {
      reply(server, source, "#E get: unknown key %s\n", key);
    } else {
      reply(server, source, "#C %s %g\n", e->key, *e->value);
    }
  }
  else if(sscanf(cmd, "set %39s %f", key, &value) == 2) {
    RobotConfig_c::SetResult result = config.set(key, value);
    if(result == RobotConfig_c::SET_UNKNOWN_KEY) {
      reply(server, source, "#E set: unknown key %s\n", key);
    } else if(result == RobotConfig_c::SET_OUT_OF_RANGE) {
      const RobotConfig_c::Entry *e = config.find(key);
      reply(server, source, "#E set: %s must be %g to %g\n", key, e->min, e->max);
    } else {
      applyConfig();
      reply(server, source, "#C %s %g\n", key, *config.find(key)->value);
    }
  }
  else if(strcmp(cmd, "save") == 0) {
    bool changed = false;
    if(!config.save(changed)) {
      reply(server, source, "#E save: couldn't write to flash\n");
    } else {
      reply(server, source, "#I config %s\n", changed ? "saved" : "unchanged, nothing written");
    }
  }
  else if(strcmp(cmd, "config reset") == 0) {
    config.resetToDefaults();
    applyConfig();
    reply(server, source, "#I config reset to defaults (send save to keep it)\n");
  }
  else {
    reply(server, source, "#E usage: set <key> <value> | get [key] | save | config reset | id | name <new>\n");
  }
}

void Controller_c::runLineFollower(Robot_c &robot, unsigned long now) {
  if (config.values.line_mode >= 0.5f) {
    runOffsetFollower(robot, now);
    return;
  }
  bool line_detected = lineDetected(robot);

  if (!line_detected && mode == FOLLOWING_LINE) {
    mode = SEARCHING_FOR_LINE;
    recovery_start_ms = now;
  }

  if (mode == SEARCHING_FOR_LINE) {
    if (line_detected) {
      mode = FOLLOWING_LINE;
    } else if (now - recovery_start_ms >= (unsigned long)config.values.recovery_timeout_ms) {
      mode = STOPPED;
    }
  }

  if (mode == STOPPED) {
    robot.setMotorPWM(0, 0);
    return;
  }

  if (mode == SEARCHING_FOR_LINE) {
    robot.setMotorPWM(config.values.recovery_left_pwm, config.values.recovery_right_pwm);
    return;
  }

  float error = (float)robot.surface.reading[1] - (float)robot.surface.reading[3];
  float turn = error * config.values.line_follow_gain;
  float base = config.values.base_pwm;
  robot.setMotorPWM(base - turn, base + turn);
}

void Controller_c::runOffsetFollower(Robot_c &robot, unsigned long now) {
  const RobotConfigValues_t &c = config.values;
  float dt_s = last_line_ms == 0 ? 0.0f : (now - last_line_ms) / 1000.0f;
  last_line_ms = now;

  if (line.detected && mode == SEARCHING_FOR_LINE) {
    mode = FOLLOWING_LINE;
    last_offset_mm = line.offset_mm;
  } else if (!line.detected && mode == FOLLOWING_LINE) {
    mode = SEARCHING_FOR_LINE;
    recovery_start_ms = now;
  } else if (mode == SEARCHING_FOR_LINE && now - recovery_start_ms >= (unsigned long)c.recovery_timeout_ms) {
    mode = STOPPED;
  }
  // The speed controllers would read the wheels as stuck and drive them harder.
  if (mode == STOPPED || now - last_enc_ok_ms > ENCODER_LOSS_MS) {
    mode = STOPPED;
    left_speed.reset();
    right_speed.reset();
    robot.setMotorPWM(0, 0);
    return;
  }

  float half_b = c.wheel_separation_mm / 2;
  float left_target, right_target;
  if (mode == SEARCHING_FOR_LINE) {
    // Spin towards the side the line was last seen on.
    float wheel = radians(c.line_search_deg_s) * half_b * (line.offset_mm >= 0.0f ? 1.0f : -1.0f);
    left_target = -wheel;
    right_target = wheel;
  } else {
    // Pure pursuit on the point the sensors see: the line's middle at (L, e)
    // ahead of the wheels is reached on an arc of curvature 2e / (L^2 + e^2).
    // The outer wheel is held at line_speed_mm_s, so the robot slows on curves.
    float e = line.offset_mm;
    float lookahead = c.line_lookahead_mm;
    float curvature = c.line_kp * 2.0f * e / (lookahead * lookahead + e * e);   // 1/mm, + anticlockwise
    float v = c.line_speed_mm_s / (1.0f + fabsf(curvature) * half_b);
    float omega = v * curvature;                                               // rad/s
    if (dt_s > 0.0f) {
      omega += radians(c.line_kd) * (e - last_offset_mm) / dt_s;
    }
    last_offset_mm = e;
    left_target = v - omega * half_b;
    right_target = v + omega * half_b;
  }
  left_speed.setTarget(left_target);
  right_speed.setTarget(right_target);
  // On the first cycle the setpoint ramp hasn't moved off 0 yet, so the
  // feedforward would push backwards for one cycle; wait for the next.
  if (dt_s <= 0.0f) {
    return;
  }
  robot.setMotorPWM(left_speed.update(dt_s), right_speed.update(dt_s));
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
       strncmp(cmd, "move ", 5) == 0 || strncmp(cmd, "turn ", 5) == 0 || strncmp(cmd, "cmd ", 4) == 0 || strncmp(cmd, "test ", 5) == 0 || strcmp(cmd, "start") == 0 ||
       strncmp(cmd, "set ", 4) == 0 || strcmp(cmd, "config reset") == 0) {
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
      twist.stop();
      left_speed.reset();
      right_speed.reset();
      float max_pwm = config.values.manual_max_pwm;
      manual_left_pwm = constrain(left, -max_pwm, max_pwm);
      manual_right_pwm = constrain(right, -max_pwm, max_pwm);
      manual_cmd_ms = now;
      manual_source = source;
    }
  }
  else if(sscanf(cmd, "vel %f %f", &left, &right) == 2) {
    if(usr_mode == MANUAL_CTRL && isfinite(left) && isfinite(right)) {
      manual_vel = true;
      mover.cancel();
      twist.stop();
      float max_speed = config.values.manual_max_speed_mm_s;
      left_speed.setTarget(constrain(left, -max_speed, max_speed));
      right_speed.setTarget(constrain(right, -max_speed, max_speed));
      manual_cmd_ms = now;
      manual_source = source;
    }
  }
  else if(strncmp(cmd, "cmd ", 4) == 0) {
    if(sscanf(cmd, "cmd %f %f", &left, &right) != 2) {
      reply(server, source, "#E usage: cmd <v mm/s> <omega deg/s>\n");
    }
    else if(usr_mode == MANUAL_CTRL && isfinite(left) && isfinite(right)) {
      manual_vel = true;
      mover.cancel();
      if(left == 0.0f && right == 0.0f && !twist.isActive()) {
        left_speed.setTarget(0.0f);
        right_speed.setTarget(0.0f);
      } else {
        float counts_per_rev = odometry.encoderCountsPerRevolution;
        float max_speed = config.values.manual_max_speed_mm_s;
        twist.set(constrain(left, -max_speed, max_speed), right, config.values.wheel_separation_mm,
                  robot.getLeftEncoderCount(), robot.getRightEncoderCount(),
                  TWO_PI * odometry.leftWheelRadiusMm / counts_per_rev,
                  TWO_PI * odometry.rightWheelRadiusMm / counts_per_rev);
      }
      manual_cmd_ms = now;
      manual_source = source;
    }
  }
  else if(strncmp(cmd, "move ", 5) == 0) {
    float speed = 0.0f;
    if(sscanf(cmd, "move %f %f", &left, &speed) < 1) {
      reply(server, source, "#E usage: move <mm> [speed]\n");
    }
    else if(usr_mode == MANUAL_CTRL && isfinite(left) && isfinite(speed)) {
      float max_mm = config.values.move_max_distance_mm;
      left = constrain(left, -max_mm, max_mm);
      startMove(robot, "move", left, left, speed, source, now);
    }
  }
  else if(strncmp(cmd, "turn ", 5) == 0) {
    float speed = 0.0f;
    if(sscanf(cmd, "turn %f %f", &left, &speed) < 1) {
      reply(server, source, "#E usage: turn <deg> [speed]\n");
    }
    else if(usr_mode == MANUAL_CTRL && isfinite(left) && isfinite(speed)) {
      // each wheel's arc: angle times half the wheel separation
      float arc_mm = constrain(left, -MAX_TURN_DEG, MAX_TURN_DEG) * DEG_TO_RAD
                     * config.values.wheel_separation_mm / 2.0f;
      startMove(robot, "turn", -arc_mm, arc_mm, speed, source, now);
    }
  }
  else if(strncmp(cmd, "test deadband", 13) == 0) {
    startDeadbandTest(robot, server, cmd, source, now);
  }
  else if(strncmp(cmd, "test keepturn", 13) == 0) {
    startKeepTurnTest(robot, server, cmd, source, now);
  }
  else if(strncmp(cmd, "test gain", 9) == 0) {
    startGainTest(robot, server, cmd, source, now);
  }
  else if(strncmp(cmd, "set ", 4) == 0 || strncmp(cmd, "get", 3) == 0 || strcmp(cmd, "save") == 0 ||
          strcmp(cmd, "config reset") == 0 || strcmp(cmd, "id") == 0 || strncmp(cmd, "name ", 5) == 0) {
    handleConfigCommand(server, cmd, source);
  }
  else if(sscanf(cmd, "ping %lu", &ping_id) == 1) {
    // Answered in the same cycle, so the round trip includes the wait for the
    // next 10 ms update (as every other command does) but nothing else.
    reply(server, source, "#P %lu %lu\n", ping_id, now);
  }
  else if(strcmp(cmd, "line") == 0) {
    reply(server, source, "#L offset_mm=%.2f residual=%.3f detected=%d norm=%.2f %.2f %.2f %.2f %.2f us=%lu max_us=%lu\n",
          line.offset_mm, line.residual, line.detected ? 1 : 0,
          line.norm[0], line.norm[1], line.norm[2], line.norm[3], line.norm[4],
          line.update_us, line.max_update_us);
  }
  else {
    reply(server, source, "#E unknown command: %s\n", cmd);
  }
}
