#include "Controller.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

namespace {

/** If a helper never reports RUNNING within this time, report its state anyway. */
const unsigned long HELPER_START_GRACE_MS = 500;

const char *motionStateName(uint8_t state) {
  switch (state) {
    case MOTION_STATUS_IDLE: return "IDLE";
    case MOTION_STATUS_RUNNING: return "RUNNING";
    case MOTION_STATUS_COMPLETE: return "COMPLETE";
    case MOTION_STATUS_TIMEOUT: return "TIMEOUT";
    case MOTION_STATUS_CANCELLED: return "CANCELLED";
    default: return "UNKNOWN";
  }
}

const char *selfTestStateName(uint8_t state) {
  switch (state) {
    case SELF_TEST_STATUS_IDLE: return "IDLE";
    case SELF_TEST_STATUS_RUNNING: return "RUNNING";
    case SELF_TEST_STATUS_COMPLETE: return "COMPLETE";
    case SELF_TEST_STATUS_CANCELLED: return "CANCELLED";
    default: return "UNKNOWN";
  }
}

/** Parses a finite float that must be followed only by whitespace or a delimiter. */
bool parseFloatField(char *&cursor, float &value, char delimiter) {
  while (isspace(static_cast<unsigned char>(*cursor))) cursor++;
  errno = 0;
  char *end = nullptr;
  float parsed = strtof(cursor, &end);
  if (end == cursor || errno == ERANGE || !isfinite(parsed)) return false;
  while (isspace(static_cast<unsigned char>(*end))) end++;
  if (*end != delimiter) return false;
  value = parsed;
  cursor = (*end == '\0') ? end : end + 1;
  return true;
}

}  // namespace

Controller_c::Controller_c()
  : surface_norm{},
    update_timer(params.values.update_interval_ms),
    telemetry_timer(params.values.telemetry_interval_ms),
    log_timer(params.values.log_interval_ms),
    read_ok_mask(0),
    tcp_log_streaming(false),
    motion_helper_active(false),
    motion_seen_running(false),
    motion_start_ms(0),
    self_test_active(false),
    self_test_seen_running(false),
    self_test_start_ms(0),
    sensor_test_input_length(0),
    sensor_test_input_overflow(false),
    sensor_test_csv_header_printed(false),
    tcp_command_length(0),
    tcp_command_overflow(false),
    tcp_control_active(false),
    tcp_telemetry_streaming(false),
    tcp_trial_active(false),
    tcp_trial_number(0),
    tcp_trial_percentage(0.0f),
    tcp_trial_samples_collected(0) {
  reset();
}

void Controller_c::reset() {
  signal = 0;
  mode = WAITING;
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

void Controller_c::begin(Robot_c &robot) {
  if (params.loadFromFlash()) {
    Serial.println("# Parameters loaded from flash.");
  } else {
    Serial.println("# Using compiled default parameters.");
  }
  applyParams(robot);
}

void Controller_c::applyParams(Robot_c &robot) {
  const RobotParams_t &p = params.values;

  update_timer.setIntervalMS(p.update_interval_ms);
  telemetry_timer.setIntervalMS(p.telemetry_interval_ms);
  log_timer.setIntervalMS(p.log_interval_ms);

  bool geometry_changed =
    odometry.leftWheelRadiusMm != p.left_wheel_radius_mm
    || odometry.rightWheelRadiusMm != p.right_wheel_radius_mm
    || odometry.wheelSeparationMm != p.wheel_separation_mm
    || odometry.encoderCountsPerRevolution != p.encoder_counts_per_rev;

  if (geometry_changed) {
    odometry.leftWheelRadiusMm = p.left_wheel_radius_mm;
    odometry.rightWheelRadiusMm = p.right_wheel_radius_mm;
    odometry.wheelSeparationMm = p.wheel_separation_mm;
    odometry.encoderCountsPerRevolution = p.encoder_counts_per_rev;
    // Keep the current pose but take a new count baseline under the new geometry.
    OdometryModel_c::Pose pose = odometry.pose;
    odometry.reset(pose.x, pose.y, pose.theta);
  }

  pushHelperGeometry(robot);
}

void Controller_c::pushHelperGeometry(Robot_c &robot) {
  // The middleware only accepts geometry while no helper is running; it is
  // pushed again when a helper finishes.
  if (motion_helper_active || robot.isMotionHelperRunning()) return;
  const RobotParams_t &p = params.values;
  if (robot.motion_geometry.left_wheel_radius_mm == p.left_wheel_radius_mm
      && robot.motion_geometry.right_wheel_radius_mm == p.right_wheel_radius_mm
      && robot.motion_geometry.wheel_separation_mm == p.wheel_separation_mm) {
    return;
  }
  if (!robot.setMotionHelperGeometry(
        p.left_wheel_radius_mm, p.right_wheel_radius_mm, p.wheel_separation_mm)
      || !robot.getMotionHelperGeometry()) {
    Serial.println("# WARNING: motion-helper geometry was not acknowledged");
  }
}

bool Controller_c::sendFormatted(RobotWifiAP_c &server, const char *format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  int length = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  if (length < 0 || static_cast<size_t>(length) >= sizeof(buffer)) {
    Serial.println("# ERROR: TCP line too long; not sent");
    return false;
  }
  return server.sendLine(buffer);
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

  // Ask the robot for latest information on surface reflectance sensors.
  bool surface_readings_updated = robot.getSurfaceSensors();
  if (surface_readings_updated) {
    normaliseSurface(robot);
  }

  // Ask the robot for the latest encoder information, and then use this
  // to update the odometry on board the StampC3 (see OdometryModel.h).
  bool encoders_updated = robot.getEncoders();
  if (encoders_updated) {
    odometry.update(robot.getLeftEncoderCount(), robot.getRightEncoderCount());
  }

  // Pose estimation from the Pololu 3Pi robot itself.
  bool pose_updated = robot.getPose();

  read_ok_mask = (surface_readings_updated ? 0x01 : 0)
                 | (encoders_updated ? 0x02 : 0)
                 | (pose_updated ? 0x04 : 0);

  // While waiting, a button press sets signal to 1 and starts the physical
  // robot's line follower, even without Processing connected. The transmitted
  // signal also starts an armed Digital Twin live comparison. This is a start
  // control, not a start/stop toggle; further presses are ignored while signal is 1.
  if (signal == 0) {
    if (robot.isButtonPressed()) {
      setSignal(1);
    }
  }

  // Sensor capture is driven by serial input and does not require the start button.
  sensorTest(robot, surface_readings_updated);
  processTcpCommands(robot, server, surface_readings_updated, now);
  recordTcpTrialSample(robot, server, surface_readings_updated, now);
  pollMotionAndSelfTest(robot, server, now);

  // TCP telemetry is sent only after the client explicitly starts it.
  if (tcp_telemetry_streaming && telemetry_timer.isReady(now)) {
    telemetry_timer.resetTimer(now);
    publishTelemetry(robot, server, now);
  }

  // The full-state log is likewise opt-in (LOG START).
  if (tcp_log_streaming && log_timer.isReady(now)) {
    log_timer.resetTimer(now);
    publishLog(robot, server, now);
  }
}

bool Controller_c::lineDetected(const Robot_c &robot) const {
  (void)robot;
  // Note: only using the central sensor to detect a line.
  return surface_norm[2] >= params.values.line_threshold;
}

void Controller_c::normaliseSurface(const Robot_c &robot) {
  const RobotParams_t &p = params.values;
  const float white[5] = {p.white_dn1, p.white_dn2, p.white_dn3, p.white_dn4, p.white_dn5};
  const float black[5] = {p.black_dn1, p.black_dn2, p.black_dn3, p.black_dn4, p.black_dn5};
  for (int i = 0; i < 5; i++) {
    float span = black[i] - white[i];
    if (span <= 0.0f) {
      // Invalid calibration (black not above white): report 0 rather than garbage.
      surface_norm[i] = 0.0f;
      continue;
    }
    float value = ((float)robot.surface.reading[i] - white[i]) / span;
    surface_norm[i] = constrain(value, 0.0f, 1.0f);
  }
}

bool Controller_c::parseSensorTestInput(
  char *line,
  long &trial_number,
  float &percentage
) const {
  char *cursor = line;
  while (isspace(static_cast<unsigned char>(*cursor))) cursor++;

  errno = 0;
  char *end = nullptr;
  long parsed_trial_number = strtol(cursor, &end, 10);
  if (end == cursor || errno == ERANGE || parsed_trial_number < 0) {
    return false;
  }

  cursor = end;
  while (isspace(static_cast<unsigned char>(*cursor))) cursor++;
  if (*cursor != ',') return false;
  cursor++;
  while (isspace(static_cast<unsigned char>(*cursor))) cursor++;

  errno = 0;
  float parsed_percentage = strtof(cursor, &end);
  if (end == cursor || errno == ERANGE || !isfinite(parsed_percentage)) {
    return false;
  }

  cursor = end;
  while (isspace(static_cast<unsigned char>(*cursor))) cursor++;
  if (*cursor != '\0' || parsed_percentage < 0.0f || parsed_percentage > 100.0f) {
    return false;
  }

  trial_number = parsed_trial_number;
  percentage = parsed_percentage;
  return true;
}

void Controller_c::sensorTest(Robot_c &robot, bool surface_readings_updated) {
  while (Serial.available() > 0) {
    char character = static_cast<char>(Serial.read());
    if (character != '\r' && character != '\n') {
      if (sensor_test_input_length < sizeof(sensor_test_input) - 1) {
        sensor_test_input[sensor_test_input_length++] = character;
      } else {
        sensor_test_input_overflow = true;
      }
      continue;
    }

    if (sensor_test_input_length == 0 && !sensor_test_input_overflow) {
      continue;
    }

    sensor_test_input[sensor_test_input_length] = '\0';
    long trial_number = 0;
    float percentage = 0.0f;
    if (sensor_test_input_overflow
        || !parseSensorTestInput(sensor_test_input, trial_number, percentage)) {
      Serial.println("# ERROR: enter trial_no,percentage (percentage 0-100)");
    } else if (!surface_readings_updated) {
      Serial.println("# ERROR: surface sensor read failed; record not written");
    } else {
      if (!sensor_test_csv_header_printed) {
        Serial.println("trial no,percentage,reading0,,reading1,reading2,reading3,reading4");
        sensor_test_csv_header_printed = true;
      }

      // Stop the motors when a measurement is requested.
      if (!robot.setMotorPWM(0, 0)) {
        Serial.println("# WARNING: motor stop request failed");
      } else {
        tcp_control_active = false;
      }
      Serial.printf(
        "%ld,%.2f,%u,,%u,%u,%u,%u\n",
        trial_number,
        static_cast<double>(percentage),
        static_cast<unsigned int>(robot.surface.reading[0]),
        static_cast<unsigned int>(robot.surface.reading[1]),
        static_cast<unsigned int>(robot.surface.reading[2]),
        static_cast<unsigned int>(robot.surface.reading[3]),
        static_cast<unsigned int>(robot.surface.reading[4])
      );
    }

    sensor_test_input_length = 0;
    sensor_test_input_overflow = false;
  }
}

void Controller_c::processTcpCommands(
  Robot_c &robot,
  RobotWifiAP_c &server,
  bool surface_readings_updated,
  unsigned long timestamp_ms
) {
  if (!server.clientConnected()) {
    tcp_command_length = 0;
    tcp_command_overflow = false;
    tcp_telemetry_streaming = false;
    tcp_log_streaming = false;
    tcp_trial_active = false;
    if (tcp_control_active) {
      if (!robot.setMotorPWM(0, 0)) {
        Serial.println("# ERROR: motor stop failed after TCP disconnect");
      }
      tcp_control_active = false;
    }
  }

  while (server.available() > 0) {
    int received = server.read();
    if (received < 0) break;

    char character = static_cast<char>(received);
    if (character != '\r' && character != '\n') {
      if (tcp_command_length < sizeof(tcp_command_buffer) - 1) {
        tcp_command_buffer[tcp_command_length++] = character;
      } else {
        tcp_command_overflow = true;
      }
      continue;
    }

    if (tcp_command_length == 0 && !tcp_command_overflow) {
      continue;
    }

    tcp_command_buffer[tcp_command_length] = '\0';
    if (tcp_command_overflow) {
      server.sendLine("ERROR,command_too_long");
    } else {
      handleTcpCommand(
        tcp_command_buffer,
        robot,
        server,
        surface_readings_updated,
        timestamp_ms
      );
    }
    tcp_command_length = 0;
    tcp_command_overflow = false;
  }
}

void Controller_c::handleTcpCommand(
  char *line,
  Robot_c &robot,
  RobotWifiAP_c &server,
  bool surface_readings_updated,
  unsigned long timestamp_ms
) {
  if (handleParamCommand(line, robot, server)) return;
  if (handleMotionCommand(line, robot, server)) return;

  if (strcmp(line, "LOG START") == 0) {
    tcp_log_streaming = true;
    log_timer.resetTimer(0);  // makes the first record go out this cycle
    server.sendLine("ACK,LOG,STARTED");
    return;
  }

  if (strcmp(line, "LOG STOP") == 0) {
    tcp_log_streaming = false;
    server.sendLine("ACK,LOG,STOPPED");
    return;
  }

  if (strcmp(line, "STATUS") == 0) {
    publishTelemetry(robot, server, timestamp_ms);
    return;
  }

  if (strcmp(line, "START TELEMETRY") == 0) {
    tcp_telemetry_streaming = true;
    server.sendLine("ACK,TELEMETRY,STARTED");
    return;
  }

  if (strcmp(line, "STOP TELEMETRY") == 0) {
    tcp_telemetry_streaming = false;
    server.sendLine("ACK,TELEMETRY,STOPPED");
    return;
  }

  if (strcmp(line, "STOP") == 0) {
    if (!robot.setMotorPWM(0, 0)) {
      server.sendLine("ERROR,motor_stop_failed");
      return;
    }
    tcp_control_active = false;
    server.sendLine("ACK,STOP");
    return;
  }

  if (strncmp(line, "TRIAL,", 6) == 0) {
    long trial_number = 0;
    float percentage = 0.0f;
    if (!parseSensorTestInput(line + 6, trial_number, percentage)) {
      server.sendLine("ERROR,expected_TRIAL_trial_no_percentage_0_to_100");
      return;
    }
    if (tcp_trial_active) {
      server.sendLine("ERROR,trial_in_progress");
      return;
    }
    if (!robot.setMotorPWM(0, 0)) {
      server.sendLine("ERROR,motor_stop_failed");
      return;
    }
    tcp_control_active = false;
    tcp_trial_number = trial_number;
    tcp_trial_percentage = percentage;
    tcp_trial_samples_collected = 0;
    tcp_trial_active = true;
    if (!server.printlnf(
      "TRIAL_STARTED,%ld,%.2f,%u",
      tcp_trial_number,
      static_cast<double>(tcp_trial_percentage),
      static_cast<unsigned int>(params.values.trial_sample_count)
    )) {
      tcp_trial_active = false;
      Serial.println("# ERROR: TCP trial start response could not be sent");
    }
    return;
  }

  if (strncmp(line, "SAMPLE,", 7) == 0) {
    long trial_number = 0;
    float percentage = 0.0f;
    if (!parseSensorTestInput(line + 7, trial_number, percentage)) {
      server.sendLine("ERROR,expected_SAMPLE_trial_no_percentage_0_to_100");
      return;
    }
    if (!surface_readings_updated) {
      server.sendLine("ERROR,surface_sensor_read_failed");
      return;
    }
    if (!robot.setMotorPWM(0, 0)) {
      server.sendLine("ERROR,motor_stop_failed");
      return;
    }
    tcp_control_active = false;
    if (!server.printlnf(
      "SAMPLE,%ld,%.2f,%u,,%u,%u,%u,%u",
      trial_number,
      static_cast<double>(percentage),
      static_cast<unsigned int>(robot.surface.reading[0]),
      static_cast<unsigned int>(robot.surface.reading[1]),
      static_cast<unsigned int>(robot.surface.reading[2]),
      static_cast<unsigned int>(robot.surface.reading[3]),
      static_cast<unsigned int>(robot.surface.reading[4])
    )) {
      Serial.println("# ERROR: TCP sample record could not be sent");
    }
    return;
  }

  if (strncmp(line, "MOTOR,", 6) == 0) {
    char *cursor = line + 6;
    errno = 0;
    char *end = nullptr;
    long left_pwm = strtol(cursor, &end, 10);
    if (end == cursor || errno == ERANGE) {
      server.sendLine("ERROR,expected_MOTOR_left_right_PWM");
      return;
    }

    while (isspace(static_cast<unsigned char>(*end))) end++;
    if (*end != ',') {
      server.sendLine("ERROR,expected_MOTOR_left_right_PWM");
      return;
    }

    cursor = end + 1;
    char *right_start = cursor;
    errno = 0;
    long right_pwm = strtol(cursor, &end, 10);
    bool has_right_pwm = end != right_start;
    while (isspace(static_cast<unsigned char>(*end))) end++;
    const long limit = static_cast<long>(params.values.motor_pwm_limit);
    if (!has_right_pwm || errno == ERANGE || *end != '\0'
        || left_pwm < -limit || left_pwm > limit
        || right_pwm < -limit || right_pwm > limit) {
      sendFormatted(server, "ERROR,expected_MOTOR_left_right_PWM_within_%ld", limit);
      return;
    }

    if (!robot.setMotorPWM(static_cast<float>(left_pwm), static_cast<float>(right_pwm))) {
      server.sendLine("ERROR,motor_command_failed");
      return;
    }
    tcp_control_active = left_pwm != 0 || right_pwm != 0;
    server.printlnf("ACK,MOTOR,%ld,%ld", left_pwm, right_pwm);
    return;
  }

  server.sendLine("ERROR,unknown_command");
}

void Controller_c::recordTcpTrialSample(
  Robot_c &robot,
  RobotWifiAP_c &server,
  bool surface_readings_updated,
  unsigned long timestamp_ms
) {
  if (!tcp_trial_active || !surface_readings_updated) {
    return;
  }

  if (!server.printlnf(
    "TRIAL_SAMPLE,%lu,%ld,%.2f,%u,%u,%u,%u,%u",
    timestamp_ms,
    tcp_trial_number,
    static_cast<double>(tcp_trial_percentage),
    static_cast<unsigned int>(robot.surface.reading[0]),
    static_cast<unsigned int>(robot.surface.reading[1]),
    static_cast<unsigned int>(robot.surface.reading[2]),
    static_cast<unsigned int>(robot.surface.reading[3]),
    static_cast<unsigned int>(robot.surface.reading[4])
  )) {
    tcp_trial_active = false;
    Serial.println("# ERROR: TCP trial sample could not be sent");
    return;
  }

  tcp_trial_samples_collected++;
  // >= so that lowering trial_sample_count mid-trial still ends the trial.
  if (tcp_trial_samples_collected >= params.values.trial_sample_count) {
    tcp_trial_active = false;
    if (!server.printlnf(
      "TRIAL_DONE,%ld,%u",
      tcp_trial_number,
      static_cast<unsigned int>(tcp_trial_samples_collected)
    )) {
      Serial.println("# ERROR: TCP trial completion response could not be sent");
    }
  }
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
    } else if (now - recovery_start_ms >= params.values.recovery_timeout_ms) {
      mode = STOPPED;
    }
  }

  if (mode == STOPPED) {
    robot.setMotorPWM(0, 0);
    return;
  }

  const RobotParams_t &p = params.values;
  if (mode == SEARCHING_FOR_LINE) {
    robot.setMotorPWM(p.recovery_left_pwm, p.recovery_right_pwm);
    return;
  }

  // Normalised DN2 - DN4 lies in -1..1, so line_follow_gain is PWM per unit error.
  float error = surface_norm[1] - surface_norm[3];
  float turn = error * p.line_follow_gain;
  robot.setMotorPWM(p.base_pwm - turn, p.base_pwm + turn);
}

bool Controller_c::handleParamCommand(char *line, Robot_c &robot, RobotWifiAP_c &server) {
  char value_text[32];

  if (strncmp(line, "SET,", 4) == 0) {
    char *name = line + 4;
    char *comma = strchr(name, ',');
    if (comma == nullptr) {
      server.sendLine("ERROR,expected_SET_name_value");
      return true;
    }
    *comma = '\0';
    switch (params.set(name, comma + 1)) {
      case PARAM_SET_OK:
        applyParams(robot);
        params.format(name, value_text, sizeof(value_text));
        sendFormatted(server, "ACK,SET,%s,%s", name, value_text);
        break;
      case PARAM_SET_UNKNOWN:
        sendFormatted(server, "ERROR,unknown_param,%s", name);
        break;
      case PARAM_SET_OUT_OF_RANGE:
        sendFormatted(server, "ERROR,out_of_range,%s", name);
        break;
      default:
        sendFormatted(server, "ERROR,bad_value,%s", name);
        break;
    }
    return true;
  }

  if (strncmp(line, "GET,", 4) == 0) {
    const char *name = line + 4;
    if (!params.format(name, value_text, sizeof(value_text))) {
      sendFormatted(server, "ERROR,unknown_param,%s", name);
    } else {
      sendFormatted(server, "PARAM,%s,%s", name, value_text);
    }
    return true;
  }

  if (strcmp(line, "PARAMS") == 0) {
    for (size_t i = 0; i < params.count(); i++) {
      const char *name = params.nameAt(i);
      params.format(name, value_text, sizeof(value_text));
      sendFormatted(server, "PARAM,%s,%s", name, value_text);
    }
    sendFormatted(server, "PARAMS_END,%lu", static_cast<unsigned long>(params.version));
    return true;
  }

  if (strcmp(line, "SAVE") == 0) {
    server.sendLine(params.saveToFlash() ? "ACK,SAVE" : "ERROR,save_failed");
    return true;
  }

  if (strcmp(line, "DEFAULTS") == 0) {
    params.restoreDefaults();
    applyParams(robot);
    server.sendLine(params.clearFlash() ? "ACK,DEFAULTS" : "ERROR,flash_clear_failed");
    return true;
  }

  return false;
}

bool Controller_c::handleMotionCommand(char *line, Robot_c &robot, RobotWifiAP_c &server) {
  unsigned long now = robot.getMillis();

  if (strncmp(line, "MOVE,", 5) == 0 || strncmp(line, "ROTATE,", 7) == 0) {
    bool is_move = line[0] == 'M';
    char *cursor = line + (is_move ? 5 : 7);
    float amount = 0.0f;
    if (!parseFloatField(cursor, amount, '\0')) {
      server.sendLine(is_move ? "ERROR,expected_MOVE_mm" : "ERROR,expected_ROTATE_rad");
      return true;
    }
    bool started = is_move ? robot.startMoveDistance(amount) : robot.startRotateAngle(amount);
    if (!started) {
      server.sendLine(is_move ? "ERROR,move_rejected" : "ERROR,rotate_rejected");
      return true;
    }
    // Helper owns the motors; a TCP disconnect should still stop the robot.
    tcp_control_active = true;
    motion_helper_active = true;
    motion_seen_running = false;
    motion_start_ms = now;
    sendFormatted(server, "ACK,%s,%.4f", is_move ? "MOVE" : "ROTATE", static_cast<double>(amount));
    return true;
  }

  if (strcmp(line, "SELFTEST") == 0) {
    if (!robot.startSelfTest()) {
      server.sendLine("ERROR,selftest_rejected");
      return true;
    }
    tcp_control_active = true;
    self_test_active = true;
    self_test_seen_running = false;
    self_test_start_ms = now;
    server.sendLine("ACK,SELFTEST");
    return true;
  }

  if (strcmp(line, "ODOM_RESET") == 0) {
    odometry.reset();
    server.sendLine("ACK,ODOM_RESET,0,0,0");
    return true;
  }

  if (strncmp(line, "ODOM_RESET,", 11) == 0) {
    char *cursor = line + 11;
    float x = 0.0f, y = 0.0f, theta = 0.0f;
    if (!parseFloatField(cursor, x, ',') || !parseFloatField(cursor, y, ',')
        || !parseFloatField(cursor, theta, '\0')) {
      server.sendLine("ERROR,expected_ODOM_RESET_x_y_theta");
      return true;
    }
    odometry.reset(x, y, theta);
    sendFormatted(server, "ACK,ODOM_RESET,%.2f,%.2f,%.5f",
                  static_cast<double>(x), static_cast<double>(y), static_cast<double>(theta));
    return true;
  }

  return false;
}

void Controller_c::pollMotionAndSelfTest(Robot_c &robot, RobotWifiAP_c &server, unsigned long now) {
  if (motion_helper_active && robot.getMotionStatus()) {
    uint8_t state = robot.motion_status.state;
    if (state == MOTION_STATUS_RUNNING) {
      motion_seen_running = true;
    } else if (motion_seen_running || now - motion_start_ms >= HELPER_START_GRACE_MS) {
      motion_helper_active = false;
      // A cancelled helper was replaced by a newer command that owns the
      // disconnect-stop flag; only clear it when the helper ended by itself.
      if (state != MOTION_STATUS_CANCELLED) tcp_control_active = false;
      sendFormatted(server, "MOTION_DONE,%s,%lu",
                    motionStateName(state),
                    static_cast<unsigned long>(robot.motion_status.elapsed_ms));
      // Geometry writes are skipped while a helper runs; catch up now.
      pushHelperGeometry(robot);
    }
  }

  if (self_test_active && robot.getSelfTestResult()) {
    const RobotSelfTestResult_t &r = robot.self_test;
    if (r.state == SELF_TEST_STATUS_RUNNING) {
      self_test_seen_running = true;
    } else if (self_test_seen_running || now - self_test_start_ms >= HELPER_START_GRACE_MS) {
      self_test_active = false;
      if (r.state != SELF_TEST_STATUS_CANCELLED) tcp_control_active = false;
      sendFormatted(server,
                    "SELFTEST_DONE,%s,%u,%u,%u,%ld,%ld,%u,%u,%u,%u,%u",
                    selfTestStateName(r.state),
                    static_cast<unsigned int>(r.motor_pass_mask),
                    static_cast<unsigned int>(r.encoder_pass_mask),
                    static_cast<unsigned int>(r.surface_pass_mask),
                    static_cast<long>(r.encoder_delta[0]),
                    static_cast<long>(r.encoder_delta[1]),
                    static_cast<unsigned int>(r.surface_range[0]),
                    static_cast<unsigned int>(r.surface_range[1]),
                    static_cast<unsigned int>(r.surface_range[2]),
                    static_cast<unsigned int>(r.surface_range[3]),
                    static_cast<unsigned int>(r.surface_range[4]));
    }
  }
}

void Controller_c::publishLog(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms) {
  if (!server.clientConnected()) return;
  if (!sendFormatted(
        server,
        "LOG,%lu,%u,%u,%u,%u,%u,%u,%ld,%ld,%d,%d,%.2f,%.2f,%.5f,%.2f,%.2f,%.5f,%u,%u,%u,%lu,"
        "%.3f,%.3f,%.3f,%.3f,%.3f",
        timestamp_ms,
        static_cast<unsigned int>(read_ok_mask),
        static_cast<unsigned int>(robot.surface.reading[0]),
        static_cast<unsigned int>(robot.surface.reading[1]),
        static_cast<unsigned int>(robot.surface.reading[2]),
        static_cast<unsigned int>(robot.surface.reading[3]),
        static_cast<unsigned int>(robot.surface.reading[4]),
        static_cast<long>(robot.getLeftEncoderCount()),
        static_cast<long>(robot.getRightEncoderCount()),
        static_cast<int>(robot.getLeftMotorPWM()),
        static_cast<int>(robot.getRightMotorPWM()),
        static_cast<double>(odometry.pose.x),
        static_cast<double>(odometry.pose.y),
        static_cast<double>(odometry.pose.theta),
        static_cast<double>(robot.pose.x),
        static_cast<double>(robot.pose.y),
        static_cast<double>(robot.pose.theta),
        static_cast<unsigned int>(mode),
        static_cast<unsigned int>(signal),
        static_cast<unsigned int>(robot.motion_status.state),
        static_cast<unsigned long>(params.version),
        static_cast<double>(surface_norm[0]),
        static_cast<double>(surface_norm[1]),
        static_cast<double>(surface_norm[2]),
        static_cast<double>(surface_norm[3]),
        static_cast<double>(surface_norm[4]))) {
    Serial.println("# ERROR: TCP log record could not be sent");
  }
}

void Controller_c::publishTelemetry(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms) {
  if (server.clientConnected()
      && !server.printf(
        "DATA,%lu,%.2f,%.2f,%.5f,%d,%d,%ld,%ld,%u,%u,%u,%u,%u,%u\n",
        timestamp_ms,
        odometry.pose.x,
        odometry.pose.y,
        odometry.pose.theta,
        robot.getLeftMotorPWM(),
        robot.getRightMotorPWM(),
        (long)robot.getLeftEncoderCount(),
        (long)robot.getRightEncoderCount(),
        static_cast<unsigned int>(robot.surface.reading[0]),
        static_cast<unsigned int>(robot.surface.reading[1]),
        static_cast<unsigned int>(robot.surface.reading[2]),
        static_cast<unsigned int>(robot.surface.reading[3]),
        static_cast<unsigned int>(robot.surface.reading[4]),
        static_cast<unsigned int>(signal)
      )) {
    Serial.println("# ERROR: TCP telemetry record could not be sent");
  }
}
