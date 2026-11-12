#include "Controller.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

const unsigned long Controller_c::UPDATE_INTERVAL_MS;
const unsigned long Controller_c::TELEMETRY_INTERVAL_MS;
const unsigned long Controller_c::RECOVERY_TIMEOUT_MS;
const uint16_t Controller_c::LINE_THRESHOLD;
const uint16_t Controller_c::TRIAL_SAMPLE_COUNT;
constexpr float Controller_c::BASE_PWM;
constexpr float Controller_c::LINE_FOLLOW_GAIN;
constexpr float Controller_c::RECOVERY_LEFT_PWM;
constexpr float Controller_c::RECOVERY_RIGHT_PWM;

Controller_c::Controller_c()
  : update_timer(UPDATE_INTERVAL_MS),
    telemetry_timer(TELEMETRY_INTERVAL_MS),
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
  if (signal == 0) {
    if (robot.isButtonPressed()) {
      setSignal(1);
    }
  }

  // Sensor capture is driven by serial input and does not require the start button.
  sensorTest(robot, surface_readings_updated);
  processTcpCommands(robot, server, surface_readings_updated, now);
  recordTcpTrialSample(robot, server, surface_readings_updated, now);

  // TCP telemetry is sent only after the client explicitly starts it.
  if (tcp_telemetry_streaming && telemetry_timer.isReady(now)) {
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
      static_cast<unsigned int>(TRIAL_SAMPLE_COUNT)
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
      server.sendLine("ERROR,expected_MOTOR_left_right_PWM_-100_to_100");
      return;
    }

    while (isspace(static_cast<unsigned char>(*end))) end++;
    if (*end != ',') {
      server.sendLine("ERROR,expected_MOTOR_left_right_PWM_-100_to_100");
      return;
    }

    cursor = end + 1;
    char *right_start = cursor;
    errno = 0;
    long right_pwm = strtol(cursor, &end, 10);
    bool has_right_pwm = end != right_start;
    while (isspace(static_cast<unsigned char>(*end))) end++;
    if (!has_right_pwm || errno == ERANGE || *end != '\0'
        || left_pwm < -100 || left_pwm > 100
        || right_pwm < -100 || right_pwm > 100) {
      server.sendLine("ERROR,expected_MOTOR_left_right_PWM_-100_to_100");
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
  if (tcp_trial_samples_collected == TRIAL_SAMPLE_COUNT) {
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
