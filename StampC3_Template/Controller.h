#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Robot.h"
#include "RobotWifiAP.h"
#include "TaskTimer.h"
#include "OdometryModel.h"
#include "RobotParams.h"

/**
 * @brief Supplies the nominal line-following controller for the StampC3 example.
 *
 * Controller_c owns the controller state, while Robot_c owns the latest hardware
 * snapshot and RobotWifiAP_c owns optional telemetry transport. Students may read
 * or replace this controller, but should treat its constants as a demonstrator
 * baseline rather than a calibration of every physical robot.
 */
class Controller_c {
  
  public:
    /** @brief Editable local encoder-to-pose model; inspect odometry.pose.
     * @note Controller mode resets do not reset this estimate. Use
     * odometry.reset(...) explicitly to start a new physical trial.
     */
    OdometryModel_c odometry;

    /**
     * @brief Runtime-tunable parameters (timing, line follower, geometry).
     * @note Defaults are in RobotParams.h. Change them over TCP with SET; call
     * applyParams() after changing values directly in code.
     */
    RobotParamsStore_c params;

    /**
     * @brief Calibrated DN1..DN5 readings: 0 = white, 1 = black, clamped.
     * @note Updated after each successful surface read using the white_dn*
     * and black_dn* parameters; keeps its previous value after a failed read.
     * Raw readings remain available in robot.surface.reading.
     */
    float surface_norm[5];

    /** @brief Creates a controller in its waiting state. */
    Controller_c();

    /**
     * @brief Loads any parameters saved in flash and applies them.
     * @note Call once from setup() after robot.initialise().
     */
    void begin(Robot_c &robot);

    /**
     * @brief Pushes the current parameter values into timers, odometry, and
     * the middleware motion-helper geometry.
     * @note Re-baselines local odometry at its current pose, because counts
     * measured under the old geometry cannot be mixed with the new geometry.
     */
    void applyParams(Robot_c &robot);

    /** @brief Clears signal and mode state, returning the controller to waiting.
     * @note This does not reset the task timers or the local odometry estimate.
     * It sends no motor command: returning to waiting does not stop the motors.
     * Send robot.setMotorPWM(0, 0) separately and check the command result.
     */
    void reset();

    /**
     * @brief Returns whether the controller has received its start signal.
     * @return Zero while waiting; one after the button or caller starts it.
     */
    uint8_t getSignal() const;

    /**
     * @brief Starts the controller or resets it to waiting.
     * @param requested_signal Zero resets to waiting without sending a motor stop;
     * a non-zero value starts line following only if the current signal is zero.
     * @note The signal remains one after a line-search timeout. Reset to zero
     * before requesting another start; a repeated non-zero value has no effect.
     */
    void setSignal(uint8_t requested_signal);

    /**
     * @brief Runs one time-controlled acquisition, control, and telemetry cycle.
     * @param robot Hardware-facing interface and cached measurement snapshot.
     * @param server Optional WiFi/TCP telemetry transport.
     * @note Each accepted update cycle attempts to refresh sensors, encoders,
     * and the middleware pose. Failed reads retain their previous cached values;
     * local odometry is updated only after a successful encoder read. The method
     * returns early between update intervals. A complete Serial line containing
     * trial_no,percentage writes one sensor CSV record; no start-button press is
     * required. TCP telemetry and logging are opt-in. See tests/README.md for
     * the full TCP command set; MOTOR requests are limited to
     * +/- params.values.motor_pwm_limit.
     */
    void update(Robot_c &robot, RobotWifiAP_c &server);

  private:
    /** @brief Internal modes used by the nominal line-following demonstration. */
    enum Mode {
      WAITING,
      FOLLOWING_LINE,
      SEARCHING_FOR_LINE,
      STOPPED
    };

    uint8_t signal;
    Mode mode;
    TaskTimer_c update_timer;
    TaskTimer_c telemetry_timer;
    TaskTimer_c log_timer;
    unsigned long recovery_start_ms;
    /** @brief bit0 surface, bit1 encoders, bit2 pose: reads that succeeded this cycle. */
    uint8_t read_ok_mask;
    bool tcp_log_streaming;
    bool motion_helper_active;
    bool motion_seen_running;
    unsigned long motion_start_ms;
    bool self_test_active;
    bool self_test_seen_running;
    unsigned long self_test_start_ms;
    char sensor_test_input[64];
    uint8_t sensor_test_input_length;
    bool sensor_test_input_overflow;
    bool sensor_test_csv_header_printed;
    char tcp_command_buffer[96];
    uint8_t tcp_command_length;
    bool tcp_command_overflow;
    bool tcp_control_active;
    bool tcp_telemetry_streaming;
    bool tcp_trial_active;
    long tcp_trial_number;
    float tcp_trial_percentage;
    uint32_t tcp_trial_samples_collected;

    /** @brief Returns whether normalised DN3 is at or above line_threshold. */
    bool lineDetected(const Robot_c &robot) const;

    /** @brief Maps raw DN1..DN5 to 0 (white) .. 1 (black) into surface_norm. */
    void normaliseSurface(const Robot_c &robot);

    /** @brief Applies the nominal controller behaviour for the current mode. */
    void runLineFollower(Robot_c &robot, unsigned long now);
    /** @brief Reads trial,percentage commands and writes CSV sensor records. */
    void sensorTest(Robot_c &robot, bool surface_readings_updated);
    /** @brief Parses a non-negative trial number and percentage from one CSV line. */
    bool parseSensorTestInput(char *line, long &trial_number, float &percentage) const;
    /** @brief Processes newline-delimited commands from the TCP client. */
    void processTcpCommands(
      Robot_c &robot,
      RobotWifiAP_c &server,
      bool surface_readings_updated,
      unsigned long timestamp_ms
    );
    /** @brief Handles one validated line from the TCP client. */
    void handleTcpCommand(
      char *line,
      Robot_c &robot,
      RobotWifiAP_c &server,
      bool surface_readings_updated,
      unsigned long timestamp_ms
    );
    /** @brief Sends one fresh reading while a 500-sample TCP trial is active. */
    void recordTcpTrialSample(
      Robot_c &robot,
      RobotWifiAP_c &server,
      bool surface_readings_updated,
      unsigned long timestamp_ms
    );

    /**
     * @brief Sends the current controller and robot snapshot as one CSV record.
     * @note Controller.cpp sends a DATA-prefixed record with these 14 values,
     * in order: timestamp_ms, x, y, theta, left_pwm, right_pwm, left_count,
     * right_count, DN1, DN2, DN3, DN4, DN5, signal. Labels are not sent as a
     * header.
     * x and y are in mm and theta is in radians, from the local Euler estimate,
     * not robot.pose from I2C. PWM values are cached requests, not measured output.
     * The timestamp is the controller-cycle start time; it does not guarantee
     * that every cached reading was successfully refreshed in that cycle.
     */
    void publishTelemetry(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms);

    /**
     * @brief Sends one full core-state LOG record.
     * @note Fields, in order: timestamp_ms, read_ok_mask, DN1..DN5, left_count,
     * right_count, left_pwm, right_pwm, odom_x, odom_y, odom_theta, mw_x, mw_y,
     * mw_theta, mode, signal, motion_state, cfg_version, DN1..DN5 normalised.
     */
    void publishLog(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms);

    /** @brief Handles SET/GET/PARAMS/SAVE/DEFAULTS. Returns false if not a param command. */
    bool handleParamCommand(char *line, Robot_c &robot, RobotWifiAP_c &server);

    /** @brief Handles MOVE/ROTATE/SELFTEST/ODOM_RESET. Returns false if not a motion command. */
    bool handleMotionCommand(char *line, Robot_c &robot, RobotWifiAP_c &server);

    /** @brief Sends wheel geometry to the middleware helpers if it differs. */
    void pushHelperGeometry(Robot_c &robot);

    /** @brief Reports helper-motion and self-test completion once each. */
    void pollMotionAndSelfTest(Robot_c &robot, RobotWifiAP_c &server, unsigned long now);

    /** @brief Sends a line built with snprintf; refuses rather than truncates. */
    bool sendFormatted(RobotWifiAP_c &server, const char *format, ...);
};

#endif
