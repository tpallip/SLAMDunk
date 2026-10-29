#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Robot.h"
#include "RobotWifiAP.h"
#include "TaskTimer.h"
#include "OdometryModel.h"
#include "WheelSpeedController.h"
#include "MoveController.h"
#include "TwistController.h"
#include "LineSensor.h"
#include "DeadbandTest.h"
#include "KeepTurnTest.h"
#include "GainTest.h"
#include "RobotConfig.h"

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

    /** @brief Settings changed with "set <key> <value>"; see RobotConfig.h for defaults. */
    RobotConfig_c config;

    /** @brief Per-wheel speed controllers used by the "vel" and "move" commands. */
    WheelSpeedController_c left_speed;
    WheelSpeedController_c right_speed;
    /** @brief Drives set wheel distances for "move" and "turn", through the speed controllers. */
    MoveController_c mover;
    const char *move_name = "move";    // "move" or "turn", for the done/timed out reply
    /** @brief Body velocity for the "cmd" command, through the speed controllers. */
    TwistController_c twist;
    /** @brief Normalised surface readings and the line's offset, updated every cycle; "line" reports them. */
    LineSensor_c line;
    /** @brief "test deadband" command. */
    DeadbandTest_c deadband;
    /** @brief "test keepturn" command. */
    KeepTurnTest_c keepturn;
    /** @brief "test gain" command. */
    GainTest_c gain;

    /** @brief Creates a controller in its waiting state. */
    Controller_c();

    /** @brief Loads the saved config from flash and applies it. Call from setup().
     * @param id This robot's name, sent back by the "id" command. */
    void begin(const char *id);

    /** @brief Pushes config.values into the timers, odometry and controllers. */
    void applyConfig();

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
     * returns early between update intervals.
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

    enum UsrMode {
      MANUAL_CTRL,
      AUTO,
      MOTOR_TEST
    };

    /** @brief Where a command came from; replies go back the same way. */
    enum CmdSource {
      FROM_WIFI,
      FROM_SERIAL
    };

    uint8_t signal;
    Mode mode;
    UsrMode usr_mode;
    char cmd_buf[64];
    uint8_t cmd_len = 0;
    char serial_buf[64];         // commands typed over USB, kept apart from WiFi's
    uint8_t serial_len = 0;
    char mode_message[64] = "";  // last complete command, kept until the next one
    CmdSource manual_source = FROM_WIFI;  // source of the last manual-mode command
    float manual_left_pwm = 0.0f;
    float manual_right_pwm = 0.0f;
    unsigned long manual_cmd_ms = 0;   // time of the last "pwm" or "vel" command
    bool manual_vel = false;           // true: "vel" speed control; false: raw "pwm"
    unsigned long last_manual_ms = 0;  // previous manual-mode cycle, for the PI's dt
    enum TestKind { DEADBAND_TEST, KEEPTURN_TEST, GAIN_TEST };
    TestKind test_kind = DEADBAND_TEST;
    CmdSource test_source = FROM_WIFI;
    unsigned long test_rows = 0;
    const char *robot_id = "unknown";
    unsigned long enc_us = 0;          // micros() at the last encoder read
    bool enc_ok = false;
    unsigned long last_enc_ok_ms = 0;  // time of the last good encoder read
    // With no good encoder read for this long, "vel" and "move" stop: the
    // speed controller would read the wheels as stuck and drive them harder.
    static const unsigned long ENCODER_LOSS_MS = 50;
    static constexpr float MAX_TURN_DEG = 3600.0f;
    TaskTimer_c update_timer;
    TaskTimer_c telemetry_timer;
    unsigned long recovery_start_ms;
    unsigned long last_line_ms = 0;    // previous line-follower cycle, for the speed PI's dt
    float last_offset_mm = 0.0f;       // previous line offset, for line_kd

    /** @brief Returns whether cached DN3 is at or above config line_threshold. */
    bool lineDetected(const Robot_c &robot) const;

    /** @brief Clears manual PWM and speed targets, without sending a motor command. */
    void clearManual();

    /** @brief Starts a "move" or "turn" with these wheel distances. */
    void startMove(Robot_c &robot, const char *name, float left_mm, float right_mm, float speed_mm_s,
                   CmdSource source, unsigned long now);

    void startDeadbandTest(Robot_c &robot, RobotWifiAP_c &server, const char *cmd,
                           CmdSource source, unsigned long now);

    void startKeepTurnTest(Robot_c &robot, RobotWifiAP_c &server, const char *cmd,
                           CmdSource source, unsigned long now);

    void startGainTest(Robot_c &robot, RobotWifiAP_c &server, const char *cmd,
                       CmdSource source, unsigned long now);

    int32_t testCount(char wheel, int32_t left, int32_t right);

    void setTestPWM(Robot_c &robot, char wheel, int pwm);

    void beginMotorTest(Robot_c &robot, CmdSource source, TestKind kind);

    void runMotorTest(Robot_c &robot, RobotWifiAP_c &server, unsigned long now);

    void handleConfigCommand(RobotWifiAP_c &server, const char *cmd, CmdSource source);

    void endMotorTest(Robot_c &robot, RobotWifiAP_c &server, unsigned long now, const char *why);

    /** @brief Applies the nominal controller behaviour for the current mode. */
    void runLineFollower(Robot_c &robot, unsigned long now);
    /** @brief line_mode 1: steers on the line sensor's offset through the wheel speed controllers. */
    void runOffsetFollower(Robot_c &robot, unsigned long now);

    /**
     * @brief Sends the current controller and robot snapshot as one CSV record.
     * @note Controller.cpp writes these 14 columns, in order: timestamp_ms,
     * x, y, theta, left_pwm, right_pwm, left_count, right_count,
     * DN1, DN2, DN3, DN4, DN5, signal. These labels are not sent as a header.
     * x and y are in mm and theta is in radians, from the local Euler estimate,
     * not robot.pose from I2C. PWM values are cached requests, not measured output.
     * The timestamp is the controller-cycle start time; it does not guarantee
     * that every cached reading was successfully refreshed in that cycle.
     */
    void publishTelemetry(Robot_c &robot, RobotWifiAP_c &server, unsigned long timestamp_ms);
    int readClient(RobotWifiAP_c &client);

    /** @brief Collects a command line from USB Serial into serial_buf.
     * @return The line's length once a newline arrives, otherwise 0. */
    int readSerial();

    /** @brief Sends a "#"-prefixed reply line over WiFi or USB Serial.
     * @param to The link the command being answered came from.
     * @note Lines starting with "#" are not telemetry; tools/logger.py passes
     * them on to tools/motor_tui.py and doesn't count them as bad records. */
    void reply(RobotWifiAP_c &server, CmdSource to, const char *format, ...);

    /**
     * @brief Acts on one complete command line.
     * @param cmd The command, null-terminated.
     * @param source Where it came from; replies and the manual-mode link check
     * use it.
     * @note Commands: "auto" and "manual" switch mode and stop the motors;
     * "start" starts the line follower in auto mode; "stop" stops the motors
     * (and returns auto mode to waiting); "pwm <left> <right>" sets the motor
     * PWM in manual mode, clamped to manual_max_pwm; "vel <left> <right>" sets
     * wheel speeds in mm/s in manual mode, clamped to manual_max_speed_mm_s;
     * "move <mm> [speed]" drives that far straight (negative: backwards) in
     * manual mode, then stops; "turn <deg> [speed]" turns on the spot
     * (positive: anticlockwise) by the encoders and wheel_separation_mm. The
     * speed (mm/s, per wheel) defaults to move_max_speed_mm_s. Any other
     * motion command or "stop" cancels either. "cmd <v> <omega>" drives at v
     * mm/s forward while turning at omega deg/s (positive: anticlockwise),
     * held like "vel" (resend within manual_timeout_ms); "cmd 0 0" stops.
     * "ping <id>" replies "#P <id> <t_ms>" at once, for measuring the round
     * trip. "test deadband <L|R|S> [dir min max step repeats rest_ms run_ms]"
     * runs the deadband test, and "test keepturn <L|R|S> [dir start step_ms
     * repeats spin_ms rest_ms]" the keep-turning test, and "test gain <L|R|S>
     * [dir min max step repeats settle_ms hold_ms rest_ms]" the speed vs PWM
     * test. S spins the robot on the spot (right wheel opposite to the
     * left) instead of one wheel; "stop" or the button ends any test.
     * "id" replies "#ID <name>". "set <key> <value>" changes a setting
     * until reboot and "get [key]" reads them back, both as "#C <key>
     * <value>"; "save" keeps the current settings over a reboot and "config
     * reset" goes back to the firmware defaults. Unknown commands get
     * "#E unknown command: ..." and are ignored. "name <new>" saves the
     * robot's name, used for its WiFi network from the next boot.
     */
    void handleCommand(Robot_c &robot, RobotWifiAP_c &server, const char *cmd,
                       CmdSource source, unsigned long now);
};

#endif
