#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "Robot.h"
#include "RobotWifiAP.h"
#include "TaskTimer.h"
#include "OdometryModel.h"
#include "WheelSpeedController.h"
#include "MoveController.h"

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

    /** @brief Fastest interval between controller and I2C acquisition cycles. */
    static const unsigned long UPDATE_INTERVAL_MS = 10;
    /** @brief Interval between telemetry records in milliseconds. */
    static const unsigned long TELEMETRY_INTERVAL_MS = 10;
    /** @brief Maximum line-search duration before the controller stops. */
    static const unsigned long RECOVERY_TIMEOUT_MS = 5000;
    /** @brief Surface-reading threshold used to decide whether a line is present. */
    static const uint16_t LINE_THRESHOLD = 1400;
    /** @brief Nominal forward PWM bias for line following. */
    static constexpr float BASE_PWM = 45.0f;
    /** @brief Proportional gain applied to the DN2-minus-DN4 surface error. */
    static constexpr float LINE_FOLLOW_GAIN = 0.01f;
    /** @brief Left PWM used during the one-direction line-search rotation. */
    static constexpr float RECOVERY_LEFT_PWM = -35.0f;
    /** @brief Right PWM used during the one-direction line-search rotation. */
    static constexpr float RECOVERY_RIGHT_PWM = 35.0f;
    /** @brief Largest PWM magnitude accepted by the "pwm" command in manual mode. */
    static constexpr float MANUAL_MAX_PWM = 100.0f;
    /** @brief Manual mode stops the motors if no "pwm" or "vel" command arrives for this
     * long, so a dropped WiFi link can't leave the robot driving (the TCP
     * connection can still look open for minutes after the link is gone).
     * tools/teleop.py resends the current "pwm" command well within this time.
     * Zero disables the check. */
    static const unsigned long MANUAL_TIMEOUT_MS = 500;
    /** @brief Largest wheel speed accepted by the "vel" command, in mm/s.
     * About 100 PWM gives 290-320 mm/s on the test robot, so this leaves the PI
     * some headroom. */
    static constexpr float MANUAL_MAX_SPEED_MM_S = 250.0f;
    /** @brief Wheel-speed PI gains, shared by both wheels. Chosen in simulation
     * to halve the slow-speed wobble of 0.1 / 1.0 with modest start overshoot;
     * tune on the robot with tools/speed_test.py (README, "Wheel speed control"). */
    static constexpr float SPEED_KP = 0.3f;
    static constexpr float SPEED_KI = 3.0f;
    /** @brief PWM that starts each wheel from rest, used to kick a stuck wheel.
     * Above the feedforward offsets because a motor needs more to start than to
     * keep turning (logs/run_20261011_172405.csv: left starts at 25-30, right
     * 22-25). Zero turns the kick off. */
    static constexpr float LEFT_BREAKAWAY_PWM = 30.0f;
    static constexpr float RIGHT_BREAKAWAY_PWM = 24.0f;
    /** @brief Lowest PWM while a wheel is meant to be turning: about what keeps
     * it turning once moving (logs/speedtest_v1.csv, on a stand: below ~13 left
     * and ~9 right the wheels stall). Stops the PI pulling a wheel that has just
     * lurched free back down until it sticks again. */
    static constexpr float LEFT_RUN_MIN_PWM = 13.0f;
    static constexpr float RIGHT_RUN_MIN_PWM = 9.0f;
    /** @brief Per-wheel feedforward, pwm = offset + speed / slope. Fitted to
     * logs/run_20261011_172405.csv at 35-100 PWM, so they belong to that robot;
     * refit for another one. */
    static constexpr float LEFT_FF_OFFSET_PWM = 17.0f;
    static constexpr float LEFT_FF_MM_S_PER_PWM = 3.5f;
    static constexpr float RIGHT_FF_OFFSET_PWM = 12.0f;
    static constexpr float RIGHT_FF_MM_S_PER_PWM = 3.75f;

    /** @brief "move" command: speed per mm still to go (1/s), and speed limits
     * in mm/s. MOVE_MIN_SPEED_MM_S keeps the wheels clear of the deadband. */
    static constexpr float MOVE_KP = 3.0f;
    static constexpr float MOVE_MAX_SPEED_MM_S = 40.0f;
    static constexpr float MOVE_MIN_SPEED_MM_S = 15.0f;
    /** @brief A wheel this close to its target (mm) has arrived; one count is 0.28 mm. */
    static constexpr float MOVE_TOLERANCE_MM = 0.2f;
    /** @brief Coasting allowance: wheels stop this many seconds of travel early.
     * Set from simulation; if moves overshoot on the robot, raise it, if they
     * stop short, lower it. */
    static constexpr float MOVE_STOP_LEAD_S = 0.07f;
    /** @brief Longest distance accepted by one "move" command, mm. */
    static constexpr float MOVE_MAX_DISTANCE_MM = 500.0f;

    /** @brief Per-wheel speed controllers used by the "vel" and "move" commands. */
    WheelSpeedController_c left_speed;
    WheelSpeedController_c right_speed;
    /** @brief Drives a set distance for the "move" command, through the speed controllers. */
    MoveController_c mover;

    /** @brief Creates a controller in its waiting state. */
    Controller_c();

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
      AUTO
    };

    uint8_t signal;
    Mode mode;
    UsrMode usr_mode;
    char cmd_buf[64];
    uint8_t cmd_len = 0;
    char mode_message[64] = "";  // last complete command, kept until the next one
    float manual_left_pwm = 0.0f;
    float manual_right_pwm = 0.0f;
    unsigned long manual_cmd_ms = 0;   // time of the last "pwm" or "vel" command
    bool manual_vel = false;           // true: "vel" speed control; false: raw "pwm"
    unsigned long last_manual_ms = 0;  // previous manual-mode cycle, for the PI's dt
    TaskTimer_c update_timer;
    TaskTimer_c telemetry_timer;
    unsigned long recovery_start_ms;

    /** @brief Returns whether cached DN3 is at or above LINE_THRESHOLD. */
    bool lineDetected(const Robot_c &robot) const;

    /** @brief Clears manual PWM and speed targets, without sending a motor command. */
    void clearManual();

    /** @brief Applies the nominal controller behaviour for the current mode. */
    void runLineFollower(Robot_c &robot, unsigned long now);

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

    /**
     * @brief Acts on the complete command in cmd_buf.
     * @note Commands: "auto" and "manual" switch mode and stop the motors;
     * "start" starts the line follower in auto mode; "stop" stops the motors
     * (and returns auto mode to waiting); "pwm <left> <right>" sets the motor
     * PWM in manual mode, clamped to MANUAL_MAX_PWM; "vel <left> <right>" sets
     * wheel speeds in mm/s in manual mode, clamped to MANUAL_MAX_SPEED_MM_S;
     * "move <mm>" drives that far straight (negative: backwards) in manual
     * mode, then stops; any other motion command or "stop" cancels it.
     * Unknown commands are reported on Serial and ignored.
     */
    void handleCommand(Robot_c &robot, unsigned long now);
};

#endif
