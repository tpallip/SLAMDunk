#ifndef TWIST_CONTROLLER_H
#define TWIST_CONTROLLER_H

#include <stdint.h>

/**
 * @brief Turns a body velocity command (v forward, omega anticlockwise) into
 * wheel speed targets for WheelSpeedController_c, for the "cmd" command.
 *
 * Wheel targets: v_left = v - omega * b/2, v_right = v + omega * b/2, with b
 * the wheel separation. On top of that a heading loop keeps the robot turning
 * at the commanded rate: it adds up the wheel distance difference the command
 * asks for, (v_right - v_left) * dt, compares it with the encoders' difference
 * since the command started, and moves the two wheel targets apart by
 * kp * error, half each way. A wheel that lags (a late start, a slower motor)
 * is then made up for instead of leaving a heading error. The error is held
 * within max_error_mm so a stuck wheel can't wind the loop up. If either
 * wheel target would exceed max_speed, both are scaled down together, which
 * keeps the path's curvature.
 *
 * With accel_mm_s2 set, the wheel targets move towards a new command at that
 * rate (the wheel that changes most at that rate, the other in proportion),
 * so the path's curvature is kept while speeding up and slowing down. A
 * command of 0 0 slows down this way with the heading loop still running,
 * until the faster wheel target is under stop_speed_mm_s.
 *
 * Then the wheels are stopped one at a time. Near the bottom of the ramp both
 * sit on their PWM floor, where the heading loop can no longer move them, and
 * each coasts its own distance once stopped, so stopping both at once turns
 * the robot. Instead the controller predicts where the wheel distance
 * difference will end up, each wheel coasting its measured speed times its
 * own stop lead (the left wheel coasts further), and stops at once any wheel whose remaining travel would add
 * to the heading error; the other keeps going at stop_speed_mm_s until the
 * predicted error is within trim_tolerance_mm, and stops then. A wheel that
 * stalls, or max_trim_s running out, stops it anyway.
 */
class TwistController_c {
  public:
    /** @brief Wheel speed correction per mm of heading error (as wheel distance difference), 1/s. */
    float kp = 4.0f;
    /** @brief Largest heading error kept, mm of wheel distance difference. */
    float max_error_mm = 20.0f;
    /** @brief Fastest wheel speed target, mm/s. */
    float max_speed_mm_s = 250.0f;
    /** @brief How fast the wheel targets follow a new command, mm/s^2; 0 jumps at once. */
    float accel_mm_s2 = 0.0f;
    /** @brief Slowing to 0 0 ends once the faster wheel target is below this, mm/s. */
    float stop_speed_mm_s = 40.0f;
    /** @brief Coasting allowance when stopping: the left wheel coasts its speed times this, s. */
    float left_stop_lead_s = 0.088f;
    /** @brief The same for the right wheel, s. */
    float right_stop_lead_s = 0.079f;
    /** @brief Stopping ends once the predicted heading error is within this, mm of wheel distance difference. */
    float trim_tolerance_mm = 0.3f;
    /** @brief Longest a wheel keeps going alone while stopping, s. */
    float max_trim_s = 0.3f;

    /**
     * @brief Sets the command. Starts the heading loop from the current counts
     * if it isn't running; a repeated command keeps it running. 0 0 slows
     * down to a stop if running, and does nothing otherwise.
     * @param v_mm_s Forward speed; @param omega_deg_s Turn rate, + anticlockwise.
     * @param separation_mm Wheel separation b.
     */
    void set(float v_mm_s, float omega_deg_s, float separation_mm, int32_t left_count, int32_t right_count,
             float left_mm_per_count, float right_mm_per_count);

    /** @brief Stops at once; isActive() becomes false. */
    void stop();

    /** @brief Returns whether a command is being followed, including slowing down to a stop. */
    bool isActive() const;

    /** @brief Returns whether it is slowing down after a 0 0 command. */
    bool isStopping() const;

    /**
     * @brief Computes this cycle's wheel speed targets, dt_s since the last call.
     * @param left_measured_mm_s, right_measured_mm_s Measured wheel speeds, used when stopping.
     */
    void update(int32_t left_count, int32_t right_count, float dt_s,
                float left_measured_mm_s, float right_measured_mm_s,
                float &left_speed_mm_s, float &right_speed_mm_s);

    /** @brief Current heading error, mm of wheel distance difference (+: turned too little anticlockwise). */
    float getError() const;

  private:
    bool active = false;
    float left_command_mm_s = 0.0f;  // from the command alone
    float right_command_mm_s = 0.0f;
    float left_target_mm_s = 0.0f;   // moving towards the command at accel_mm_s2
    float right_target_mm_s = 0.0f;
    int32_t left_start = 0;          // counts when the loop started
    int32_t right_start = 0;
    float left_mm_per_count = 1.0f;
    float right_mm_per_count = 1.0f;
    float wanted_diff_mm = 0.0f;     // commanded right - left distance so far
    float error_mm = 0.0f;
    bool trimming = false;           // stopping the wheels one at a time
    bool left_stopped = false;
    bool right_stopped = false;
    float left_direction = 1.0f;     // each wheel's direction of travel when trimming started
    float right_direction = 1.0f;
    float trim_s = 0.0f;

    void trim(float diff_mm, float dt_s, float left_measured_mm_s, float right_measured_mm_s,
              float &left_speed_mm_s, float &right_speed_mm_s);
};

#endif
