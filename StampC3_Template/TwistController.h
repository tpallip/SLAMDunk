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
 */
class TwistController_c {
  public:
    /** @brief Wheel speed correction per mm of heading error (as wheel distance difference), 1/s. */
    float kp = 4.0f;
    /** @brief Largest heading error kept, mm of wheel distance difference. */
    float max_error_mm = 20.0f;
    /** @brief Fastest wheel speed target, mm/s. */
    float max_speed_mm_s = 250.0f;

    /**
     * @brief Sets the command. Starts the heading loop from the current counts
     * if it isn't running; a repeated command keeps it running.
     * @param v_mm_s Forward speed; @param omega_deg_s Turn rate, + anticlockwise.
     * @param separation_mm Wheel separation b.
     */
    void set(float v_mm_s, float omega_deg_s, float separation_mm, int32_t left_count, int32_t right_count,
             float left_mm_per_count, float right_mm_per_count);

    /** @brief Stops; isActive() becomes false. */
    void stop();

    /** @brief Returns whether a command is being followed. */
    bool isActive() const;

    /** @brief Computes this cycle's wheel speed targets, dt_s since the last call. */
    void update(int32_t left_count, int32_t right_count, float dt_s,
                float &left_speed_mm_s, float &right_speed_mm_s);

    /** @brief Current heading error, mm of wheel distance difference (+: turned too little anticlockwise). */
    float getError() const;

  private:
    bool active = false;
    float left_target_mm_s = 0.0f;   // from the command alone
    float right_target_mm_s = 0.0f;
    int32_t left_start = 0;          // counts when the loop started
    int32_t right_start = 0;
    float left_mm_per_count = 1.0f;
    float right_mm_per_count = 1.0f;
    float wanted_diff_mm = 0.0f;     // commanded right - left distance so far
    float error_mm = 0.0f;
};

#endif
