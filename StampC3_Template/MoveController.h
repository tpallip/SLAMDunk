#ifndef MOVE_CONTROLLER_H
#define MOVE_CONTROLLER_H

#include <stdint.h>

/**
 * @brief Drives each wheel a set distance using the encoders: both the same
 * way for "move", opposite ways for "turn".
 *
 * Gives each wheel a target count, then on every cycle turns the remaining
 * distance into wheel speed targets for WheelSpeedController_c:
 * speed = kp * remaining, scaled down on both wheels together when either
 * would exceed max_speed (so they keep pace and the robot drives straight),
 * and at least min_speed so the wheels don't stall in the motor deadband near
 * the end. A wheel stops once it is within tolerance of its target or has
 * passed it, allowing for the distance it coasts after stopping (its speed
 * times stop_lead_s); the move ends when both have, or at a deadline.
 */
class MoveController_c {
  public:
    /** @brief Speed per mm still to go, in 1/s. */
    float kp;
    /** @brief Fastest wheel speed during a move, mm/s. */
    float max_speed_mm_s;
    /** @brief Slowest wheel speed while a wheel still has distance to go, mm/s. */
    float min_speed_mm_s;
    /** @brief A wheel this close to its target (mm) counts as arrived. */
    float tolerance_mm;
    /** @brief Coasting allowance: a wheel stops this many seconds of travel early. */
    float stop_lead_s;

    MoveController_c(float kp, float max_speed_mm_s, float min_speed_mm_s, float tolerance_mm,
                     float stop_lead_s);

    /**
     * @brief Starts a move: each wheel drives its own distance (negative: backwards).
     * @param speed_mm_s Fastest wheel speed for this move; 0 uses max_speed_mm_s.
     * @param left_count, right_count Current absolute encoder counts.
     * @param left_mm_per_count, right_mm_per_count Wheel travel per count.
     * @param now_ms Current time; the move gives up at a deadline that allows
     * twice the time at min_speed_mm_s, plus one second.
     */
    void start(float left_mm, float right_mm, float speed_mm_s, int32_t left_count, int32_t right_count,
               float left_mm_per_count, float right_mm_per_count, unsigned long now_ms);

    /** @brief Abandons the move; isActive() becomes false. */
    void cancel();

    /** @brief Returns whether a move is in progress. */
    bool isActive() const;

    /**
     * @brief Computes this cycle's wheel speed targets.
     * @return false once the move has ended (both speeds are then zero).
     */
    bool update(int32_t left_count, int32_t right_count, unsigned long now_ms,
                float &left_speed_mm_s, float &right_speed_mm_s);

    /** @brief Distance still to go when the move ended (mm; negative = overshoot). */
    float getLeftError() const;
    float getRightError() const;
    /** @brief Whether the last move ended at its deadline instead of arriving. */
    bool timedOut() const;

  private:
    bool active;
    bool timed_out;
    int32_t left_target;
    int32_t right_target;
    float left_mm_per_count;
    float right_mm_per_count;
    float left_direction;       // +1 forwards, -1 backwards
    float right_direction;
    float speed_mm_s;           // fastest wheel speed for this move
    bool left_done;
    bool right_done;
    float left_error_mm;
    float right_error_mm;
    float left_last_speed;      // last speed target, for the coasting allowance
    float right_last_speed;
    unsigned long deadline_ms;
};

#endif
