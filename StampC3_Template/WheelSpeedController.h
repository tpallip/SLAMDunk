#ifndef WHEEL_SPEED_CONTROLLER_H
#define WHEEL_SPEED_CONTROLLER_H

#include <stdint.h>

/**
 * @brief Holds one wheel at a target speed using its encoder: feedforward plus PI,
 * with stiction compensation.
 *
 * The feedforward turns the target speed straight into a PWM from a measured
 * straight-line fit, pwm = offset + speed / mm_s_per_pwm, so each motor gets
 * about the right PWM at once despite its own deadband and strength. The PI
 * term then removes what the fit misses (battery level, load).
 *
 * Near the deadband a motor sticks: it needs much more PWM to start moving
 * (breakaway, and it varies) than to keep moving. Two guards:
 * - Kick: when the wheel should be turning but no encoder count has arrived
 *   for longer than the target speed allows, the output starts at
 *   breakaway_pwm and rises by kick_ramp_pwm_per_s until the wheel moves, so
 *   it can't stay stuck below what this particular start needs. The integral
 *   is held meanwhile.
 * - Floor: while the target is non-zero, the output doesn't drop below
 *   run_min_pwm (about what keeps the wheel turning), so the PI can't pull a
 *   wheel that has just lurched free back down until it stalls again. The
 *   integral doesn't wind down while the output sits on the floor.
 *
 * Speed is measured over a window long enough for about min_counts counts at
 * the previous estimate (between 30 ms and window_ms), and capped at one count
 * over the time since the last count, so a stall shows up within a few cycles.
 * Choosing the window from the previous estimate, not from the counts in it,
 * keeps the estimate unbiased.
 *
 * One instance per wheel; Controller_c owns two.
 */
class WheelSpeedController_c {
  public:
    /** @brief Proportional gain, in PWM per mm/s of speed error. */
    float kp;
    /** @brief Integral gain, in PWM per mm of accumulated speed error. */
    float ki;
    /** @brief Feedforward PWM where the fitted line crosses zero speed. */
    float ff_offset_pwm;
    /** @brief Feedforward slope: mm/s gained per extra PWM above the offset. */
    float ff_mm_s_per_pwm;
    /** @brief Output limit; the integral stops growing while the output is held here. */
    float max_pwm;
    /** @brief PWM the kick starts from; 0 turns the kick off. */
    float breakaway_pwm;
    /** @brief Lowest output while the target is non-zero; 0 turns the floor off. */
    float run_min_pwm;
    /** @brief How fast the kick rises while the wheel stays stuck, PWM per second. */
    float kick_ramp_pwm_per_s = 200.0f;
    /** @brief Longest window for the speed estimate, ms. */
    unsigned long window_ms = 80;
    /** @brief The window aims to hold about this many counts at the previous speed. */
    float min_counts = 6.0f;
    /** @brief A wheel counts as stuck after max(this, 2.5 count periods at the target) with no count, ms. */
    unsigned long stall_min_ms = 30;

    WheelSpeedController_c(float kp, float ki, float ff_offset_pwm, float ff_mm_s_per_pwm,
                           float max_pwm, float breakaway_pwm, float run_min_pwm);

    /** @brief Sets the target in mm/s; zero stops the wheel and clears the integral. */
    void setTarget(float target_mm_s);

    /** @brief Returns the current target in mm/s. */
    float getTarget() const;

    /** @brief Returns the measured speed in mm/s. */
    float getSpeed() const;

    /** @brief Returns whether the last update() applied the breakaway kick. */
    bool isKicking() const;

    /** @brief Clears the target and integral. The speed estimate is kept. */
    void reset();

    /**
     * @brief Updates the speed estimate from one successful encoder read.
     * @param count Absolute encoder count for this wheel.
     * @param now_ms Time of the read.
     * @param mm_per_count Wheel travel per encoder count.
     * @note A failed read can simply be skipped: the next call measures over
     * the longer interval.
     */
    void measure(int32_t count, unsigned long now_ms, float mm_per_count);

    /**
     * @brief Returns the PWM for this control cycle.
     * @param dt_s Time since the previous call, in seconds.
     */
    float update(float dt_s);

  private:
    static const int HISTORY = 16;     // encoder samples kept; covers window_ms at 10 ms per read

    float target_mm_s;
    float speed_mm_s;
    float integral_mm;
    float mm_per_count;
    bool kicking;
    float kick_pwm;                    // current kick level while stuck
    int32_t counts[HISTORY];
    unsigned long times[HISTORY];
    int samples;                       // valid entries, up to HISTORY
    int newest;                        // index of the latest sample
    unsigned long last_change_ms;      // time of the latest sample whose count changed
};

#endif
