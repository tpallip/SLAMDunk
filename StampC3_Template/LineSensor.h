#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdint.h>

/**
 * @brief Turns the five surface readings into where the line is: each
 * sensor normalised to 0 on white and 1 on black, and the line's sideways
 * offset in mm.
 *
 * Normalised reading: n = (raw - white) / (black - white), held to 0..1,
 * with each sensor's own white and black levels (TODO 3.2), so one sensor
 * reading 1400 doesn't mean the same as another.
 *
 * Offset: the sensors sit on a 45 mm circle at 45, 14, 0, -14 and -45 deg
 * (DN1 left to DN5 right), so sideways at y = r sin(angle) = 31.8, 10.9, 0,
 * -10.9, -31.8 mm. A line of width w whose middle is at e (mm, + to the
 * robot's left) seen through a Gaussian footprint of width sigma gives
 *
 *     n(e) = Phi((y - e + w/2) / sigma) - Phi((y - e - w/2) / sigma)
 *
 * (Phi the normal CDF; sigma per sensor from TODO 3.4). The offset is the e
 * whose predicted readings match the five normalised ones best (least
 * squares), searched every 2 mm over +-50 mm, then every 0.5 mm within 2 mm
 * of the best, and refined with a parabola through the best point and its
 * neighbours (the fit is smooth on the 2 mm scale: the footprints are
 * 2.3-3.6 mm and the line 16.5 mm wide). That stays right across the
 * whole range, where a weighted average of the readings flattens out once
 * the line is past DN2 or DN4. configure() tabulates each sensor's n(e) on
 * the search grid, as integers (n times 1000), so the search is integer
 * sums: the ESP32-C3 has no FPU, and the same search in float took 2.7 ms.
 *
 * Where only DN1 or DN5 sees the line, a reading fits the line on either
 * side of that sensor about equally; a weak pull towards the previous offset
 * picks the side the line came from.
 *
 * The line counts as seen when any sensor reads at least detect_level. The
 * offset is measured along the sensor circle's sideways axis, so it is the
 * line's offset at about 40 mm ahead of the wheels, not at the wheels.
 */
class LineSensor_c {
  public:
    static const int N = 5;

    /** @brief Raw reading on white and on black, per sensor (DN1..DN5). */
    float white[N] = {700, 700, 700, 700, 700};
    float black[N] = {2200, 2200, 2200, 2200, 2200};
    /** @brief Gaussian footprint width per sensor, mm. */
    float sigma_mm[N] = {3.5f, 3.5f, 3.5f, 3.5f, 3.5f};
    /** @brief Width of the line, mm. */
    float line_width_mm = 18.0f;
    /** @brief The line is seen when any normalised reading reaches this. */
    float detect_level = 0.5f;

    /** @brief Latest normalised readings, 0 white to 1 black. */
    float norm[N] = {0, 0, 0, 0, 0};
    /** @brief Line's offset, mm, + to the robot's left. Kept from the last time the line was seen. */
    float offset_mm = 0.0f;
    /** @brief rms difference between the readings and the best fit (0..1); large on a corner or junction. */
    float residual = 0.0f;
    /** @brief Whether the line was seen in the latest update. */
    bool detected = false;
    /** @brief Time the latest update took, and the longest since configure(), us. */
    unsigned long update_us = 0;
    unsigned long max_update_us = 0;

    /** @brief Rebuilds the response tables. Call after changing sigma_mm or line_width_mm. */
    void configure();

    /** @brief Normalises one snapshot of readings and finds the line's offset. */
    void update(const uint16_t reading[N]);

  private:
    static constexpr float RADIUS_MM = 45.0f;
    static constexpr float MAX_OFFSET_MM = 50.0f;
    static constexpr float STEP_MM = 0.5f;
    static const int COARSE = 4;               // coarse search every 4 table steps (2 mm)
    static const int OFFSETS = 201;           // -50 to 50 mm in 0.5 mm steps
    static const int32_t SCALE = 1000;        // readings as integers, n * SCALE
    // Pull towards the previous offset: 1e-5 per mm^2 in squared readings,
    // which is 1e-5 * SCALE^2 * STEP_MM^2 = 2.5 per grid step^2, kept as 5/2.
    static const int32_t CONTINUITY_X2 = 5;

    int16_t predicted[N][OFFSETS];             // n * SCALE with the line's middle at offset j
    int32_t norm_q[N];

    int32_t cost(int j, int previous_j) const;
};

#endif
