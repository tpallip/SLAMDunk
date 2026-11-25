#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H

#include <Arduino.h>

/**
 * @brief Runtime-tunable parameters held in RAM.
 *
 * Every field can be changed over TCP with SET,<name>,<value> without
 * reflashing. The compiled defaults below are used at boot unless a set of
 * values was previously stored with SAVE. When adding a field, also add a
 * row to the descriptor table in RobotParams.cpp, a key in
 * config/robot_params.json, and (if range-checked there) tests/tcp_logger.py.
 */
struct RobotParams_t {
  // Timing
  uint32_t update_interval_ms     = 10;
  uint32_t telemetry_interval_ms  = 50;
  uint32_t log_interval_ms        = 10;
  uint32_t trial_sample_count     = 500;
  uint32_t recovery_timeout_ms    = 5000;
  // Line follower (operates on normalised 0..1 sensor values)
  float    line_threshold         = 0.5f;
  float    base_pwm               = 45.0f;
  float    line_follow_gain       = 20.0f;
  float    recovery_left_pwm      = -35.0f;
  float    recovery_right_pwm     = 35.0f;
  // Motor command limit applied to TCP MOTOR requests
  uint32_t motor_pwm_limit        = 100;
  // Geometry (odometry and motion helpers)
  float    left_wheel_radius_mm   = 16.0f;
  float    right_wheel_radius_mm  = 16.0f;
  float    wheel_separation_mm    = 90.0f;
  float    encoder_counts_per_rev = 358.3f;
  // Surface-sensor calibration: raw reading that maps to 0 (white) and 1
  // (black) for DN1..DN5. Defaults are the mean raw readings at 0% and 100%
  // blackness over calibration trials 1-6 (tests/calibrate_sensors.py).
  float    white_dn1              = 862.2f;
  float    white_dn2              = 746.2f;
  float    white_dn3              = 548.7f;
  float    white_dn4              = 629.8f;
  float    white_dn5              = 898.3f;
  float    black_dn1              = 2309.4f;
  float    black_dn2              = 2834.1f;
  float    black_dn3              = 2095.4f;
  float    black_dn4              = 2561.8f;
  float    black_dn5              = 2645.4f;
};

/** @brief Result of a SET request. */
enum ParamSetResult_t {
  PARAM_SET_OK,
  PARAM_SET_UNKNOWN,
  PARAM_SET_BAD_VALUE,
  PARAM_SET_OUT_OF_RANGE
};

/**
 * @brief Name-addressable access to RobotParams_t, with optional NVS storage.
 *
 * A single descriptor table drives set, get, listing, and validation so the
 * parameter list exists in one place on the firmware side.
 */
class RobotParamsStore_c {
  public:
    /** @brief Current values. Read freely; change only through set(). */
    RobotParams_t values;

    /** @brief Incremented on every successful change; sent in LOG records. */
    uint32_t version = 0;

    /** @brief Parses text and stores it into the named parameter. */
    ParamSetResult_t set(const char *name, const char *text);

    /**
     * @brief Formats the named parameter's value into buffer.
     * @return false if the name is unknown.
     */
    bool format(const char *name, char *buffer, size_t buffer_size) const;

    /** @brief Number of parameters in the descriptor table. */
    size_t count() const;

    /** @brief Name of the parameter at index, or nullptr when out of range. */
    const char *nameAt(size_t index) const;

    /** @brief Restores compiled defaults in RAM (does not touch flash). */
    void restoreDefaults();

    /** @brief Writes the current values to ESP32 flash (NVS). */
    bool saveToFlash() const;

    /**
     * @brief Loads values previously written by saveToFlash().
     * @return true when a stored set matching this firmware's layout was found.
     * @note A stored set from a firmware with a different parameter layout is
     * ignored, leaving the compiled defaults in place.
     */
    bool loadFromFlash();

    /** @brief Removes any stored values from flash. */
    bool clearFlash() const;
};

#endif
