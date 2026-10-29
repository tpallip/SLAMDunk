#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

// Settings that can be changed without reflashing, with "set <key> <value>".
// The defaults here are safe for any robot. Each robot's own values live in
// config/<robot id>.json, which the logger sends when it connects.
struct RobotConfigValues_t {
  // timing
  float update_interval_ms = 10;
  float telemetry_interval_ms = 10;

  // geometry
  float left_wheel_radius_mm = 16.0f;
  float right_wheel_radius_mm = 16.0f;
  float wheel_separation_mm = 90.0f;

  // motors
  float left_breakaway_pwm = 45;
  float right_breakaway_pwm = 45;
  float left_run_min_pwm = 20;
  float right_run_min_pwm = 20;
  float left_ff_offset_pwm = 17;
  float left_ff_mm_s_per_pwm = 3.5f;
  float right_ff_offset_pwm = 12;
  float right_ff_mm_s_per_pwm = 3.75f;

  // wheel speed controller
  float speed_kp = 0.3f;
  float speed_ki = 3.0f;
  float kick_ramp_pwm_per_s = 200;
  float speed_window_ms = 80;
  float speed_min_counts = 6;
  float stall_min_ms = 30;
  float accel_mm_s2 = 0;

  // "move" command
  float move_kp = 3.0f;
  float move_max_speed_mm_s = 40;
  float move_min_speed_mm_s = 15;
  float move_tolerance_mm = 0.2f;
  float move_stop_lead_s = 0.07f;
  float turn_stop_lead_s = 0.07f;
  float move_sync_kp = 0.0f;

  // "cmd" command
  float twist_kp = 4.0f;
  float twist_max_error_mm = 20.0f;
  float twist_left_lead_s = 0.088f;   // coast when stopping, s of travel at the wheel's speed
  float twist_right_lead_s = 0.079f;

  // line follower
  float line_threshold = 1400;
  float base_pwm = 45;
  float line_follow_gain = 0.01f;
  float recovery_left_pwm = -35;
  float recovery_right_pwm = 35;
  float recovery_timeout_ms = 5000;

  // line sensor: per-sensor levels (TODO 3.2), footprint (3.4), line width
  float dn1_white = 700;
  float dn2_white = 700;
  float dn3_white = 700;
  float dn4_white = 700;
  float dn5_white = 700;
  float dn1_black = 2200;
  float dn2_black = 2200;
  float dn3_black = 2200;
  float dn4_black = 2200;
  float dn5_black = 2200;
  float dn1_sigma_mm = 3.5f;
  float dn2_sigma_mm = 3.5f;
  float dn3_sigma_mm = 3.5f;
  float dn4_sigma_mm = 3.5f;
  float dn5_sigma_mm = 3.5f;
  float line_width_mm = 18;
  float line_detect_level = 0.5f;

  // line follower on the line sensor's offset (line_mode 1; 0 is the raw-PWM follower above)
  float line_mode = 0;
  float line_speed_mm_s = 100;        // outer wheel's speed; the robot slows on curves
  float line_lookahead_mm = 42;       // how far ahead of the wheels the sensors see the line
  float line_kp = 1.0f;               // times the pure pursuit curvature
  float line_kd = 0.0f;               // deg/s of turn per mm/s of offset change
  float line_accel_mm_s2 = 1000;      // wheel speed ramp while following
  float line_search_deg_s = 90;       // spin rate looking for a lost line

  // safety limits: the defaults are the maximum, config can only lower them
  float manual_max_pwm = 100;
  float manual_max_speed_mm_s = 250;
  float manual_timeout_ms = 500;
  float move_max_distance_mm = 500;
};

class RobotConfig_c {
  public:
    struct Entry {
      const char *key;
      float *value;
      float min;
      float max;
    };

    enum SetResult { SET_OK, SET_UNKNOWN_KEY, SET_OUT_OF_RANGE };

    RobotConfigValues_t values;

    RobotConfig_c();
    RobotConfig_c(const RobotConfig_c &) = delete;
    RobotConfig_c &operator=(const RobotConfig_c &) = delete;

    int size() const { return count; }
    const Entry &entry(int i) const { return entries[i]; }
    const Entry *find(const char *key) const;
    SetResult set(const char *key, float value);
    void resetToDefaults();

    // load() returns false if nothing is saved, or it was saved by firmware
    // with a different list of keys. save() only writes when something changed.
    bool load();
    bool save(bool &changed);

    // The robot's name (WiFi network and config file id), kept apart from the
    // settings so "config reset" doesn't touch it.
    static const int MAX_NAME_LEN = 20;
    static bool validName(const char *name);
    static bool loadName(char *name, int size);
    static bool saveName(const char *name);

  private:
    static const int MAX_ENTRIES = 80;
    Entry entries[MAX_ENTRIES];
    int count = 0;

    void add(const char *key, float *value, float min, float max);
    uint32_t keysHash() const;
};

#endif
