#include "RobotConfig.h"

#include <Arduino.h>
#include <Preferences.h>

static const char *NVS_NAMESPACE = "robotcfg";
static const char *NVS_KEY = "values";
static const char *NVS_NAME_KEY = "name";

RobotConfig_c::RobotConfig_c() {
  RobotConfigValues_t &v = values;
  add("update_interval_ms", &v.update_interval_ms, 5, 50);
  add("telemetry_interval_ms", &v.telemetry_interval_ms, 5, 1000);

  add("left_wheel_radius_mm", &v.left_wheel_radius_mm, 5, 50);
  add("right_wheel_radius_mm", &v.right_wheel_radius_mm, 5, 50);
  add("wheel_separation_mm", &v.wheel_separation_mm, 30, 200);

  add("left_breakaway_pwm", &v.left_breakaway_pwm, 0, 100);
  add("right_breakaway_pwm", &v.right_breakaway_pwm, 0, 100);
  add("left_run_min_pwm", &v.left_run_min_pwm, 0, 100);
  add("right_run_min_pwm", &v.right_run_min_pwm, 0, 100);
  add("left_ff_offset_pwm", &v.left_ff_offset_pwm, 0, 100);
  add("left_ff_mm_s_per_pwm", &v.left_ff_mm_s_per_pwm, 0.5f, 20);
  add("right_ff_offset_pwm", &v.right_ff_offset_pwm, 0, 100);
  add("right_ff_mm_s_per_pwm", &v.right_ff_mm_s_per_pwm, 0.5f, 20);

  add("speed_kp", &v.speed_kp, 0, 5);
  add("speed_ki", &v.speed_ki, 0, 50);
  add("kick_ramp_pwm_per_s", &v.kick_ramp_pwm_per_s, 0, 2000);
  add("speed_window_ms", &v.speed_window_ms, 30, 150);
  add("speed_min_counts", &v.speed_min_counts, 1, 20);
  add("stall_min_ms", &v.stall_min_ms, 10, 500);
  add("accel_mm_s2", &v.accel_mm_s2, 0, 5000);

  add("move_kp", &v.move_kp, 0, 20);
  add("move_max_speed_mm_s", &v.move_max_speed_mm_s, 5, 250);
  add("move_min_speed_mm_s", &v.move_min_speed_mm_s, 0, 100);
  add("move_tolerance_mm", &v.move_tolerance_mm, 0.05f, 10);
  add("move_stop_lead_s", &v.move_stop_lead_s, 0, 0.5f);
  add("turn_stop_lead_s", &v.turn_stop_lead_s, 0, 0.5f);
  add("move_sync_kp", &v.move_sync_kp, 0, 50);

  add("twist_kp", &v.twist_kp, 0, 50);
  add("twist_max_error_mm", &v.twist_max_error_mm, 0, 200);
  add("twist_left_lead_s", &v.twist_left_lead_s, 0, 0.5f);
  add("twist_right_lead_s", &v.twist_right_lead_s, 0, 0.5f);

  add("line_threshold", &v.line_threshold, 0, 65535);
  add("base_pwm", &v.base_pwm, 0, 100);
  add("line_follow_gain", &v.line_follow_gain, 0, 1);
  add("recovery_left_pwm", &v.recovery_left_pwm, -100, 100);
  add("recovery_right_pwm", &v.recovery_right_pwm, -100, 100);
  add("recovery_timeout_ms", &v.recovery_timeout_ms, 0, 60000);

  add("dn1_white", &v.dn1_white, 0, 65535);
  add("dn2_white", &v.dn2_white, 0, 65535);
  add("dn3_white", &v.dn3_white, 0, 65535);
  add("dn4_white", &v.dn4_white, 0, 65535);
  add("dn5_white", &v.dn5_white, 0, 65535);
  add("dn1_black", &v.dn1_black, 0, 65535);
  add("dn2_black", &v.dn2_black, 0, 65535);
  add("dn3_black", &v.dn3_black, 0, 65535);
  add("dn4_black", &v.dn4_black, 0, 65535);
  add("dn5_black", &v.dn5_black, 0, 65535);
  add("dn1_sigma_mm", &v.dn1_sigma_mm, 0.5f, 20);
  add("dn2_sigma_mm", &v.dn2_sigma_mm, 0.5f, 20);
  add("dn3_sigma_mm", &v.dn3_sigma_mm, 0.5f, 20);
  add("dn4_sigma_mm", &v.dn4_sigma_mm, 0.5f, 20);
  add("dn5_sigma_mm", &v.dn5_sigma_mm, 0.5f, 20);
  add("line_width_mm", &v.line_width_mm, 1, 100);
  add("line_detect_level", &v.line_detect_level, 0.05f, 1);

  add("line_mode", &v.line_mode, 0, 1);
  add("line_speed_mm_s", &v.line_speed_mm_s, 0, 250);
  add("line_lookahead_mm", &v.line_lookahead_mm, 5, 200);
  add("line_kp", &v.line_kp, 0, 10);
  add("line_kd", &v.line_kd, 0, 50);
  add("line_accel_mm_s2", &v.line_accel_mm_s2, 0, 5000);
  add("line_search_deg_s", &v.line_search_deg_s, 0, 400);

  RobotConfigValues_t defaults;
  add("manual_max_pwm", &v.manual_max_pwm, 0, defaults.manual_max_pwm);
  add("manual_max_speed_mm_s", &v.manual_max_speed_mm_s, 0, defaults.manual_max_speed_mm_s);
  add("manual_timeout_ms", &v.manual_timeout_ms, 50, defaults.manual_timeout_ms);
  add("move_max_distance_mm", &v.move_max_distance_mm, 0, defaults.move_max_distance_mm);
}

void RobotConfig_c::add(const char *key, float *value, float min, float max) {
  if (count < MAX_ENTRIES) {
    entries[count++] = {key, value, min, max};
  }
}

const RobotConfig_c::Entry *RobotConfig_c::find(const char *key) const {
  for (int i = 0; i < count; i++) {
    if (strcmp(entries[i].key, key) == 0) return &entries[i];
  }
  return nullptr;
}

RobotConfig_c::SetResult RobotConfig_c::set(const char *key, float value) {
  const Entry *e = find(key);
  if (e == nullptr) return SET_UNKNOWN_KEY;
  if (!isfinite(value) || value < e->min || value > e->max) return SET_OUT_OF_RANGE;
  *e->value = value;
  return SET_OK;
}

void RobotConfig_c::resetToDefaults() {
  values = RobotConfigValues_t();
}

// FNV-1a over the key names, so values saved by firmware with different keys
// aren't loaded into the wrong places
uint32_t RobotConfig_c::keysHash() const {
  uint32_t h = 2166136261u;
  for (int i = 0; i < count; i++) {
    for (const char *c = entries[i].key; ; c++) {
      h = (h ^ (uint8_t)*c) * 16777619u;
      if (*c == '\0') break;
    }
  }
  return h;
}

bool RobotConfig_c::load() {
  struct { uint32_t hash; float v[MAX_ENTRIES]; } blob;
  size_t size = sizeof(uint32_t) + count * sizeof(float);

  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, true)) return false;
  bool ok = prefs.getBytesLength(NVS_KEY) == size &&
            prefs.getBytes(NVS_KEY, &blob, size) == size &&
            blob.hash == keysHash();
  prefs.end();
  if (!ok) return false;

  for (int i = 0; i < count; i++) {
    const Entry &e = entries[i];
    if (isfinite(blob.v[i]) && blob.v[i] >= e.min && blob.v[i] <= e.max) {
      *e.value = blob.v[i];
    }
  }
  return true;
}

bool RobotConfig_c::save(bool &changed) {
  struct { uint32_t hash; float v[MAX_ENTRIES]; } blob, stored;
  size_t size = sizeof(uint32_t) + count * sizeof(float);
  blob.hash = keysHash();
  for (int i = 0; i < count; i++) blob.v[i] = *entries[i].value;

  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false)) return false;
  changed = !(prefs.getBytesLength(NVS_KEY) == size &&
              prefs.getBytes(NVS_KEY, &stored, size) == size &&
              memcmp(&blob, &stored, size) == 0);
  bool ok = !changed || prefs.putBytes(NVS_KEY, &blob, size) == size;
  prefs.end();
  return ok;
}

bool RobotConfig_c::validName(const char *name) {
  int len = strlen(name);
  if (len < 1 || len > MAX_NAME_LEN) return false;
  for (int i = 0; i < len; i++) {
    char c = name[i];
    if (!isalnum((unsigned char)c) && c != '-' && c != '_') return false;
  }
  return true;
}

bool RobotConfig_c::loadName(char *name, int size) {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, true)) return false;
  String saved = prefs.getString(NVS_NAME_KEY, "");
  prefs.end();
  if (!validName(saved.c_str()) || (int)saved.length() >= size) return false;
  strcpy(name, saved.c_str());
  return true;
}

bool RobotConfig_c::saveName(const char *name) {
  if (!validName(name)) return false;
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false)) return false;
  bool ok = prefs.putString(NVS_NAME_KEY, name) == strlen(name);
  prefs.end();
  return ok;
}
