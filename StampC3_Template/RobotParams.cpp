#include "RobotParams.h"

#include <Preferences.h>
#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

namespace {

enum ParamType_t { PARAM_U32, PARAM_F32 };

struct ParamDescriptor_t {
  const char *name;
  ParamType_t type;
  size_t offset;
  float min_value;
  float max_value;
};

#define PARAM_ROW(field, type, lo, hi) \
  { #field, type, offsetof(RobotParams_t, field), lo, hi }

// Ranges are deliberately permissive sanity limits, not tuned values.
// Geometry limits match those accepted by the middleware motion helpers.
const ParamDescriptor_t PARAM_TABLE[] = {
  PARAM_ROW(update_interval_ms,     PARAM_U32, 1,      1000),
  PARAM_ROW(telemetry_interval_ms,  PARAM_U32, 10,     10000),
  PARAM_ROW(log_interval_ms,        PARAM_U32, 5,      10000),
  PARAM_ROW(trial_sample_count,     PARAM_U32, 1,      20000),
  PARAM_ROW(recovery_timeout_ms,    PARAM_U32, 0,      600000),
  PARAM_ROW(line_threshold,         PARAM_F32, 0,      1),
  PARAM_ROW(base_pwm,               PARAM_F32, -200,   200),
  PARAM_ROW(line_follow_gain,       PARAM_F32, -200,   200),
  PARAM_ROW(recovery_left_pwm,      PARAM_F32, -200,   200),
  PARAM_ROW(recovery_right_pwm,     PARAM_F32, -200,   200),
  PARAM_ROW(motor_pwm_limit,        PARAM_U32, 0,      200),
  PARAM_ROW(left_wheel_radius_mm,   PARAM_F32, 5,      50),
  PARAM_ROW(right_wheel_radius_mm,  PARAM_F32, 5,      50),
  PARAM_ROW(wheel_separation_mm,    PARAM_F32, 20,     200),
  PARAM_ROW(encoder_counts_per_rev, PARAM_F32, 1,      100000),
  PARAM_ROW(white_dn1,              PARAM_F32, 0,      65535),
  PARAM_ROW(white_dn2,              PARAM_F32, 0,      65535),
  PARAM_ROW(white_dn3,              PARAM_F32, 0,      65535),
  PARAM_ROW(white_dn4,              PARAM_F32, 0,      65535),
  PARAM_ROW(white_dn5,              PARAM_F32, 0,      65535),
  PARAM_ROW(black_dn1,              PARAM_F32, 0,      65535),
  PARAM_ROW(black_dn2,              PARAM_F32, 0,      65535),
  PARAM_ROW(black_dn3,              PARAM_F32, 0,      65535),
  PARAM_ROW(black_dn4,              PARAM_F32, 0,      65535),
  PARAM_ROW(black_dn5,              PARAM_F32, 0,      65535),
};

#undef PARAM_ROW

const size_t PARAM_COUNT = sizeof(PARAM_TABLE) / sizeof(PARAM_TABLE[0]);

const char *NVS_NAMESPACE = "params";
const char *NVS_KEY_BLOB = "blob";
const char *NVS_KEY_LAYOUT = "layout";

const ParamDescriptor_t *findParam(const char *name) {
  if (name == nullptr) return nullptr;
  for (size_t i = 0; i < PARAM_COUNT; i++) {
    if (strcmp(PARAM_TABLE[i].name, name) == 0) return &PARAM_TABLE[i];
  }
  return nullptr;
}

/** FNV-1a hash of the table layout, so stale flash data is never applied. */
uint32_t layoutSignature() {
  uint32_t hash = 2166136261u;
  auto mix = [&hash](uint8_t byte) {
    hash ^= byte;
    hash *= 16777619u;
  };
  for (size_t i = 0; i < PARAM_COUNT; i++) {
    for (const char *c = PARAM_TABLE[i].name; *c; c++) mix((uint8_t)*c);
    mix((uint8_t)PARAM_TABLE[i].type);
    mix((uint8_t)PARAM_TABLE[i].offset);
  }
  uint32_t size = sizeof(RobotParams_t);
  for (int i = 0; i < 4; i++) mix((uint8_t)(size >> (8 * i)));
  return hash;
}

bool onlyTrailingSpace(const char *cursor) {
  while (isspace((unsigned char)*cursor)) cursor++;
  return *cursor == '\0';
}

}  // namespace

ParamSetResult_t RobotParamsStore_c::set(const char *name, const char *text) {
  const ParamDescriptor_t *param = findParam(name);
  if (param == nullptr) return PARAM_SET_UNKNOWN;
  if (text == nullptr) return PARAM_SET_BAD_VALUE;

  while (isspace((unsigned char)*text)) text++;
  errno = 0;
  char *end = nullptr;
  float parsed = strtof(text, &end);
  if (end == text || errno == ERANGE || !isfinite(parsed) || !onlyTrailingSpace(end)) {
    return PARAM_SET_BAD_VALUE;
  }
  if (parsed < param->min_value || parsed > param->max_value) {
    return PARAM_SET_OUT_OF_RANGE;
  }

  uint8_t *base = reinterpret_cast<uint8_t *>(&values);
  if (param->type == PARAM_U32) {
    if (parsed != floorf(parsed)) return PARAM_SET_BAD_VALUE;
    uint32_t value = (uint32_t)parsed;
    memcpy(base + param->offset, &value, sizeof(value));
  } else {
    memcpy(base + param->offset, &parsed, sizeof(parsed));
  }
  version++;
  return PARAM_SET_OK;
}

bool RobotParamsStore_c::format(const char *name, char *buffer, size_t buffer_size) const {
  const ParamDescriptor_t *param = findParam(name);
  if (param == nullptr || buffer == nullptr || buffer_size == 0) return false;

  const uint8_t *base = reinterpret_cast<const uint8_t *>(&values);
  if (param->type == PARAM_U32) {
    uint32_t value;
    memcpy(&value, base + param->offset, sizeof(value));
    snprintf(buffer, buffer_size, "%lu", (unsigned long)value);
  } else {
    float value;
    memcpy(&value, base + param->offset, sizeof(value));
    snprintf(buffer, buffer_size, "%.6g", (double)value);
  }
  return true;
}

size_t RobotParamsStore_c::count() const {
  return PARAM_COUNT;
}

const char *RobotParamsStore_c::nameAt(size_t index) const {
  return index < PARAM_COUNT ? PARAM_TABLE[index].name : nullptr;
}

void RobotParamsStore_c::restoreDefaults() {
  values = RobotParams_t();
  version++;
}

bool RobotParamsStore_c::saveToFlash() const {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false)) return false;
  bool ok = prefs.putUInt(NVS_KEY_LAYOUT, layoutSignature()) == sizeof(uint32_t)
            && prefs.putBytes(NVS_KEY_BLOB, &values, sizeof(values)) == sizeof(values);
  prefs.end();
  return ok;
}

bool RobotParamsStore_c::loadFromFlash() {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, true)) return false;

  bool ok = false;
  if (prefs.getUInt(NVS_KEY_LAYOUT, 0) == layoutSignature()
      && prefs.getBytesLength(NVS_KEY_BLOB) == sizeof(RobotParams_t)) {
    RobotParams_t stored;
    if (prefs.getBytes(NVS_KEY_BLOB, &stored, sizeof(stored)) == sizeof(stored)) {
      values = stored;
      version++;
      ok = true;
    }
  }
  prefs.end();
  return ok;
}

bool RobotParamsStore_c::clearFlash() const {
  Preferences prefs;
  if (!prefs.begin(NVS_NAMESPACE, false)) return false;
  bool ok = prefs.clear();
  prefs.end();
  return ok;
}
