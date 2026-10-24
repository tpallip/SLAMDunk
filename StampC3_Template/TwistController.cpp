#include "TwistController.h"

#include <Arduino.h>

void TwistController_c::set(float v_mm_s, float omega_deg_s, float separation_mm,
                            int32_t left_count, int32_t right_count,
                            float left_mm_per_count_in, float right_mm_per_count_in) {
  float half_diff = omega_deg_s * DEG_TO_RAD * separation_mm / 2.0f;
  left_target_mm_s = v_mm_s - half_diff;
  right_target_mm_s = v_mm_s + half_diff;
  left_mm_per_count = left_mm_per_count_in;
  right_mm_per_count = right_mm_per_count_in;
  if (!active) {
    left_start = left_count;
    right_start = right_count;
    wanted_diff_mm = 0.0f;
    error_mm = 0.0f;
    active = true;
  }
}

void TwistController_c::stop() {
  active = false;
  error_mm = 0.0f;
}

bool TwistController_c::isActive() const {
  return active;
}

float TwistController_c::getError() const {
  return error_mm;
}

void TwistController_c::update(int32_t left_count, int32_t right_count, float dt_s,
                               float &left_speed_mm_s, float &right_speed_mm_s) {
  left_speed_mm_s = 0.0f;
  right_speed_mm_s = 0.0f;
  if (!active) {
    return;
  }

  // Commanded and measured right - left distance since the start. Counts are
  // subtracted as unsigned so a counter wrap still gives the small change.
  wanted_diff_mm += (right_target_mm_s - left_target_mm_s) * dt_s;
  float left_mm = (int32_t)((uint32_t)left_count - (uint32_t)left_start) * left_mm_per_count;
  float right_mm = (int32_t)((uint32_t)right_count - (uint32_t)right_start) * right_mm_per_count;
  float diff_mm = right_mm - left_mm;

  // Hold the error within max_error_mm by moving the commanded total, so a
  // stuck wheel can't build up a debt that later swings the robot round.
  error_mm = wanted_diff_mm - diff_mm;
  if (error_mm > max_error_mm) {
    error_mm = max_error_mm;
    wanted_diff_mm = diff_mm + max_error_mm;
  } else if (error_mm < -max_error_mm) {
    error_mm = -max_error_mm;
    wanted_diff_mm = diff_mm - max_error_mm;
  }

  float correction = kp * error_mm;
  float left = left_target_mm_s - correction / 2.0f;
  float right = right_target_mm_s + correction / 2.0f;

  // Scale both down together if either is too fast, keeping the curvature.
  float fastest = fmaxf(fabsf(left), fabsf(right));
  if (fastest > max_speed_mm_s) {
    left *= max_speed_mm_s / fastest;
    right *= max_speed_mm_s / fastest;
  }
  left_speed_mm_s = left;
  right_speed_mm_s = right;
}
