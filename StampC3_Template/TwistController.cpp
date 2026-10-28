#include "TwistController.h"

#include <Arduino.h>

void TwistController_c::set(float v_mm_s, float omega_deg_s, float separation_mm,
                            int32_t left_count, int32_t right_count,
                            float left_mm_per_count_in, float right_mm_per_count_in) {
  float half_diff = omega_deg_s * DEG_TO_RAD * separation_mm / 2.0f;
  left_command_mm_s = v_mm_s - half_diff;
  right_command_mm_s = v_mm_s + half_diff;
  left_mm_per_count = left_mm_per_count_in;
  right_mm_per_count = right_mm_per_count_in;
  if (!isStopping()) {
    trimming = false;
  }
  if (!active) {
    left_start = left_count;
    right_start = right_count;
    left_target_mm_s = 0.0f;
    right_target_mm_s = 0.0f;
    wanted_diff_mm = 0.0f;
    error_mm = 0.0f;
    active = true;
  }
}

void TwistController_c::stop() {
  active = false;
  trimming = false;
  left_target_mm_s = 0.0f;
  right_target_mm_s = 0.0f;
  error_mm = 0.0f;
}

bool TwistController_c::isActive() const {
  return active;
}

bool TwistController_c::isStopping() const {
  return active && left_command_mm_s == 0.0f && right_command_mm_s == 0.0f;
}

float TwistController_c::getError() const {
  return error_mm;
}

void TwistController_c::update(int32_t left_count, int32_t right_count, float dt_s,
                               float left_measured_mm_s, float right_measured_mm_s,
                               float &left_speed_mm_s, float &right_speed_mm_s) {
  left_speed_mm_s = 0.0f;
  right_speed_mm_s = 0.0f;
  if (!active) {
    return;
  }

  // Move the targets towards the command, the wheel that changes most at
  // accel_mm_s2 and the other in proportion, so the curvature is kept.
  float left_change = left_command_mm_s - left_target_mm_s;
  float right_change = right_command_mm_s - right_target_mm_s;
  float largest = fmaxf(fabsf(left_change), fabsf(right_change));
  float allowed = accel_mm_s2 * dt_s;
  if (accel_mm_s2 > 0.0f && largest > allowed) {
    left_target_mm_s += left_change * allowed / largest;
    right_target_mm_s += right_change * allowed / largest;
  } else {
    left_target_mm_s = left_command_mm_s;
    right_target_mm_s = right_command_mm_s;
  }
  // Measured right - left distance since the start. Counts are subtracted as
  // unsigned so a counter wrap still gives the small change.
  float left_mm = (int32_t)((uint32_t)left_count - (uint32_t)left_start) * left_mm_per_count;
  float right_mm = (int32_t)((uint32_t)right_count - (uint32_t)right_start) * right_mm_per_count;
  float diff_mm = right_mm - left_mm;

  // Slowing to a stop ends near the slowest steady wheel speed: below it the
  // wheels can only stick and slip. Stop them one at a time from there.
  if (isStopping() && (trimming || fmaxf(fabsf(left_target_mm_s), fabsf(right_target_mm_s)) < stop_speed_mm_s)) {
    trim(diff_mm, dt_s, left_measured_mm_s, right_measured_mm_s, left_speed_mm_s, right_speed_mm_s);
    return;
  }

  wanted_diff_mm += (right_target_mm_s - left_target_mm_s) * dt_s;

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

void TwistController_c::trim(float diff_mm, float dt_s, float left_measured_mm_s, float right_measured_mm_s,
                             float &left_speed_mm_s, float &right_speed_mm_s) {
  if (!trimming) {
    trimming = true;
    left_stopped = false;
    right_stopped = false;
    trim_s = 0.0f;
    // Direction from the measured speed; from the target if the wheel reads still.
    float left_ref = left_measured_mm_s != 0.0f ? left_measured_mm_s : left_target_mm_s;
    float right_ref = right_measured_mm_s != 0.0f ? right_measured_mm_s : right_target_mm_s;
    left_direction = left_ref >= 0.0f ? 1.0f : -1.0f;
    right_direction = right_ref >= 0.0f ? 1.0f : -1.0f;
  }
  trim_s += dt_s;

  // Heading error once both wheels have coasted to a stop (+: the right - left
  // difference should still grow).
  float final_diff = diff_mm + right_measured_mm_s * right_stop_lead_s - left_measured_mm_s * left_stop_lead_s;
  float end_error = wanted_diff_mm - final_diff;
  float need = 0.0f;
  if (end_error > trim_tolerance_mm) need = 1.0f;
  else if (end_error < -trim_tolerance_mm) need = -1.0f;

  // A wheel keeps going only while its travel moves the difference the way
  // it needs to go: the right wheel adds to it, the left takes from it.
  if (need * right_direction <= 0.0f) right_stopped = true;
  if (need * -left_direction <= 0.0f) left_stopped = true;
  // A wheel that has stalled, or has had long enough, stops anyway.
  if (!left_stopped && (fabsf(left_measured_mm_s) < 0.25f * stop_speed_mm_s || trim_s > max_trim_s)) {
    left_stopped = true;
  }
  if (!right_stopped && (fabsf(right_measured_mm_s) < 0.25f * stop_speed_mm_s || trim_s > max_trim_s)) {
    right_stopped = true;
  }

  error_mm = end_error;
  if (left_stopped && right_stopped) {
    stop();
    return;
  }
  left_speed_mm_s = left_stopped ? 0.0f : left_direction * stop_speed_mm_s;
  right_speed_mm_s = right_stopped ? 0.0f : right_direction * stop_speed_mm_s;
}
