#include "MoveController.h"

#include <Arduino.h>

MoveController_c::MoveController_c(float kp, float max_speed_mm_s, float min_speed_mm_s,
                                   float tolerance_mm, float stop_lead_s)
  : kp(kp),
    max_speed_mm_s(max_speed_mm_s),
    min_speed_mm_s(min_speed_mm_s),
    tolerance_mm(tolerance_mm),
    stop_lead_s(stop_lead_s),
    active(false),
    timed_out(false),
    left_target(0),
    right_target(0),
    left_mm_per_count(1.0f),
    right_mm_per_count(1.0f),
    left_direction(1.0f),
    right_direction(1.0f),
    speed_mm_s(0.0f),
    left_done(true),
    right_done(true),
    left_error_mm(0.0f),
    right_error_mm(0.0f),
    left_last_speed(0.0f),
    right_last_speed(0.0f),
    deadline_ms(0) {
}

void MoveController_c::start(float left_mm, float right_mm, float speed_mm_s_in,
                             int32_t left_count, int32_t right_count,
                             float left_mm_per_count_in, float right_mm_per_count_in,
                             unsigned long now_ms) {
  left_mm_per_count = left_mm_per_count_in;
  right_mm_per_count = right_mm_per_count_in;
  left_target = left_count + (int32_t)lroundf(left_mm / left_mm_per_count);
  right_target = right_count + (int32_t)lroundf(right_mm / right_mm_per_count);
  left_direction = left_mm >= 0.0f ? 1.0f : -1.0f;
  right_direction = right_mm >= 0.0f ? 1.0f : -1.0f;
  speed_mm_s = speed_mm_s_in > 0.0f ? speed_mm_s_in : max_speed_mm_s;
  left_error_mm = left_mm;
  right_error_mm = right_mm;
  left_done = false;
  right_done = false;
  left_last_speed = 0.0f;
  right_last_speed = 0.0f;
  timed_out = false;
  active = true;
  deadline_ms = now_ms + 1000UL
                + (unsigned long)(2000.0f * fmaxf(fabsf(left_mm), fabsf(right_mm))
                                  / fminf(min_speed_mm_s, speed_mm_s));
}

void MoveController_c::cancel() {
  active = false;
}

bool MoveController_c::isActive() const {
  return active;
}

bool MoveController_c::update(int32_t left_count, int32_t right_count, unsigned long now_ms,
                              float &left_speed_mm_s, float &right_speed_mm_s) {
  left_speed_mm_s = 0.0f;
  right_speed_mm_s = 0.0f;
  if (!active) {
    return false;
  }

  // Distance still to go, measured in the direction of travel: negative once
  // a wheel has passed its target.
  left_error_mm = (int32_t)((uint32_t)left_target - (uint32_t)left_count) * left_mm_per_count;
  right_error_mm = (int32_t)((uint32_t)right_target - (uint32_t)right_count) * right_mm_per_count;
  float left_ahead = left_error_mm * left_direction;
  float right_ahead = right_error_mm * right_direction;

  // A wheel that arrives or passes its target stops and stays stopped, so the
  // move never hunts back and forth around the target. It stops early by the
  // distance it will coast once the motor is off.
  if (left_ahead <= tolerance_mm + left_last_speed * stop_lead_s) left_done = true;
  if (right_ahead <= tolerance_mm + right_last_speed * stop_lead_s) right_done = true;

  if ((left_done && right_done) || (long)(now_ms - deadline_ms) >= 0) {
    timed_out = !(left_done && right_done);
    active = false;
    return false;
  }

  float left_speed = left_done ? 0.0f : kp * left_ahead;
  float right_speed = right_done ? 0.0f : kp * right_ahead;

  // Scale both wheels by the same factor so they keep pace.
  float fastest = fmaxf(left_speed, right_speed);
  if (fastest > speed_mm_s) {
    left_speed *= speed_mm_s / fastest;
    right_speed *= speed_mm_s / fastest;
  }
  float slowest = fminf(min_speed_mm_s, speed_mm_s);
  if (!left_done) left_speed = fmaxf(left_speed, slowest);
  if (!right_done) right_speed = fmaxf(right_speed, slowest);

  left_last_speed = left_speed;
  right_last_speed = right_speed;
  left_speed_mm_s = left_direction * left_speed;
  right_speed_mm_s = right_direction * right_speed;
  return true;
}

float MoveController_c::getLeftError() const {
  return left_error_mm;
}

float MoveController_c::getRightError() const {
  return right_error_mm;
}

bool MoveController_c::timedOut() const {
  return timed_out;
}
