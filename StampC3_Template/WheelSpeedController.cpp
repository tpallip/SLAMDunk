#include "WheelSpeedController.h"

#include <Arduino.h>

WheelSpeedController_c::WheelSpeedController_c(float kp, float ki, float ff_offset_pwm,
                                               float ff_mm_s_per_pwm, float max_pwm,
                                               float breakaway_pwm, float run_min_pwm)
  : kp(kp),
    ki(ki),
    ff_offset_pwm(ff_offset_pwm),
    ff_mm_s_per_pwm(ff_mm_s_per_pwm),
    max_pwm(max_pwm),
    breakaway_pwm(breakaway_pwm),
    run_min_pwm(run_min_pwm),
    target_mm_s(0.0f),
    setpoint_mm_s(0.0f),
    speed_mm_s(0.0f),
    integral_mm(0.0f),
    mm_per_count(0.0f),
    kicking(false),
    kick_pwm(0.0f),
    samples(0),
    newest(0),
    last_change_ms(0) {
}

void WheelSpeedController_c::setTarget(float new_target_mm_s) {
  // An integral built up driving one way would push the wrong way after a
  // reversal, so start it again when the direction changes.
  if (new_target_mm_s * target_mm_s <= 0.0f) {
    integral_mm = 0.0f;
  }
  target_mm_s = new_target_mm_s;
}

float WheelSpeedController_c::getTarget() const {
  return target_mm_s;
}

float WheelSpeedController_c::getSetpoint() const {
  return setpoint_mm_s;
}

float WheelSpeedController_c::getSpeed() const {
  return speed_mm_s;
}

bool WheelSpeedController_c::isKicking() const {
  return kicking;
}


void WheelSpeedController_c::reset() {
  target_mm_s = 0.0f;
  setpoint_mm_s = 0.0f;
  integral_mm = 0.0f;
}

void WheelSpeedController_c::measure(int32_t count, unsigned long now_ms, float mm_per_count_in) {
  mm_per_count = mm_per_count_in;
  if (samples > 0 && now_ms == times[newest]) {
    return;
  }
  if (samples == 0 || count != counts[newest]) {
    last_change_ms = now_ms;
  }
  newest = (newest + 1) % HISTORY;
  counts[newest] = count;
  times[newest] = now_ms;
  if (samples < HISTORY) samples++;
  if (samples < 2) {
    return;
  }

  // Window long enough for about min_counts counts at the previous speed,
  // 30 ms .. window_ms. Choosing it from the previous estimate rather than
  // stopping once the counts in it reach min_counts avoids favouring windows
  // that happened to catch counts early, which reads fast. Subtract counts as
  // unsigned so a counter wrap still gives the small signed change.
  unsigned long want_ms = window_ms;
  if (fabsf(speed_mm_s) > 0.0f) {
    float ms_for_counts = 1000.0f * min_counts * mm_per_count / fabsf(speed_mm_s);
    want_ms = (unsigned long)constrain(ms_for_counts, 30.0f, (float)window_ms);
  }
  int32_t delta = 0;
  unsigned long span_ms = 0;
  for (int k = 1; k < samples; k++) {
    int j = (newest - k + HISTORY) % HISTORY;
    delta = (int32_t)((uint32_t)count - (uint32_t)counts[j]);
    span_ms = now_ms - times[j];
    if (span_ms >= want_ms) break;
  }
  float speed = span_ms > 0 ? delta * mm_per_count * 1000.0f / span_ms : 0.0f;

  // No count for a while means the wheel is at most one count per that time:
  // cap the estimate so a stopping or stuck wheel reads slow at once.
  unsigned long quiet_ms = now_ms - last_change_ms;
  if (quiet_ms > 0) {
    float bound = mm_per_count * 1000.0f / quiet_ms;
    if (fabsf(speed) > bound) speed = speed > 0.0f ? bound : -bound;
  }
  speed_mm_s = speed;
}

void WheelSpeedController_c::stepSetpoint(float dt_s) {
  float direction = target_mm_s > 0.0f ? 1.0f : -1.0f;
  if (accel_mm_s2 <= 0.0f) {
    setpoint_mm_s = target_mm_s;
  } else if (setpoint_mm_s * direction <= 0.0f) {
    // starting, or reversing: climb from zero
    setpoint_mm_s = direction * fminf(fabsf(target_mm_s), accel_mm_s2 * dt_s);
  } else if (fabsf(target_mm_s) <= fabsf(setpoint_mm_s)) {
    setpoint_mm_s = target_mm_s;
  } else {
    setpoint_mm_s += direction * fminf(fabsf(target_mm_s) - fabsf(setpoint_mm_s), accel_mm_s2 * dt_s);
  }
}

float WheelSpeedController_c::update(float dt_s) {
  if (target_mm_s == 0.0f) {
    kicking = false;
    integral_mm = 0.0f;
    setpoint_mm_s = 0.0f;
    return 0.0f;
  }
  stepSetpoint(dt_s);

  float direction = setpoint_mm_s > 0.0f ? 1.0f : -1.0f;
  float feedforward = direction * (ff_offset_pwm + fabsf(setpoint_mm_s) / ff_mm_s_per_pwm);
  float error = setpoint_mm_s - speed_mm_s;

  // Stuck: no count for longer than the target speed allows (2.5 count periods,
  // at least stall_min_ms). Push from the breakaway PWM upwards until the
  // wheel moves, holding the integral, which would otherwise wind up.
  if (breakaway_pwm > 0.0f && mm_per_count > 0.0f && samples > 1) {
    unsigned long count_period_ms = (unsigned long)(1000.0f * mm_per_count / fabsf(setpoint_mm_s));
    unsigned long allowed_ms = max(stall_min_ms, count_period_ms * 5 / 2);
    if (times[newest] - last_change_ms > allowed_ms) {
      float pi = fabsf(feedforward + kp * error + ki * integral_mm);
      kick_pwm = kicking ? kick_pwm + kick_ramp_pwm_per_s * dt_s : fmaxf(breakaway_pwm, pi);
      kicking = true;
      return direction * fminf(kick_pwm, max_pwm);
    }
  }
  kicking = false;

  float new_integral = integral_mm + error * dt_s;
  float pwm = feedforward + kp * error + ki * new_integral;

  // Floor: a wheel meant to be turning keeps at least run_min_pwm, so the PI
  // can't stall it after it lurches free. Don't wind the integral down here.
  if (run_min_pwm > 0.0f && pwm * direction < run_min_pwm) {
    pwm = direction * run_min_pwm;
    if (error * direction < 0.0f) new_integral = integral_mm;
  }

  // Anti-windup: while the output is held at the limit, keep the integral
  // only if this error would move the output back inside it.
  if (pwm > max_pwm) {
    pwm = max_pwm;
    if (error < 0.0f) integral_mm = new_integral;
  } else if (pwm < -max_pwm) {
    pwm = -max_pwm;
    if (error > 0.0f) integral_mm = new_integral;
  } else {
    integral_mm = new_integral;
  }

  return pwm;
}
