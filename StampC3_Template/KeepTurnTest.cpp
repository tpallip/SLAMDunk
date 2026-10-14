#include "KeepTurnTest.h"

const unsigned long KeepTurnTest_c::STILL_MS;
const unsigned long KeepTurnTest_c::REST_EXTRA_MAX_MS;
const int KeepTurnTest_c::MAX_PWM;

const char *KeepTurnTest_c::start(const Params &p, unsigned long now_ms, int32_t count) {
  if (p.wheel != 'L' && p.wheel != 'R' && p.wheel != 'S') return "wheel must be L, R or S";
  if (p.dir != 1 && p.dir != -1) return "dir must be 1 or -1";
  if (p.start_pwm < 1 || p.start_pwm > MAX_PWM) return "need 1 <= start <= 100";
  if (p.step_ms < 10) return "step_ms must be at least 10";
  if (p.repeats < 1) return "repeats must be at least 1";
  if (p.spin_ms < STILL_MS || p.rest_ms < STILL_MS) return "spin_ms and rest_ms must be >= 300";

  params = p;
  trial = 0;
  last_count = count;
  last_move_ms = now_ms;
  beginRest(now_ms);
  return nullptr;
}

const char *KeepTurnTest_c::getPhaseName() const {
  switch (phase) {
    case REST: return "rest";
    case SPIN: return "spin";
    case RAMP: return "ramp";
    default: return "done";
  }
}

void KeepTurnTest_c::beginRest(unsigned long now_ms) {
  trial++;
  phase = REST;
  phase_start_ms = now_ms;
  level = params.start_pwm;
}

void KeepTurnTest_c::endTrial(unsigned long now_ms) {
  if (trial >= params.repeats) {
    phase = DONE;
  } else {
    beginRest(now_ms);
  }
}

void KeepTurnTest_c::update(unsigned long now_ms, int32_t count) {
  if (count != last_count) {
    last_count = count;
    last_move_ms = now_ms;
  }
  bool still = now_ms - last_move_ms >= STILL_MS;

  if (phase == REST) {
    unsigned long resting = now_ms - phase_start_ms;
    if (resting >= params.rest_ms && (still || resting >= params.rest_ms + REST_EXTRA_MAX_MS)) {
      phase = SPIN;
      phase_start_ms = now_ms;
    }
  }
  else if (phase == SPIN) {
    if (now_ms - phase_start_ms >= params.spin_ms) {
      phase = RAMP;
      phase_start_ms = now_ms;
      last_step_ms = now_ms;
    }
  }
  else if (phase == RAMP) {
    if (still || level <= 0) {
      endTrial(now_ms);
    }
    else if (now_ms - last_step_ms >= params.step_ms) {
      level--;
      last_step_ms += params.step_ms;
    }
  }
}

void KeepTurnTest_c::abort() {
  phase = DONE;
}
