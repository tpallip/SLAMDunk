#include "GainTest.h"

const unsigned long GainTest_c::STILL_MS;
const unsigned long GainTest_c::REST_EXTRA_MAX_MS;
const int GainTest_c::MAX_PWM;

const char *GainTest_c::start(const Params &p, unsigned long now_ms, int32_t count) {
  if (p.wheel != 'L' && p.wheel != 'R' && p.wheel != 'S') return "wheel must be L, R or S";
  if (p.dir != 1 && p.dir != -1) return "dir must be 1 or -1";
  if (p.pwm_min < 0 || p.pwm_max > MAX_PWM || p.pwm_min > p.pwm_max) return "need 0 <= min <= max <= 100";
  if (p.pwm_step < 1) return "step must be at least 1";
  if (p.repeats < 1) return "repeats must be at least 1";
  if (p.hold_ms < 100 || p.rest_ms < STILL_MS) return "hold_ms >= 100 and rest_ms >= 300";

  params = p;
  trial = 0;
  last_count = count;
  last_move_ms = now_ms;
  beginRest(now_ms);
  return nullptr;
}

const char *GainTest_c::getPhaseName() const {
  switch (phase) {
    case REST: return "rest";
    case SETTLE: return "settle";
    case HOLD: return "hold";
    default: return "done";
  }
}

void GainTest_c::beginRest(unsigned long now_ms) {
  trial++;
  phase = REST;
  phase_start_ms = now_ms;
  going_up = false;
  index = levelsPerSweep() - 1;
}

void GainTest_c::beginLevel(unsigned long now_ms) {
  phase = SETTLE;
  phase_start_ms = now_ms;
}

void GainTest_c::update(unsigned long now_ms, int32_t count) {
  if (count != last_count) {
    last_count = count;
    last_move_ms = now_ms;
  }

  if (phase == REST) {
    unsigned long resting = now_ms - phase_start_ms;
    bool still = now_ms - last_move_ms >= STILL_MS;
    if (resting >= params.rest_ms && (still || resting >= params.rest_ms + REST_EXTRA_MAX_MS)) {
      beginLevel(now_ms);
    }
  }
  else if (phase == SETTLE) {
    if (now_ms - phase_start_ms >= params.settle_ms) {
      phase = HOLD;
      phase_start_ms = now_ms;
    }
  }
  else if (phase == HOLD) {
    if (now_ms - phase_start_ms < params.hold_ms) return;

    if (!going_up) {
      if (index > 0) {
        index--;
        beginLevel(now_ms);
        return;
      }
      going_up = true;   // reached the bottom, head back up
    }
    if (index < levelsPerSweep() - 1) {
      index++;
      beginLevel(now_ms);
    } else if (trial >= params.repeats) {
      phase = DONE;
    } else {
      beginRest(now_ms);
    }
  }
}

void GainTest_c::abort() {
  phase = DONE;
}
