#include "DeadbandTest.h"

const unsigned long DeadbandTest_c::STILL_MS;
const unsigned long DeadbandTest_c::REST_EXTRA_MAX_MS;
const int DeadbandTest_c::MAX_PWM;

int DeadbandTest_c::levelsPerSweep() const {
  return (params.pwm_max - params.pwm_min) / params.pwm_step + 1;
}

const char *DeadbandTest_c::start(const Params &p, unsigned long now_ms, int32_t count) {
  if (p.wheel != 'L' && p.wheel != 'R') return "wheel must be L or R";
  if (p.dir != 1 && p.dir != -1) return "dir must be 1 or -1";
  if (p.pwm_min < 0 || p.pwm_max > MAX_PWM || p.pwm_min > p.pwm_max) return "need 0 <= min <= max <= 100";
  if (p.pwm_step < 1) return "step must be at least 1";
  if (p.repeats < 1) return "repeats must be at least 1";
  if (p.run_ms < 100 || p.rest_ms < STILL_MS) return "run_ms >= 100 and rest_ms >= 300";

  params = p;
  trial = 0;
  repeat = 1;
  level = params.pwm_min;
  aborted = false;
  last_count = count;
  last_move_ms = now_ms;
  beginRest(now_ms);
  return nullptr;
}

void DeadbandTest_c::beginRest(unsigned long now_ms) {
  trial++;
  phase = REST;
  phase_start_ms = now_ms;
  rest_ok = false;
}

bool DeadbandTest_c::nextLevel() {
  level += params.pwm_step;
  if (level > params.pwm_max) {
    level = params.pwm_min;
    repeat++;
    if (repeat > params.repeats) {
      return false;
    }
  }
  return true;
}

void DeadbandTest_c::update(unsigned long now_ms, int32_t count) {
  if (count != last_count) {
    last_count = count;
    last_move_ms = now_ms;
  }

  if (phase == REST) {
    unsigned long resting = now_ms - phase_start_ms;
    bool still = now_ms - last_move_ms >= STILL_MS;
    if (resting >= params.rest_ms && (still || resting >= params.rest_ms + REST_EXTRA_MAX_MS)) {
      rest_ok = still;
      phase = RUN;
      phase_start_ms = now_ms;
    }
  }
  else if (phase == RUN) {
    if (now_ms - phase_start_ms >= params.run_ms) {
      if (nextLevel()) {
        beginRest(now_ms);
      } else {
        phase = DONE;
      }
    }
  }
}

void DeadbandTest_c::abort() {
  if (active()) {
    aborted = true;
  }
  phase = DONE;
}
