#ifndef GAIN_TEST_H
#define GAIN_TEST_H

#include <stdint.h>

// Speed vs PWM for one wheel. Each trial: rest until still, start at pwm_max,
// step down to pwm_min and back up to pwm_max without stopping. Every level
// is held for settle_ms (speed settling) then hold_ms (speed measured).
// Controller_c does the encoder reads, motor output and printing.
class GainTest_c {
  public:
    enum Phase { IDLE, REST, SETTLE, HOLD, DONE };

    struct Params {
      char wheel = 'L';
      int dir = 1;
      int pwm_min = 20;
      int pwm_max = 100;
      int pwm_step = 5;
      int repeats = 3;
      unsigned long settle_ms = 500;
      unsigned long hold_ms = 1000;
      unsigned long rest_ms = 1500;
    };

    static const unsigned long STILL_MS = 300;
    static const unsigned long REST_EXTRA_MAX_MS = 3000;
    static const int MAX_PWM = 100;

    // returns nullptr if ok, otherwise an error message
    const char *start(const Params &params, unsigned long now_ms, int32_t count);
    void update(unsigned long now_ms, int32_t count);
    void abort();

    bool active() const { return phase == REST || phase == SETTLE || phase == HOLD; }
    const char *getPhaseName() const;
    const char *getLegName() const { return going_up ? "up" : "down"; }
    const Params &getParams() const { return params; }
    int getPwm() const { return (phase == SETTLE || phase == HOLD) ? params.dir * level() : 0; }
    int getTrial() const { return trial; }
    int levelsPerSweep() const { return (params.pwm_max - params.pwm_min) / params.pwm_step + 1; }

  private:
    Params params;
    Phase phase = IDLE;
    int index = 0;           // level = pwm_min + index * pwm_step
    bool going_up = false;
    int trial = 0;
    unsigned long phase_start_ms = 0;
    unsigned long last_move_ms = 0;
    int32_t last_count = 0;

    int level() const { return params.pwm_min + index * params.pwm_step; }
    void beginRest(unsigned long now_ms);
    void beginLevel(unsigned long now_ms);
};

#endif
