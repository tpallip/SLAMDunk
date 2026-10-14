#ifndef KEEP_TURN_TEST_H
#define KEEP_TURN_TEST_H

#include <stdint.h>

// Keep-turning test for one wheel: how low can the PWM go once it's moving.
// Each trial: rest until still, spin at start_pwm for spin_ms, then drop the
// PWM by 1 every step_ms until the wheel stops (no count for STILL_MS).
// Controller_c does the encoder reads, motor output and printing.
class KeepTurnTest_c {
  public:
    enum Phase { IDLE, REST, SPIN, RAMP, DONE };

    struct Params {
      char wheel = 'L';
      int dir = 1;
      int start_pwm = 50;
      unsigned long step_ms = 200;
      int repeats = 5;
      unsigned long spin_ms = 1000;
      unsigned long rest_ms = 1500;
    };

    static const unsigned long STILL_MS = 300;
    static const unsigned long REST_EXTRA_MAX_MS = 3000;
    static const int MAX_PWM = 100;

    // returns nullptr if ok, otherwise an error message
    const char *start(const Params &params, unsigned long now_ms, int32_t count);
    void update(unsigned long now_ms, int32_t count);
    void abort();

    bool active() const { return phase == REST || phase == SPIN || phase == RAMP; }
    Phase getPhase() const { return phase; }
    const char *getPhaseName() const;
    const Params &getParams() const { return params; }
    int getPwm() const { return (phase == SPIN || phase == RAMP) ? params.dir * level : 0; }
    int getTrial() const { return trial; }

  private:
    Params params;
    Phase phase = IDLE;
    int level = 0;
    int trial = 0;
    unsigned long phase_start_ms = 0;
    unsigned long last_step_ms = 0;
    unsigned long last_move_ms = 0;
    int32_t last_count = 0;

    void beginRest(unsigned long now_ms);
    void endTrial(unsigned long now_ms);
};

#endif
