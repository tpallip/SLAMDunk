#ifndef DEADBAND_TEST_H
#define DEADBAND_TEST_H

#include <stdint.h>

// Open loop deadband test for one wheel.
// Each trial: wait until the wheel is still, then run one PWM level for run_ms.
// Levels go pwm_min..pwm_max in steps of pwm_step, and the sweep is repeated.
// Controller_c does the encoder reads, motor output and printing.
class DeadbandTest_c {
  public:
    enum Phase { IDLE, REST, RUN, DONE };

    struct Params {
      char wheel = 'L';
      int dir = 1;
      int pwm_min = 0;
      int pwm_max = 60;
      int pwm_step = 5;
      int repeats = 3;
      unsigned long rest_ms = 1500;
      unsigned long run_ms = 1500;
    };

    static const unsigned long STILL_MS = 300;         // no counts for this long = stopped
    static const unsigned long REST_EXTRA_MAX_MS = 3000;
    static const int MAX_PWM = 100;

    // returns nullptr if ok, otherwise an error message
    const char *start(const Params &params, unsigned long now_ms, int32_t count);
    void update(unsigned long now_ms, int32_t count);
    void abort();

    bool active() const { return phase == REST || phase == RUN; }
    Phase getPhase() const { return phase; }
    const Params &getParams() const { return params; }
    int getPwm() const { return phase == RUN ? params.dir * level : 0; }
    int getLevel() const { return level; }
    int getTrial() const { return trial; }
    int getRepeat() const { return repeat; }
    bool getRestOk() const { return rest_ok; }
    bool wasAborted() const { return aborted; }
    int levelsPerSweep() const;

  private:
    Params params;
    Phase phase = IDLE;
    int level = 0;
    int trial = 0;
    int repeat = 0;
    bool rest_ok = false;
    bool aborted = false;
    unsigned long phase_start_ms = 0;
    unsigned long last_move_ms = 0;
    int32_t last_count = 0;

    void beginRest(unsigned long now_ms);
    bool nextLevel();
};

#endif
