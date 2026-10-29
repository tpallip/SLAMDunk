#include "LineSensor.h"

#include <Arduino.h>
#include <math.h>

static const float ANGLES_DEG[LineSensor_c::N] = {45, 14, 0, -14, -45};

static float normalCdf(float z) {
  return 0.5f * (1.0f + erff(z * 0.70710678f));
}

void LineSensor_c::configure() {
  for (int i = 0; i < N; i++) {
    float y = RADIUS_MM * sinf(ANGLES_DEG[i] * (float)M_PI / 180.0f);
    float sigma = sigma_mm[i] > 0.1f ? sigma_mm[i] : 0.1f;
    for (int j = 0; j < OFFSETS; j++) {
      float d = y - (-MAX_OFFSET_MM + j * STEP_MM);   // sensor's sideways distance from the line's middle
      float n = normalCdf((d + line_width_mm / 2) / sigma) - normalCdf((d - line_width_mm / 2) / sigma);
      predicted[i][j] = (int16_t)lroundf(n * SCALE);
    }
  }
  max_update_us = 0;
}

int32_t LineSensor_c::cost(int j, int previous_j) const {
  int32_t c = 0;
  for (int i = 0; i < N; i++) {
    int32_t diff = norm_q[i] - predicted[i][j];
    c += diff * diff;
  }
  int32_t jump = j - previous_j;
  return c + CONTINUITY_X2 * jump * jump / 2;
}

void LineSensor_c::update(const uint16_t reading[N]) {
  unsigned long start_us = micros();
  float top = 0.0f;
  for (int i = 0; i < N; i++) {
    float span = black[i] - white[i];
    float n = span > 1.0f ? (reading[i] - white[i]) / span : 0.0f;
    norm[i] = n < 0.0f ? 0.0f : n > 1.0f ? 1.0f : n;
    norm_q[i] = (int32_t)lroundf(norm[i] * SCALE);
    if (norm[i] > top) top = norm[i];
  }
  detected = top >= detect_level;
  if (detected) {
    int previous_j = (int)lroundf((offset_mm + MAX_OFFSET_MM) / STEP_MM);
    // Every 2 mm, then every 0.5 mm within 2 mm of the best.
    int best = 0;
    int32_t best_cost = cost(0, previous_j);
    for (int j = COARSE; j < OFFSETS; j += COARSE) {
      int32_t c = cost(j, previous_j);
      if (c < best_cost) { best = j; best_cost = c; }
    }
    int lo = best - COARSE < 0 ? 0 : best - COARSE;
    int hi = best + COARSE > OFFSETS - 1 ? OFFSETS - 1 : best + COARSE;
    int32_t fine[2 * COARSE + 1];
    for (int j = lo; j <= hi; j++) {
      fine[j - lo] = j == best ? best_cost : cost(j, previous_j);
      if (fine[j - lo] < best_cost) { best = j; best_cost = fine[j - lo]; }
    }

    // Parabola through the best point and its neighbours, for under 0.5 mm.
    float shift = 0.0f;
    if (best > lo && best < hi) {
      float a = fine[best - 1 - lo], b = best_cost, c = fine[best + 1 - lo];
      float curve = a - 2.0f * b + c;
      if (curve > 0.0f) shift = 0.5f * (a - c) / curve;
    }
    int32_t jump = best - previous_j;
    int32_t fit = best_cost - CONTINUITY_X2 * jump * jump / 2;
    residual = sqrtf((fit > 0 ? fit : 0) / (float)N) / SCALE;
    offset_mm = -MAX_OFFSET_MM + (best + shift) * STEP_MM;
  }
  update_us = micros() - start_us;
  if (update_us > max_update_us) max_update_us = update_us;
}
