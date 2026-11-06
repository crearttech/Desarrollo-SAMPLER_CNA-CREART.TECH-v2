#pragma once
// Reverb de red de retardos realimentados (FDN) con 8 líneas.
//
// Las salidas de las 8 líneas de retardo se mezclan con una matriz de
// Hadamard y se reinyectan a las entradas, así los ecos se multiplican hasta
// formar una cola densa. Cada línea tiene su propia ganancia, calculada a
// partir de su retardo, para que todas decaigan al mismo ritmo: de ese modo
// el tamaño de la sala (SIZE) y la duración de la cola (DECAY) no se afectan
// entre sí.
//
// Memoria: 8 líneas x 8192 muestras x 4 bytes = 256 KB, en la SDRAM.
//
// SetSize, SetRT60 y SetDamping usan powf y expf, por eso se llaman solo
// desde loop(). Process() solo hace sumas y multiplicaciones.

#include <math.h>
#include "DaisyDuino.h"

namespace crearttech {

//====================================================================
// --- FDNREVERB ---
// 8 líneas de retardo con sus ganancias por línea. SIZE, DECAY y DAMP son independientes.
//====================================================================
class FdnReverb {
 public:
  static const int kLines  = 8;
  static const int kMaxLen = 8192;
  static const int kMask   = kMaxLen - 1;

  static constexpr int kBaseDelays[kLines] = {
      1013, 1259, 1553, 1877, 2311, 2803, 3391, 4093};

  static constexpr float kSizeMin = 0.30f;
  static constexpr float kSizeMax = 1.60f;

  // Se llama una vez en setup(). line_buffers son 8 punteros a búferes de kMaxLen floats (en la SDRAM).
  void Init(float sample_rate, float* const* line_buffers) {
    sr_ = sample_rate;
    for (int n = 0; n < kLines; n++) {
      buf_[n] = line_buffers[n];
      for (int i = 0; i < kMaxLen; i++) buf_[n][i] = 0.0f;
      write_pos_[n] = 0;
      lp_state_[n]  = 0.0f;
      gain_[n]      = 0.0f;
      delay_[n]     = (float)kBaseDelays[n];
      delay_target_[n] = delay_[n];

      lfo_phase_[n] = -kPi + (2.0f * kPi) * ((float)n / (float)kLines);
      lfo_inc_[n]   = (0.11f + 0.037f * (float)n) * 2.0f * kPi / sr_;
    }
    size01_ = 0.5f;
    rt60_   = 2.0f;
    SetDamping(8000.0f);
    RecalcDelays();
    for (int n = 0; n < kLines; n++) delay_[n] = delay_target_[n];
    RecalcGains();
  }

  // Parámetros (solo desde loop()): tamaño de la sala (0 a 1), duración de la cola (RT60, en segundos)
  void SetSize(float size01) {
    if (size01 < 0.0f) size01 = 0.0f;
    if (size01 > 1.0f) size01 = 1.0f;
    size01_ = size01;
    RecalcDelays();
    RecalcGains();
  }

  void SetRT60(float seconds) {
    if (seconds < 0.1f)  seconds = 0.1f;
    if (seconds > 20.0f) seconds = 20.0f;
    rt60_ = seconds;
    RecalcGains();
  }

  void SetDamping(float freq_hz) {
    if (freq_hz < 200.0f)   freq_hz = 200.0f;
    if (freq_hz > 18000.0f) freq_hz = 18000.0f;

    damp_coef_ = 1.0f - expf(-2.0f * kPi * freq_hz / sr_);
  }

  //====================================================================
  // --- PROCESO POR MUESTRA ---
  // Una muestra mono de entrada, dos de salida (izquierda y derecha). Lee las 8 líneas, saca la salida
  // estéreo, aplica el filtro de agudos y la ganancia de cada línea, mezcla con la matriz de Hadamard y
  // reinyecta a las líneas.
  //====================================================================
  inline void Process(float in, float* out_l, float* out_r) {
    float v[kLines];

    for (int n = 0; n < kLines; n++) {

      delay_[n] += kDelaySlew * (delay_target_[n] - delay_[n]);

      lfo_phase_[n] += lfo_inc_[n];
      if (lfo_phase_[n] > kPi) lfo_phase_[n] -= 2.0f * kPi;
      float d = delay_[n] + kModDepth * FastSin(lfo_phase_[n]);

      int   di = (int)d;
      float df = d - (float)di;
      int   i0 = (write_pos_[n] - di)     & kMask;
      int   i1 = (write_pos_[n] - di - 1) & kMask;
      v[n] = buf_[n][i0] + (buf_[n][i1] - buf_[n][i0]) * df;
    }

    *out_l = (v[0] + v[2] + v[4] + v[6]) * kOutGain;
    *out_r = (v[1] + v[3] + v[5] + v[7]) * kOutGain;

    for (int n = 0; n < kLines; n++) {
      lp_state_[n] += damp_coef_ * (v[n] - lp_state_[n]);
      v[n] = lp_state_[n] * gain_[n];
    }

    float a0 = v[0] + v[1], a1 = v[0] - v[1];
    float a2 = v[2] + v[3], a3 = v[2] - v[3];
    float a4 = v[4] + v[5], a5 = v[4] - v[5];
    float a6 = v[6] + v[7], a7 = v[6] - v[7];

    float b0 = a0 + a2, b1 = a1 + a3, b2 = a0 - a2, b3 = a1 - a3;
    float b4 = a4 + a6, b5 = a5 + a7, b6 = a4 - a6, b7 = a5 - a7;

    v[0] = b0 + b4; v[1] = b1 + b5; v[2] = b2 + b6; v[3] = b3 + b7;
    v[4] = b0 - b4; v[5] = b1 - b5; v[6] = b2 - b6; v[7] = b3 - b7;

    for (int n = 0; n < kLines; n++) {
      write_pos_[n] = (write_pos_[n] + 1) & kMask;
      buf_[n][write_pos_[n]] = in + v[n] * kHadamardNorm;
    }
  }

 private:
  //====================================================================
  // --- CÁLCULOS INTERNOS ---
  //====================================================================
  static constexpr float kPi           = 3.14159265f;
  static constexpr float kModDepth     = 6.0f;
  static constexpr float kOutGain      = 0.30f;
  static constexpr float kHadamardNorm = 0.35355339f;

  static constexpr float kDelaySlew    = 0.0002f;

  // Seno aproximado con una parábola (x entre -pi y pi); alcanza para modular un retardo.
  static inline float FastSin(float x) {
    const float kB = 4.0f / kPi;
    const float kC = -4.0f / (kPi * kPi);
    return kB * x + kC * x * (x < 0.0f ? -x : x);
  }

  // Retardo objetivo de cada línea a partir de SIZE.
  void RecalcDelays() {
    float scale = kSizeMin + size01_ * (kSizeMax - kSizeMin);
    for (int n = 0; n < kLines; n++) {
      float d = (float)kBaseDelays[n] * scale * (sr_ / 48000.0f);

      if (d < kModDepth + 2.0f)                  d = kModDepth + 2.0f;
      if (d > (float)kMaxLen - kModDepth - 2.0f) d = (float)kMaxLen - kModDepth - 2.0f;
      delay_target_[n] = d;
    }
  }

  // Ganancia de cada línea: g = 10^(-3 * T / RT60); una línea corta necesita menos ganancia.
  void RecalcGains() {
    for (int n = 0; n < kLines; n++) {
      float t_sec = delay_target_[n] / sr_;
      float g = powf(10.0f, -3.0f * t_sec / rt60_);
      if (g > 0.9999f) g = 0.9999f;
      gain_[n] = g;
    }
  }

  //====================================================================
  // --- ESTADO ---
  //====================================================================
  float  sr_        = 48000.0f;
  float  size01_    = 0.5f;
  float  rt60_      = 2.0f;
  float  damp_coef_ = 0.5f;

  float* buf_[kLines]          = {nullptr};
  int    write_pos_[kLines]    = {0};
  float  delay_[kLines]        = {0.0f};
  float  delay_target_[kLines] = {0.0f};
  float  gain_[kLines]         = {0.0f};
  float  lp_state_[kLines]     = {0.0f};
  float  lfo_phase_[kLines]    = {0.0f};
  float  lfo_inc_[kLines]      = {0.0f};
};

}
