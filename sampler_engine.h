#pragma once
// Motor del looper: guarda el audio en un búfer y lo reproduce en bucle.
//
// Un cabezal de reproducción recorre la región del loop y vuelve al inicio
// al llegar al final. El cabezal es un número decimal para poder avanzar a
// otra velocidad (cambia el tono) o hacia atrás (reversa). El overdub suma
// la entrada a lo que ya está grabado en la posición del cabezal.
//
// Process() se llama una vez por muestra desde el callback de audio.

#include <math.h>
#include <stddef.h>
#include <string.h>

namespace crearttech {

//====================================================================
// --- OVERDUBLOOPER ---
// Guarda el audio en un búfer y lo reproduce en bucle. Se llama a Process() una vez por muestra.
//====================================================================
class OverdubLooper {
public:

  // Prepara el looper con su búfer de memoria (buf, de length muestras).
  void Init(float *buf, size_t length) {
    _buffer = buf;
    _buffer_length = length;
    memset(_buffer, 0, sizeof(float) * _buffer_length);

    _inv_crossfade_samples = 1.0f / static_cast<float>(CROSSFADE_SAMPLES);

    _is_empty = true;
    _reverse = false;
    _playback_speed = 1.0f;
  }

  // Control de estado: grabar, terminar la grabación, overdub y volver al inicio del loop.
  void StartRecording() {
    _rec_head = 0;
    _play_head = 0.0f;
    _is_empty = false;
    _is_recording = true;
    _overdubbing = false;
  }

  void StopRecording() {
    _is_recording = false;
    ApplyCrossfade();
  }

  void StartOverdub() { _overdubbing = true; }
  void StopOverdub()  { _overdubbing = false; }

  void Restart() { _play_head = 0; }

  // Región del loop con muestras exactas (el final se incluye).
  void SetLoopRegion(size_t start_sample, size_t end_sample) {
    _loop_start = start_sample;
    _loop_length = end_sample - start_sample + 1;
    if (_loop_length < 1) _loop_length = 1;

    if (_play_head >= _loop_length) _play_head = 0;
  }

  // Dirección y velocidad de reproducción (1.0 normal, 2.0 sube una octava, 0.5 baja una octava).
  void SetReverse(bool reverse) { _reverse = reverse; }

  void SetPlaybackSpeed(float speed) { _playback_speed = speed; }

  // Posición del cabezal dentro de la región del loop, en muestras.
  size_t GetLoopPlayheadPosition() const { return static_cast<size_t>(_play_head); }

  //====================================================================
  // --- PROCESO POR MUESTRA ---
  // Graba, reproduce o mezcla (overdub) según el estado y devuelve la muestra de salida.
  //====================================================================
  float Process(float in) {

    if (_is_recording) {
      _buffer[_rec_head] = in;
      _rec_head++;
      if (_rec_head >= _buffer_length) {
        _rec_head = 0;
        _is_recording = false;
      }
      return in;
    }

    if (_is_empty) return 0.0f;

    float out = GetInterpolatedSample(_play_head);

    if (_overdubbing) {
      size_t index = (_loop_start + static_cast<size_t>(_play_head)) % _buffer_length;
      float mixed = SoftClip(_buffer[index] + in);
      _buffer[index] = mixed;
      out = mixed;
    }

    if (_reverse) {
      _play_head -= _playback_speed;
      if (_play_head < 0.0f) _play_head += _loop_length;
    } else {
      _play_head += _playback_speed;
      if (_play_head >= _loop_length) _play_head -= _loop_length;
    }

    return out;
  }

private:
  //====================================================================
  // --- FUNCIONES INTERNAS ---
  //====================================================================
  static const size_t CROSSFADE_SAMPLES = 128;

  // Mezcla el final del loop sobre su inicio para que el salto de vuelta no suene como un click.
  void ApplyCrossfade() {
    if (_loop_length < CROSSFADE_SAMPLES * 2) return;

    for (size_t i = 0; i < CROSSFADE_SAMPLES; i++) {
      float fade = static_cast<float>(i) * _inv_crossfade_samples;
      size_t start_idx = (_loop_start + i) % _buffer_length;
      size_t end_idx = (_loop_start + _loop_length - CROSSFADE_SAMPLES + i) % _buffer_length;
      _buffer[start_idx] = _buffer[start_idx] * fade + _buffer[end_idx] * (1.0f - fade);
    }
  }

  // Recorte suave con tanh para que las capas sumadas no saturen de golpe.
  float SoftClip(float x) {
    return tanhf(x * 0.7f) / 0.7f;
  }

  // Lee el búfer en una posición decimal, interpolando entre las dos muestras vecinas.
  float GetInterpolatedSample(float position) {
    size_t idx0 = static_cast<size_t>(position);
    size_t idx1 = (idx0 + 1) % _loop_length;
    float frac = position - static_cast<float>(idx0);
    size_t actual_idx0 = (_loop_start + idx0) % _buffer_length;
    size_t actual_idx1 = (_loop_start + idx1) % _buffer_length;
    return _buffer[actual_idx0] * (1.0f - frac) + _buffer[actual_idx1] * frac;
  }

  //====================================================================
  // --- ESTADO INTERNO ---
  //====================================================================
  float* _buffer;
  size_t _buffer_length;
  size_t _loop_start;
  size_t _loop_length;

  float  _play_head;
  size_t _rec_head;

  bool _is_empty;
  bool _is_recording;
  bool _reverse;
  bool _overdubbing;

  float _playback_speed;
  float _inv_crossfade_samples = 0.0f;
};

}
