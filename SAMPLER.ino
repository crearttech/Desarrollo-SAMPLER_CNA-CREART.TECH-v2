/*
 * SAMPLER CNA - Programa principal
 * Crearttech · Centro Nacional de las Artes Delia Zapata Olivella
 *
 * Looper de audio en tiempo real para la Daisy Seed. Graba hasta 10 s,
 * reproduce en bucle, sobregraba capas (overdub), reproduce en reversa y
 * aplica pitch, filtros, delay y reverb. Una pantalla TFT muestra la forma
 * de onda, el estado y el valor de los encoders.
 *
 * Archivos:
 *   SAMPLER.ino        pines, pantalla, controles y audio
 *   sampler_engine.h   motor del looper (grabar, reproducir, overdub)
 *   sampler_reverb.h   reverb de 8 líneas con tamaño de sala ajustable
 *
 * Controles:
 *   REC (D18)     mantener para grabar; con el loop sonando, mantener para overdub
 *   PLAY (D17)    play / pausa; doble pulsación borra el loop
 *   REVERSA (D16) activa y desactiva la reproducción en reversa
 *   RESET (D23)   doble pulsación borra los efectos; mantener 1 s borra también
 *                 la ganancia y la región del loop
 *   ENC1          pitch, pasa altos y pasa bajos
 *   ENC2          reverb: mezcla, size, decay y damp
 *   ENC3          delay: tiempo, feedback y mezcla
 *   ENC4          ganancia, inicio, final y movimiento de la región del loop
 *   El pulsador de cada encoder cambia el parámetro que controla.
 *
 * Cadena de audio al reproducir:
 *   looper, pasa altos, pasa bajos, delay, reverb, ganancia, limitador, salida L/R
 *
 * El programa tiene tres partes que corren por separado:
 *   AudioCallback()  la llama la Daisy cada 48 muestras (1 ms) para grabar y
 *                    procesar el audio; tiene que ser rápida
 *   encoderN_isr()   interrupciones que cuentan los pasos de cada encoder
 *   loop()           lee botones y encoders, calcula los parámetros y dibuja
 * Las variables que comparten se declaran volatile.
 */

#include <DaisyDuino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <math.h>
#include "sampler_engine.h"
#include "sampler_reverb.h"

using namespace daisy;

//====================================================================
// --- PANTALLA ---
// Tamaño de la pantalla TFT y lienzo en RAM: todo se dibuja primero en el lienzo y
// después se envía completo a la pantalla, así la imagen no parpadea.
//====================================================================
#define SCREEN_WIDTH 160
#define SCREEN_HEIGHT 128

GFXcanvas16* canvas = NULL;

//====================================================================
// --- CONFIGURACIÓN DE PINES ---
// Pantalla (SPI), botones y encoders. Los botones usan pull-up: sueltos leen HIGH y
// pulsados LOW. Cada encoder usa CLK (dispara la interrupción), DT (sentido) y SW (pulsador).
//====================================================================
#define TFT_CS D7
#define TFT_DC D6
#define TFT_RST D29

class PantallaST7735 : public Adafruit_ST7735 {
public:
  using Adafruit_ST7735::Adafruit_ST7735;
  void quitarDesplazamiento() { setColRowStart(0, 0); }
};
PantallaST7735 tft(TFT_CS, TFT_DC, TFT_RST);

#define REC_BUTTON_PIN D18
#define PLAY_BUTTON_PIN D17
#define REVERSE_BUTTON_PIN D16
#define RESET_BUTTON_PIN D23

#define ENC1_CLK_PIN D0
#define ENC1_DT_PIN D1
#define ENC1_SW_PIN D9

#define ENC2_CLK_PIN D3
#define ENC2_DT_PIN D4
#define ENC2_SW_PIN D5

#define ENC3_CLK_PIN D13
#define ENC3_DT_PIN D14
#define ENC3_SW_PIN D25

#define ENC4_CLK_PIN D20
#define ENC4_DT_PIN D21
#define ENC4_SW_PIN D24

const int PITCH_SENSITIVITY = 4;

//====================================================================
// --- BÚFERES DE AUDIO ---
// El audio del loop (10 s a 48 kHz) y una copia para dibujar la onda. Son grandes, por eso
// van en la SDRAM externa de la Daisy.
//====================================================================
static const uint32_t kBufferLengthSec = 10;
static const uint32_t kSampleRate = 48000;
static const size_t kBufferLengthSamples = kBufferLengthSec * kSampleRate;

static float DSY_SDRAM_BSS buffer[kBufferLengthSamples] __attribute__((aligned(32)));
static float DSY_SDRAM_BSS waveform_source_buffer[kBufferLengthSamples] __attribute__((aligned(32)));

//====================================================================
// --- OBJETOS DE AUDIO Y ESTADOS GLOBALES ---
// Looper, filtros, reverb y delay, y los estados y modos de cada encoder.
//====================================================================
static crearttech::OverdubLooper looper;

static daisysp::Svf* g_highpass_filter;
static daisysp::Svf* g_lowpass_filter;

static float DSY_SDRAM_BSS fdn_lines[crearttech::FdnReverb::kLines]
                                   [crearttech::FdnReverb::kMaxLen]
                                   __attribute__((aligned(32)));
static float* fdn_ptrs[crearttech::FdnReverb::kLines];
static crearttech::FdnReverb reverb_fdn;

static daisysp::DelayLine<float, 48000> DSY_SDRAM_BSS delay_effect;

enum LooperState { STOPPED, RECORDING, PLAYING, OVERDUB, PAUSED };
LooperState looper_state = STOPPED;

enum Knob2Mode { REVERB, SIZE, DECAY, DAMP };
Knob2Mode knob2_mode = REVERB;

enum Knob3Mode { TIME, DELAY, MIX };
Knob3Mode knob3_mode = TIME;

enum Enc1Mode { PITCH, HIGHPASS, LOWPASS };
Enc1Mode enc1_mode = PITCH;

int knob1_pitch_val = 0;
int knob1_hpass_val = 0;
int knob1_lpass_val = 100;

enum Enc4Mode {
    ENC4_MODE_START_POINT,
    ENC4_MODE_END_POINT,
    ENC4_MODE_MOVE,
    ENC4_MODE_GAIN
};
Enc4Mode enc4_mode = ENC4_MODE_GAIN;

static float g_gain = 1.0f;

//====================================================================
// --- ESTADO DE BOTONES Y ENCODERS ---
// Lectura anterior de cada botón (para detectar la pulsación), tiempos de doble
// pulsación y contadores que llenan las interrupciones de los encoders.
//====================================================================
bool last_rec_button_state = HIGH, last_play_button_state = HIGH, last_reverse_button_state = HIGH;
bool last_reset_button_state = HIGH;
bool last_enc1_sw_state = HIGH, last_enc2_sw_state = HIGH, last_enc3_sw_state = HIGH, last_enc4_sw_state = HIGH;

const unsigned long DOUBLE_PRESS_TIME_MS = 500;
const unsigned long LONG_PRESS_TIME_MS = 1000;
unsigned long lastPlayPressTime = 0;
int playPressCount = 0;
unsigned long play_button_press_time = 0;
bool play_button_long_press_actioned = false;
unsigned long last_reset_press_time = 0;
int reset_press_count = 0;
unsigned long reset_press_start_time = 0;
bool reset_long_fired = false;

volatile int enc1_counter = 0, enc2_counter = 0, enc3_counter = 0;
volatile int enc4_counter = 0;
static int last_e4 = 0;

volatile int knob2_reverb_val = 0, knob2_size_val = 50, knob2_decay_val = 75;
volatile int knob2_damp_val = 45;
volatile int knob3_time_val = 0, knob3_feedback_val = 0, knob3_mix_val = 0;

//====================================================================
// --- DELAY ---
// Parámetros que calcula loop() y usa el audio. El tiempo se suaviza para evitar chasquidos.
//====================================================================
volatile float delay_time_samples = 48000.0f / 1000.0f * 10.0f;
volatile float delay_feedback = 0.0f;
volatile float delay_mix = 0.0f;

static float delay_time_smooth = 48000.0f / 1000.0f * 10.0f;

//====================================================================
// --- REVERB: OBJETIVOS Y ESTADO INTERNO ---
// loop() calcula los objetivos a partir de los knobs y el audio se acerca a ellos poco a poco.
// DECAY va de 0,4 a 8,4 s y DAMP de 18 kHz a 1 kHz, ambos con curva exponencial.
//====================================================================
volatile float g_reverb_dry_target = 1.0f;
volatile float g_reverb_wet_target = 0.0f;
volatile float g_reverb_rt60    = 2.0f;
volatile float g_reverb_damp_hz = 5000.0f;

static float reverb_dry_smooth = 1.0f;
static float reverb_wet_smooth = 0.0f;

static const float kRt60Min = 0.4f;
static const float kRt60Max = 8.4f;
static const float kDampBright = 18000.0f;
static const float kDampDark   =  1000.0f;

static float reverb_tail_env = 0.0f;
static const float kReverbEnvDecay  = 0.9999f;
static const float kReverbTailFloor = 1.0e-5f;
static const float kReverbMixFloor  = 1.0e-4f;

//====================================================================
// --- ESTADO DEL LOOP Y DE LA PANTALLA ---
// Dirección de reproducción, grabación en curso, región del loop y datos de la onda en pantalla.
//====================================================================
bool reverse_mode = false;
static const size_t kFadeInSamples = 1440;
volatile size_t rec_fade_counter = 0;
volatile size_t record_counter = 0;
volatile size_t recorded_samples = 0;
bool waveform_ready = false;
float waveform_scale = 1.0f;

volatile size_t loop_start_sample = 0;
volatile size_t loop_end_sample = 0;

struct WaveformPixel { float min; float max; };
const int DISPLAY_W = (SCREEN_WIDTH - 5 * 2);
WaveformPixel displayWaveform[160];

struct Star { float x, y, z; float speed; };
#define MAX_STARS 100
Star stars[MAX_STARS];

//====================================================================
// --- COLORES Y POSICIONES ---
// Colores en RGB565 y posición de cada zona de la pantalla.
//====================================================================
#define COLOR(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))
const uint16_t C_TEXT_LIGHT = COLOR(200, 220, 255);
const uint16_t C_TEXT_DARK = COLOR(100, 110, 130);
const uint16_t C_ACCENT_CYAN = COLOR(0, 255, 255);
const uint16_t C_ACCENT_MAGENTA = COLOR(255, 0, 255);
const uint16_t C_ACCENT_ORANGE = COLOR(0, 165, 255);
const uint16_t C_STATE_REC = COLOR(0, 0, 255);

const int STATUS_Y = 10;
const int WAVEFORM_X = 5, WAVEFORM_Y = 25, WAVEFORM_H = 45;
const int KNOBS_Y = 85;

//====================================================================
// --- ENCODERS: INTERRUPCIONES ---
// Se ejecutan al cambiar el pin CLK. Ignoran los cambios a menos de 3 ms (rebote) y suman o
// restan un paso según el nivel de DT.
//====================================================================
volatile unsigned long last_isr_time_1 = 0;
volatile unsigned long last_isr_time_2 = 0;
volatile unsigned long last_isr_time_3 = 0;
volatile unsigned long last_isr_time_4 = 0;

void encoder1_isr() {
  if (micros() - last_isr_time_1 < 3000) return;
  last_isr_time_1 = micros();
  if (digitalRead(ENC1_DT_PIN) == digitalRead(ENC1_CLK_PIN)) {
    enc1_counter++;
  } else {
    enc1_counter--;
  }
}

void encoder2_isr() {
  if (micros() - last_isr_time_2 < 3000) return;
  last_isr_time_2 = micros();
  if (digitalRead(ENC2_DT_PIN) == digitalRead(ENC2_CLK_PIN)) {
    enc2_counter++;
  } else {
    enc2_counter--;
  }
}

void encoder3_isr() {
  if (micros() - last_isr_time_3 < 3000) return;
  last_isr_time_3 = micros();
  if (digitalRead(ENC3_DT_PIN) == digitalRead(ENC3_CLK_PIN)) {
    enc3_counter++;
  } else {
    enc3_counter--;
  }
}

void encoder4_isr() {
  if (micros() - last_isr_time_4 < 3000) return;
  last_isr_time_4 = micros();
  if (digitalRead(ENC4_DT_PIN) == digitalRead(ENC4_CLK_PIN)) {
    enc4_counter++;
  } else {
    enc4_counter--;
  }
}

//====================================================================
// --- PANTALLA: FUNCIONES DE DIBUJO ---
//====================================================================
// Reduce el audio grabado a una columna (mínimo y máximo) por píxel de ancho.
void generarOnda(WaveformPixel* displayBuf, int displayLen, float* audioBuf, size_t audioLen) {
  if (audioLen == 0 || displayLen <= 0) return;
  int samples_per_pixel = (int)(audioLen / displayLen);
  if (samples_per_pixel < 4) samples_per_pixel = 4;

  for (int i = 0; i < displayLen; i++) {
    size_t chunk_start = (size_t)i * samples_per_pixel;
    size_t chunk_end = chunk_start + samples_per_pixel;
    if (chunk_start >= audioLen) {
      displayBuf[i].min = 0.0f;
      displayBuf[i].max = 0.0f;
      continue;
    }
    if (chunk_end > audioLen) chunk_end = audioLen;
    float sum_sq = 0.0f;
    float min_val = 1.0f;
    float max_val = -1.0f;
    for (size_t j = chunk_start; j < chunk_end; j++) {
      float s = audioBuf[j];
      sum_sq += s * s;
      if (s < min_val) min_val = s;
      if (s > max_val) max_val = s;
    }
    float rms = sqrtf(sum_sq / (float)(chunk_end - chunk_start));

    const float blend = 0.65f;
    displayBuf[i].max = (max_val * blend) + (rms * (1.0f - blend));
    displayBuf[i].min = (min_val * blend) - (rms * (1.0f - blend));
  }
}

// Fondo: degradado oscuro con estrellas que se acercan a la cámara.
void drawBackground() {
  uint8_t r_start = 5, g_start = 10, b_start = 25;
  for (int y = 0; y < SCREEN_HEIGHT; y++) {
    uint8_t b = b_start - (y * b_start / SCREEN_HEIGHT);
    uint8_t g = g_start - (y * g_start / SCREEN_HEIGHT);
    uint8_t r = r_start - (y * r_start / SCREEN_HEIGHT);
    canvas->drawFastHLine(0, y, SCREEN_WIDTH, COLOR(r, g, b));
  }
  float center_x = SCREEN_WIDTH / 2.0;
  float center_y = SCREEN_HEIGHT / 2.0;
  for (int i = 0; i < MAX_STARS; i++) {

    stars[i].z -= stars[i].speed * 0.2;
    if (stars[i].z <= 0.5) {
      stars[i].x = random(-SCREEN_WIDTH / 2, SCREEN_WIDTH / 2);
      stars[i].y = random(-SCREEN_HEIGHT / 2, SCREEN_HEIGHT / 2);
      stars[i].z = random(8, 15);
      stars[i].speed = (15.0 - stars[i].z) * 0.3 + 0.5;
    }

    float perspective_scale = 1.0 / stars[i].z;
    int display_x = (int)(center_x + stars[i].x * perspective_scale * 5);
    int display_y = (int)(center_y + stars[i].y * perspective_scale * 5);

    uint16_t star_color = COLOR(200, 200, 200);
    int star_size = 1;
    if (stars[i].z < 3) {
      star_color = COLOR(255, 255, 255);
      star_size = 2;
    } else if (stars[i].z < 6) {
      star_color = COLOR(220, 220, 215);
    }
    if (display_x >= 0 && display_x < SCREEN_WIDTH && display_y >= 0 && display_y < SCREEN_HEIGHT) {
      if (star_size > 1) {
        canvas->fillRect(display_x, display_y, star_size, star_size, star_color);
      } else {
        canvas->drawPixel(display_x, display_y, star_color);
      }
    }
  }
}

// Pantalla de arranque: título y barra de carga (progress de 0 a 1).
void drawSplashScreen(float progress) {
  drawBackground();
  const char* title = "SAMPLER";
  int16_t x1, y1;
  uint16_t w, h;
  canvas->setFont(NULL);
  canvas->setTextSize(3);
  canvas->getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  int16_t cursor_x = (SCREEN_WIDTH - w) / 2;
  int bar_height = 8;
  int spacing = 12;
  int total_content_height = h + spacing + bar_height;
  int16_t cursor_y = (SCREEN_HEIGHT - total_content_height) / 2;
  canvas->setCursor(cursor_x, cursor_y);
  canvas->setTextColor(C_ACCENT_CYAN);
  canvas->print(title);
  int bar_width = 100;
  int bar_x = (SCREEN_WIDTH - bar_width) / 2;
  int bar_y = cursor_y + h + spacing;
  canvas->drawRect(bar_x, bar_y, bar_width, bar_height, C_TEXT_LIGHT);
  int progress_width = (int)(progress * (float)(bar_width - 4));
  if (progress_width > 0) {
    canvas->fillRect(bar_x + 2, bar_y + 2, progress_width, bar_height - 4, C_ACCENT_CYAN);
  }
}

// Barra superior: ícono y nombre del estado y flechas de dirección de reproducción.
void drawStatusPanel() {
  const char* state_text;
  const char* state_icon;
  uint16_t state_color;
  switch (looper_state) {
    case RECORDING: state_text = "REC"; state_icon = "●"; state_color = C_STATE_REC; break;
    case PLAYING: state_text = "PLAY"; state_icon = "►"; state_color = COLOR(0, 255, 0); break;
    case OVERDUB: state_text = "OVERDUB"; state_icon = "+"; state_color = C_STATE_REC; break;
    case PAUSED: state_text = "PAUSE"; state_icon = "||"; state_color = COLOR(255, 0, 0); break;
    default: state_text = "STOP"; state_icon = "■"; state_color = C_ACCENT_ORANGE; break;
  }
  canvas->setCursor(10, STATUS_Y);
  switch (looper_state) {
    case RECORDING: canvas->fillCircle(10 + 6, STATUS_Y + 8, 6, state_color); break;
    case PLAYING: canvas->fillTriangle(10, STATUS_Y + 2, 10, STATUS_Y + 14, 10 + 12, STATUS_Y + 8, state_color); break;
    case PAUSED: canvas->setTextSize(2); canvas->setTextColor(state_color); canvas->print(state_icon); break;
    case OVERDUB: canvas->setTextSize(2); canvas->setTextColor(state_color); canvas->print(state_icon); break;
    default: canvas->fillRect(10, STATUS_Y + 2, 12, 12, state_color); break;
  }
  canvas->setTextSize(1);
  canvas->setCursor(30, STATUS_Y + 4);
  canvas->setTextColor(state_color);
  canvas->print(state_text);

  if (reverse_mode) {
    canvas->fillTriangle(SCREEN_WIDTH - 10, STATUS_Y + 2, SCREEN_WIDTH - 10, STATUS_Y + 14, SCREEN_WIDTH - 10 - 8, STATUS_Y + 8, C_ACCENT_MAGENTA);
    canvas->fillTriangle(SCREEN_WIDTH - 10 - 6, STATUS_Y + 2, SCREEN_WIDTH - 10 - 6, STATUS_Y + 14, SCREEN_WIDTH - 10 - 6 - 8, STATUS_Y + 8, C_ACCENT_MAGENTA);
  } else {
    canvas->fillTriangle(SCREEN_WIDTH - 10 - 8, STATUS_Y + 2, SCREEN_WIDTH - 10 - 8, STATUS_Y + 14, SCREEN_WIDTH - 10, STATUS_Y + 8, C_ACCENT_CYAN);
    canvas->fillTriangle(SCREEN_WIDTH - 10 - 8 - 6, STATUS_Y + 2, SCREEN_WIDTH - 10 - 8 - 6, STATUS_Y + 14, SCREEN_WIDTH - 10 - 6, STATUS_Y + 8, C_ACCENT_CYAN);
  }
}

// Forma de onda con la región del loop (cian dentro, gris fuera), sus marcas y el cabezal (magenta).
void drawWaveform() {
  if (looper_state == STOPPED && !waveform_ready) return;
  int displayLen = DISPLAY_W;
  int draw_limit_x = displayLen;

  if (looper_state == RECORDING) {
    noInterrupts(); size_t local_count = record_counter; interrupts();
    draw_limit_x = (int)((float)local_count / (float)kBufferLengthSamples * displayLen);
    draw_limit_x = constrain(draw_limit_x, 0, displayLen);
  }
  if (waveform_ready) {
    int midY = WAVEFORM_Y + (WAVEFORM_H / 2);

    for (int x = 0; x < draw_limit_x; x++) {
      float min_val = displayWaveform[x].min * g_gain;
      float max_val = displayWaveform[x].max * g_gain;
      int y_top = midY - (int)(max_val * waveform_scale);
      int y_bottom = midY - (int)(min_val * waveform_scale);
      if (y_top > y_bottom) { int tmp = y_top; y_top = y_bottom; y_bottom = tmp; }
      y_top = constrain(y_top, WAVEFORM_Y, WAVEFORM_Y + WAVEFORM_H - 1);
      y_bottom = constrain(y_bottom, WAVEFORM_Y, WAVEFORM_Y + WAVEFORM_H - 1);
      int height = y_bottom - y_top;
      if (height < 1) height = 1;
      uint16_t waveform_color = C_ACCENT_CYAN;
      if (recorded_samples > 0) {
        int loop_start_screen_x = (int)((float)loop_start_sample / recorded_samples * displayLen);
        int loop_end_screen_x = (int)((float)loop_end_sample / recorded_samples * displayLen);
        if (x < loop_start_screen_x || x > loop_end_screen_x) waveform_color = C_TEXT_DARK;
      }
      canvas->drawFastVLine(WAVEFORM_X + x, y_top, height, waveform_color);
    }

    bool should_draw_playhead = (looper_state == PLAYING || looper_state == OVERDUB || looper_state == PAUSED);
    if (should_draw_playhead && recorded_samples > 0) {
      size_t absolute_playhead_pos;
      noInterrupts(); size_t relative_playhead = looper.GetLoopPlayheadPosition(); interrupts();
      absolute_playhead_pos = loop_start_sample + relative_playhead;
      if (absolute_playhead_pos >= recorded_samples) absolute_playhead_pos = recorded_samples - 1;
      float progress = (float)absolute_playhead_pos / (float)recorded_samples;
      int play_x = WAVEFORM_X + (int)(progress * displayLen);
      play_x = constrain(play_x, WAVEFORM_X, WAVEFORM_X + displayLen - 1);
      canvas->drawFastVLine(play_x, WAVEFORM_Y, WAVEFORM_H, C_ACCENT_MAGENTA);
    }

    if (recorded_samples > 0) {
      int start_x = WAVEFORM_X + (int)((float)loop_start_sample / recorded_samples * displayLen);
      int end_x = WAVEFORM_X + (int)((float)loop_end_sample / recorded_samples * displayLen);
      start_x = constrain(start_x, WAVEFORM_X, WAVEFORM_X + displayLen - 1);
      end_x = constrain(end_x, WAVEFORM_X, WAVEFORM_X + displayLen - 1);
      canvas->drawFastVLine(start_x, WAVEFORM_Y, WAVEFORM_H, C_TEXT_LIGHT);
      canvas->drawFastVLine(end_x, WAVEFORM_Y, WAVEFORM_H, C_TEXT_LIGHT);
    }
  }
}

// Arco grueso entre dos ángulos en grados (0° a la derecha, sentido horario).
void drawArc(Adafruit_GFX& gfx, int16_t cx, int16_t cy, int16_t radius, uint8_t thickness, int16_t start_angle, int16_t end_angle, uint16_t color) {
  if (end_angle < start_angle) { end_angle += 360; }
  for (int r = radius; r < radius + thickness; r++) {
    for (int i = start_angle; i <= end_angle; i++) {
        gfx.drawPixel(cx + r * cos(i * M_PI / 180.0), cy + r * sin(i * M_PI / 180.0), color);
    }
  }
}

// Knob circular: arco de valor (de 135° a 405°) y nombre debajo.
void drawCircularKnob(Adafruit_GFX& gfx, int16_t cx, int16_t cy, const char* label, int16_t arc_start_angle, int16_t arc_end_angle, uint16_t fgColor, uint16_t textColor) {
  const int16_t radius = 18; const int16_t outline_radius = 12;
  gfx.drawCircle(cx, cy, outline_radius, COLOR(255, 255, 255));
  drawArc(gfx, cx, cy, radius, 3, arc_start_angle, arc_end_angle, fgColor);
  gfx.setFont(NULL); gfx.setTextSize(1); gfx.setTextColor(textColor);
  int16_t x1, y1; uint16_t w, h;
  gfx.setTextSize(1); gfx.setTextColor(C_TEXT_LIGHT);
  gfx.getTextBounds(label, 0, 0, &x1, &y1, &w, &h);
  gfx.setCursor(cx - w / 2, cy + radius + 5); gfx.print(label);
}

// Fila de tres knobs: ENC1 (magenta), ENC3 (naranja) y ENC2 (cian), cada uno con su valor en unidades reales.
void drawKnobsPanel() {

  const char* knob1_label; int16_t knob1_arc_start, knob1_arc_end;
  if (enc1_mode == PITCH) {

    knob1_label = "PITCH"; int16_t center_angle = 270; int16_t max_sweep = 135;
    float pitch_value = (float)(enc1_counter / PITCH_SENSITIVITY);
    if (pitch_value >= 0) { knob1_arc_start = center_angle; knob1_arc_end = center_angle + (int16_t)((pitch_value / 6.0f) * max_sweep); }
    else { knob1_arc_start = center_angle - (int16_t)((-pitch_value / 6.0f) * max_sweep); knob1_arc_end = center_angle; }
  } else if (enc1_mode == HIGHPASS) {
    knob1_label = "HPASS"; knob1_arc_start = 135; knob1_arc_end = 135 + (int16_t)((float)enc1_counter / 100.0f * 270);
  } else {
    knob1_label = "LPASS"; knob1_arc_start = 135; knob1_arc_end = 135 + (int16_t)((float)enc1_counter / 100.0f * 270);
  }
  drawCircularKnob(*canvas, 30, KNOBS_Y + 10, knob1_label, knob1_arc_start, knob1_arc_end, C_ACCENT_MAGENTA, C_TEXT_LIGHT);

  char knob1_val_str[12];
  if (enc1_mode == PITCH) {

    int raw = enc1_counter * 10;
    int st_x10 = (raw >= 0) ? (raw / PITCH_SENSITIVITY) : -((-raw) / PITCH_SENSITIVITY);
    int abs_st = (st_x10 < 0) ? -st_x10 : st_x10;
    char sign = (st_x10 >= 0) ? '+' : '-';
    snprintf(knob1_val_str, sizeof(knob1_val_str), "%c%d.%dst", sign, abs_st / 10, abs_st % 10);
  } else {

    int freq;
    if (enc1_mode == HIGHPASS) freq = (int)(20.0f * powf(500.0f, (float)knob1_hpass_val / 100.0f));
    else freq = (int)(200.0f * powf(100.0f, (float)knob1_lpass_val / 100.0f));
    if (freq >= 10000) {
      snprintf(knob1_val_str, sizeof(knob1_val_str), "%dk", freq / 1000);
    } else if (freq >= 1000) {
      int kWhole = freq / 1000;
      int kFrac = (freq % 1000) / 100;
      snprintf(knob1_val_str, sizeof(knob1_val_str), "%d.%dk", kWhole, kFrac);
    } else {
      snprintf(knob1_val_str, sizeof(knob1_val_str), "%dHz", freq);
    }
  }
  canvas->setFont(NULL); canvas->setTextSize(1); canvas->setTextColor(C_ACCENT_MAGENTA);
  int16_t vx1, vy1; uint16_t vw, vh;
  canvas->getTextBounds(knob1_val_str, 0, 0, &vx1, &vy1, &vw, &vh);
  canvas->setCursor(30 - vw / 2, KNOBS_Y - 18); canvas->print(knob1_val_str);

  const char* knob3_label; int knob3_val_to_display;
  switch (knob3_mode) {
    case DELAY: knob3_label = "FBACK"; knob3_val_to_display = knob3_feedback_val; break;
    case MIX: knob3_label = "MIX"; knob3_val_to_display = knob3_mix_val; break;
    default: knob3_label = "DELAY"; knob3_val_to_display = knob3_time_val; break;
  }
  drawCircularKnob(*canvas, 80, KNOBS_Y + 10, knob3_label, 135, 135 + (int16_t)((float)knob3_val_to_display / 100.0f * 270), C_ACCENT_ORANGE, C_TEXT_LIGHT);

  char knob3_val_str[12];
  if (knob3_mode == TIME) {

    int ms = (int)(10.0f * powf(100.0f, (float)knob3_time_val / 100.0f));
    if (ms >= 1000) snprintf(knob3_val_str, sizeof(knob3_val_str), "%d.%ds", ms / 1000, (ms % 1000) / 100);
    else snprintf(knob3_val_str, sizeof(knob3_val_str), "%dms", ms);
  } else if (knob3_mode == DELAY) {
    snprintf(knob3_val_str, sizeof(knob3_val_str), "%d%%", knob3_feedback_val);
  } else {
    snprintf(knob3_val_str, sizeof(knob3_val_str), "%d%%", knob3_mix_val);
  }
  canvas->setTextColor(C_ACCENT_ORANGE);
  canvas->getTextBounds(knob3_val_str, 0, 0, &vx1, &vy1, &vw, &vh);
  canvas->setCursor(80 - vw / 2, KNOBS_Y - 18); canvas->print(knob3_val_str);

  const char* knob2_label;
  switch (knob2_mode) {
    case SIZE:     knob2_label = "SIZE";  break;
    case DECAY:    knob2_label = "DECAY"; break;
    case DAMP:     knob2_label = "DAMP";  break;
    default:       knob2_label = "REVERB"; break;
  }
  drawCircularKnob(*canvas, 130, KNOBS_Y + 10, knob2_label, 135, 135 + (int16_t)((float)enc2_counter / 100.0f * 270), C_ACCENT_CYAN, C_TEXT_LIGHT);

  char knob2_val_str[10];
  if (knob2_mode == DECAY) {
    int rt_x10 = (int)(g_reverb_rt60 * 10.0f);
    snprintf(knob2_val_str, sizeof(knob2_val_str), "%d.%ds", rt_x10 / 10, rt_x10 % 10);
  } else if (knob2_mode == DAMP) {
    int hz = (int)g_reverb_damp_hz;
    if (hz >= 10000)     snprintf(knob2_val_str, sizeof(knob2_val_str), "%dk", hz / 1000);
    else if (hz >= 1000) snprintf(knob2_val_str, sizeof(knob2_val_str), "%d.%dk", hz / 1000, (hz % 1000) / 100);
    else                 snprintf(knob2_val_str, sizeof(knob2_val_str), "%dHz", hz);
  } else {
    snprintf(knob2_val_str, sizeof(knob2_val_str), "%d%%", enc2_counter);
  }
  canvas->setTextColor(C_ACCENT_CYAN);
  canvas->getTextBounds(knob2_val_str, 0, 0, &vx1, &vy1, &vw, &vh);
  canvas->setCursor(130 - vw / 2, KNOBS_Y - 18); canvas->print(knob2_val_str);
}

// Compone la pantalla completa en el lienzo. Lo que se dibuja después queda encima.
void drawScreen() {
  drawBackground();
  drawStatusPanel();
  drawWaveform();
  drawKnobsPanel();

  int current_y = STATUS_Y + 4; int text_x = SCREEN_WIDTH - 30;
  canvas->setFont(NULL); canvas->setTextSize(1); canvas->setTextWrap(false);
  const char* enc4_mode_text; uint16_t enc4_mode_color = C_ACCENT_MAGENTA;
  switch (enc4_mode) {
    case ENC4_MODE_START_POINT: enc4_mode_text = "S.PT"; break;
    case ENC4_MODE_END_POINT: enc4_mode_text = "E.PT"; break;
    case ENC4_MODE_MOVE: enc4_mode_text = "MOVE"; break;
    case ENC4_MODE_GAIN: enc4_mode_text = "GAIN"; break;
    default: enc4_mode_text = ""; break;
  }
  int16_t x1, y1; uint16_t w, h;
  canvas->getTextBounds(enc4_mode_text, 0, 0, &x1, &y1, &w, &h);
  canvas->setCursor(text_x - w, current_y); canvas->setTextColor(enc4_mode_color); canvas->print(enc4_mode_text);
}

//====================================================================
// --- REVERB: OBJETIVOS Y COLA ---
//====================================================================
// Traduce los knobs de ENC2 a los objetivos de la reverb: REVERB (mezcla con ley de potencia constante),
// DECAY (duración de la cola), SIZE (tamaño de la sala) y DAMP (absorción de agudos).
static void UpdateReverbTargets() {
  float theta = (float)knob2_reverb_val * 0.01f * 1.57079633f;
  g_reverb_dry_target = cosf(theta);
  g_reverb_wet_target = sinf(theta);

  static int last_size = -1, last_decay = -1, last_damp = -1;

  if (knob2_size_val != last_size) {
    reverb_fdn.SetSize((float)knob2_size_val * 0.01f);
    last_size = knob2_size_val;
  }
  if (knob2_decay_val != last_decay) {
    g_reverb_rt60 = kRt60Min * powf(kRt60Max / kRt60Min, (float)knob2_decay_val * 0.01f);
    reverb_fdn.SetRT60(g_reverb_rt60);
    last_decay = knob2_decay_val;
  }
  if (knob2_damp_val != last_damp) {
    g_reverb_damp_hz = kDampBright * powf(kDampDark / kDampBright, (float)knob2_damp_val * 0.01f);
    reverb_fdn.SetDamping(g_reverb_damp_hz);
    last_damp = knob2_damp_val;
  }
}

// Lleva las ganancias suavizadas a su objetivo sin rampa. Solo con el audio parado.
static void SnapReverbSmoothing() {
  reverb_dry_smooth = g_reverb_dry_target;
  reverb_wet_smooth = g_reverb_wet_target;
}

// Con el looper parado o grabando, pasa silencio por la reverb para que su cola se apague sola.
static inline void FlushReverbTail(size_t size) {
  if (reverb_tail_env <= kReverbTailFloor) {
    reverb_tail_env = 0.0f;
    return;
  }
  float rl = 0.0f, rr = 0.0f;
  for (size_t i = 0; i < size; i++) {
    reverb_fdn.Process(0.0f, &rl, &rr);
    float mag = fabsf(rl) + fabsf(rr);
    reverb_tail_env = (mag > reverb_tail_env) ? mag : (reverb_tail_env * kReverbEnvDecay);
  }
}

//====================================================================
// --- AUDIO ---
// La Daisy llama a AudioCallback() cada 48 muestras (1 ms): in[0] es la entrada que se graba y
// out[0] y out[1] son las salidas L y R.
// STOPPED y PAUSED: salida en silencio.  RECORDING y OVERDUB: graba la entrada.  PLAYING: reproduce con efectos.
//====================================================================
void AudioCallback(float** in, float** out, size_t size) {

  // STOPPED y PAUSED: salida en silencio y se apagan las colas de delay y reverb.
  if (looper_state == PAUSED || looper_state == STOPPED) {
    for (size_t i = 0; i < size; i++) {
      out[0][i] = out[1][i] = 0.0f;
    }
    delay_effect.Write(0.0f);
    delay_time_smooth = delay_time_samples;
    SnapReverbSmoothing();
    FlushReverbTail(size);
    return;
  }

  // RECORDING y OVERDUB: se graba la entrada en el looper y la salida queda en silencio.
  if (looper_state == RECORDING || looper_state == OVERDUB) {
    for (size_t i = 0; i < size; i++) {
      float input_signal = in[0][i];

      if (rec_fade_counter < kFadeInSamples) {
        input_signal *= (float)rec_fade_counter / (float)kFadeInSamples;
        rec_fade_counter++;
      }
      looper.Process(input_signal);

      if (looper_state == RECORDING) {
        size_t pos = record_counter;
        if (pos < kBufferLengthSamples) {
          waveform_source_buffer[pos] = input_signal;
          record_counter++;
          if (record_counter > kBufferLengthSamples) record_counter = kBufferLengthSamples;
        }
      }

      out[0][i] = out[1][i] = 0.0f;
    }
    delay_time_smooth = delay_time_samples;
    SnapReverbSmoothing();
    FlushReverbTail(size);
    return;
  }

  // PLAYING: el único estado con salida audible. Los parámetros se suavizan con este coeficiente.
  const float kSmoothCoeff = 0.0005f;

  for (size_t i = 0; i < size; i++) {

    float signal_to_process = looper.Process(0.0f);

    // Filtros: se aplican siempre que no estén en su posición neutral.
    if (knob1_hpass_val > 0) {
      g_highpass_filter->Process(signal_to_process);
      signal_to_process = g_highpass_filter->High();
    }
    if (knob1_lpass_val < 100) {
      g_lowpass_filter->Process(signal_to_process);
      signal_to_process = g_lowpass_filter->Low();
    }

    // Delay: se mezcla la señal con su repetición según delay_mix.
    delay_time_smooth += kSmoothCoeff * (delay_time_samples - delay_time_smooth);
    delay_effect.SetDelay(delay_time_smooth);
    float delayed = delay_effect.Read();
    delay_effect.Write(signal_to_process + (delayed * delay_feedback));
    float post_delay = (signal_to_process * (1.0f - delay_mix)) + (delayed * delay_mix);

    // Reverb: se calcula mientras el knob pida reverb o quede cola por apagar.
    reverb_dry_smooth += kSmoothCoeff * (g_reverb_dry_target - reverb_dry_smooth);
    reverb_wet_smooth += kSmoothCoeff * (g_reverb_wet_target - reverb_wet_smooth);

    float reverb_l = 0.0f, reverb_r = 0.0f;
    if (g_reverb_wet_target > 0.0f ||
        reverb_wet_smooth   > kReverbMixFloor ||
        reverb_tail_env     > kReverbTailFloor) {
      reverb_fdn.Process(post_delay, &reverb_l, &reverb_r);

      float mag = fabsf(reverb_l) + fabsf(reverb_r);
      reverb_tail_env = (mag > reverb_tail_env) ? mag : (reverb_tail_env * kReverbEnvDecay);
    }

    // Salida estéreo: el seco va igual en los dos canales y solo la reverb se abre.
    float dry_part = post_delay * reverb_dry_smooth;
    float out_l = (dry_part + reverb_l * reverb_wet_smooth) * g_gain;
    float out_r = (dry_part + reverb_r * reverb_wet_smooth) * g_gain;

    out[0][i] = tanhf(out_l);
    out[1][i] = tanhf(out_r);
  }
}

//====================================================================
// --- RESET ---
//====================================================================
// Borra los efectos (pitch, filtros, delay y reverb). No toca la ganancia ni la región del loop.
void resetSystem() {
  delay_effect.Reset();
  g_highpass_filter->Init(DAISY.AudioSampleRate());
  g_highpass_filter->SetRes(0.7f); g_highpass_filter->SetDrive(0.7f); g_highpass_filter->SetFreq(20.0f);
  g_lowpass_filter->Init(DAISY.AudioSampleRate());
  g_lowpass_filter->SetRes(0.7f); g_lowpass_filter->SetDrive(0.7f); g_lowpass_filter->SetFreq(20000.0f);
  knob1_pitch_val = 0; knob1_hpass_val = 0; knob1_lpass_val = 100;

  knob2_reverb_val = 0; knob2_size_val = 50; knob2_decay_val = 75; knob2_damp_val = 45;
  UpdateReverbTargets();
  SnapReverbSmoothing();
  knob3_time_val = 0; knob3_feedback_val = 0; knob3_mix_val = 0;
  delay_time_samples = 48000.0f / 1000.0f * 10.0f;
  delay_time_smooth = delay_time_samples;
  delay_feedback = 0.0f; delay_mix = 0.0f;
  noInterrupts(); enc1_counter = 0; enc2_counter = 0; enc3_counter = 0; interrupts();
  enc1_mode = PITCH; knob2_mode = REVERB; knob3_mode = TIME;
}

// Borra los efectos, la ganancia y la región del loop.
void resetSystemFull() {
  resetSystem();
  g_gain = 1.0f;
  if (recorded_samples > 0) {
    loop_start_sample = 0; loop_end_sample = recorded_samples - 1;
    looper.SetLoopRegion(loop_start_sample, loop_end_sample);
  }
  enc4_mode = ENC4_MODE_GAIN;
  noInterrupts(); enc4_counter = 0; last_e4 = 0; interrupts();
}

//====================================================================
// --- SETUP ---
// Se ejecuta una vez al encender: audio a 48 kHz, lienzo, looper y efectos, pines, interrupciones,
// pantalla y pantalla de arranque.
//====================================================================
void setup() {
  delay(250);

  DAISY.init(DAISY_SEED, AUDIO_SR_48K);

  canvas = new GFXcanvas16(SCREEN_WIDTH, SCREEN_HEIGHT);

  looper.Init(buffer, kBufferLengthSamples);
  g_highpass_filter = new daisysp::Svf();
  g_highpass_filter->Init(DAISY.AudioSampleRate());
  g_highpass_filter->SetRes(0.7f); g_highpass_filter->SetDrive(0.7f); g_highpass_filter->SetFreq(10.0f);
  g_lowpass_filter = new daisysp::Svf();
  g_lowpass_filter->Init(DAISY.AudioSampleRate());
  g_lowpass_filter->SetRes(0.7f); g_lowpass_filter->SetDrive(0.7f); g_lowpass_filter->SetFreq(20000.0f);
  delay_effect.Init();
  delay_effect.SetDelay(delay_time_smooth);
  for (int i = 0; i < crearttech::FdnReverb::kLines; i++) fdn_ptrs[i] = fdn_lines[i];
  reverb_fdn.Init(DAISY.AudioSampleRate(), fdn_ptrs);
  UpdateReverbTargets();
  SnapReverbSmoothing();

  for (int i = 0; i < MAX_STARS; i++) {
    stars[i].x = random(-SCREEN_WIDTH / 2, SCREEN_WIDTH / 2);
    stars[i].y = random(-SCREEN_HEIGHT / 2, SCREEN_HEIGHT / 2);
    stars[i].z = random(8, 15);
    stars[i].speed = (15.0 - stars[i].z) * 0.3 + 0.5;
  }

  pinMode(REC_BUTTON_PIN, INPUT_PULLUP);
  pinMode(PLAY_BUTTON_PIN, INPUT_PULLUP);
  pinMode(REVERSE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  pinMode(ENC1_CLK_PIN, INPUT_PULLUP); pinMode(ENC1_DT_PIN, INPUT_PULLUP); pinMode(ENC1_SW_PIN, INPUT_PULLUP);
  pinMode(ENC2_CLK_PIN, INPUT_PULLUP); pinMode(ENC2_DT_PIN, INPUT_PULLUP); pinMode(ENC2_SW_PIN, INPUT_PULLUP);
  pinMode(ENC3_CLK_PIN, INPUT_PULLUP); pinMode(ENC3_DT_PIN, INPUT_PULLUP); pinMode(ENC3_SW_PIN, INPUT_PULLUP);
  pinMode(ENC4_CLK_PIN, INPUT_PULLUP); pinMode(ENC4_DT_PIN, INPUT_PULLUP); pinMode(ENC4_SW_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_CLK_PIN), encoder1_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_CLK_PIN), encoder2_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3_CLK_PIN), encoder3_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4_CLK_PIN), encoder4_isr, CHANGE);

  tft.initR(INITR_GREENTAB);
  tft.quitarDesplazamiento();
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  unsigned long splash_start_time = millis();
  while (millis() - splash_start_time < 2000) {
    float progress = (float)(millis() - splash_start_time) / 2000.0f;
    drawSplashScreen(progress);
    tft.drawRGBBitmap(0, 0, canvas->getBuffer(), SCREEN_WIDTH, SCREEN_HEIGHT);
    delay(30);
  }

  DAISY.StartAudio(AudioCallback);
}

//====================================================================
// --- LOOP ---
// Se repite mientras el equipo está encendido: encoders, forma de onda, botones y pantalla.
//====================================================================
void loop() {

  // Lectura de los contadores de los encoders con las interrupciones pausadas.
  noInterrupts();
  int e1 = enc1_counter; int e2 = enc2_counter; int e3 = enc3_counter; int e4 = enc4_counter;
  interrupts();
  int e4_delta = e4 - last_e4; last_e4 = e4;

  // ENC4: ganancia, inicio, final y movimiento de la región del loop.
  bool enc4_sw = digitalRead(ENC4_SW_PIN);
  if (last_enc4_sw_state == HIGH && enc4_sw == LOW) {
    if (enc4_mode == ENC4_MODE_GAIN) enc4_mode = ENC4_MODE_START_POINT;
    else if (enc4_mode == ENC4_MODE_START_POINT) enc4_mode = ENC4_MODE_END_POINT;
    else if (enc4_mode == ENC4_MODE_END_POINT) enc4_mode = ENC4_MODE_MOVE;
    else enc4_mode = ENC4_MODE_GAIN;
    noInterrupts(); enc4_counter = 0; last_e4 = 0; interrupts();
  }
  last_enc4_sw_state = enc4_sw;

  if (e4_delta != 0 && recorded_samples > 0) {
    int sensitivity = max(1, (int)(recorded_samples / 500));
    long delta = (long)e4_delta * sensitivity;
    switch (enc4_mode) {
      case ENC4_MODE_START_POINT: {
        long new_start = (long)loop_start_sample + delta;
        if (new_start < 0) new_start = 0;
        if (new_start >= loop_end_sample) new_start = (loop_end_sample > 0) ? (loop_end_sample - 1) : 0;
        loop_start_sample = (size_t)new_start; break;
      }
      case ENC4_MODE_END_POINT: {
        long new_end = (long)loop_end_sample + delta;
        if (new_end <= loop_start_sample) new_end = loop_start_sample + 1;
        if (new_end >= recorded_samples) new_end = recorded_samples - 1;
        if (new_end < 0) new_end = 0;
        loop_end_sample = (size_t)new_end; break;
      }
      case ENC4_MODE_MOVE: {

        long new_start = (long)loop_start_sample + delta;
        long new_end = (long)loop_end_sample + delta;
        if (new_start < 0) { new_start = 0; new_end = (loop_end_sample - loop_start_sample); }
        if (new_end >= recorded_samples) { new_end = recorded_samples - 1; new_start = new_end - (loop_end_sample - loop_start_sample); }
        if (new_start < 0) new_start = 0;
        if (new_start >= new_end) { new_start = new_end - 1; if (new_start < 0) new_start = 0; }
        loop_start_sample = (size_t)new_start; loop_end_sample = (size_t)new_end; break;
      }
      case ENC4_MODE_GAIN: {

        g_gain += (float)e4_delta * 0.01f; g_gain = constrain(g_gain, 0.0f, 2.0f); break;
      }
    }
    looper.SetLoopRegion(loop_start_sample, loop_end_sample);
  }

  // ENC1: pitch (de -6 a +6 semitonos), pasa altos y pasa bajos.
  switch (enc1_mode) {
    case PITCH: {

        const int kPitchCounterLimit = 6 * PITCH_SENSITIVITY;
        int clamped_e1 = constrain(e1, -kPitchCounterLimit, kPitchCounterLimit);
        if (clamped_e1 != e1) { noInterrupts(); enc1_counter = clamped_e1; interrupts(); }
        e1 = clamped_e1;
        int pitch_semitones = (int)roundf((float)e1 / (float)PITCH_SENSITIVITY);
        looper.SetPlaybackSpeed(powf(2.0f, (float)pitch_semitones / 12.0f));
        knob1_pitch_val = e1;
      } break;
    case HIGHPASS: {

        int clamped_e1 = constrain(e1, 0, 100);
        if (clamped_e1 != e1) { noInterrupts(); enc1_counter = clamped_e1; interrupts(); }
        e1 = clamped_e1;
        g_highpass_filter->SetFreq(20.0f * powf(500.0f, (float)e1 / 100.0f));
        knob1_hpass_val = e1;
      } break;
    case LOWPASS: {

        int clamped_e1 = constrain(e1, 0, 100);
        if (clamped_e1 != e1) { noInterrupts(); enc1_counter = clamped_e1; interrupts(); }
        e1 = clamped_e1;
        g_lowpass_filter->SetFreq(200.0f * powf(100.0f, (float)e1 / 100.0f));
        knob1_lpass_val = e1;
      } break;
  }

  // ENC2: reverb (REVERB, SIZE, DECAY y DAMP).
  e2 = constrain(e2, 0, 100); e3 = constrain(e3, 0, 100);
  noInterrupts(); enc2_counter = e2; enc3_counter = e3; interrupts();

  switch (knob2_mode) {
    case REVERB: knob2_reverb_val = e2; break;
    case SIZE: knob2_size_val = e2; break;
    case DECAY: knob2_decay_val = e2; break;
    case DAMP: knob2_damp_val = e2; break;
  }

  bool enc2_sw = digitalRead(ENC2_SW_PIN);
  if (last_enc2_sw_state == HIGH && enc2_sw == LOW) {
    if (knob2_mode == REVERB) { knob2_mode = SIZE; enc2_counter = knob2_size_val; }
    else if (knob2_mode == SIZE) { knob2_mode = DECAY; enc2_counter = knob2_decay_val; }
    else if (knob2_mode == DECAY) { knob2_mode = DAMP; enc2_counter = knob2_damp_val; }
    else { knob2_mode = REVERB; enc2_counter = knob2_reverb_val; }
  }
  last_enc2_sw_state = enc2_sw;
  UpdateReverbTargets();

  // ENC3: delay (tiempo de 10 ms a 1 s, feedback y mezcla).
  switch (knob3_mode) {
    case TIME: { knob3_time_val = e3; float delay_ms = 10.0f * powf(100.0f, (float)knob3_time_val / 100.0f); delay_time_samples = DAISY.AudioSampleRate() / 1000.0f * delay_ms; } break;
    case DELAY: delay_feedback = (float)e3 / 100.0f; knob3_feedback_val = e3; break;
    case MIX: delay_mix = (float)e3 / 100.0f; knob3_mix_val = e3; break;
  }

  bool enc3_sw = digitalRead(ENC3_SW_PIN);
  if (last_enc3_sw_state == HIGH && enc3_sw == LOW) {
    if (knob3_mode == TIME) { knob3_mode = DELAY; enc3_counter = knob3_feedback_val; }
    else if (knob3_mode == DELAY) { knob3_mode = MIX; enc3_counter = knob3_mix_val; }
    else { knob3_mode = TIME; enc3_counter = knob3_time_val; }
  }
  last_enc3_sw_state = enc3_sw;

  // Forma de onda: se escala al alto del recuadro y se reduce a una columna por píxel.
  noInterrupts(); size_t current_recorded_samples = record_counter; interrupts();
  if (current_recorded_samples > 0) {
    float max_abs_val = 1e-6f;
    for (size_t i = 0; i < current_recorded_samples; i++) {
        float a = fabsf(waveform_source_buffer[i]); if (a > max_abs_val) max_abs_val = a;
    }
    if (max_abs_val < 1e-6f) max_abs_val = 1e-6f;
    waveform_scale = ((WAVEFORM_H / 2.0f) / max_abs_val) * 0.7f;
    generarOnda(displayWaveform, DISPLAY_W, waveform_source_buffer, current_recorded_samples);
    waveform_ready = true;
  } else waveform_ready = false;

  // Botones.
  bool rec_button = digitalRead(REC_BUTTON_PIN);
  bool play_button = digitalRead(PLAY_BUTTON_PIN);
  bool reverse_button = digitalRead(REVERSE_BUTTON_PIN);
  bool reset_button = digitalRead(RESET_BUTTON_PIN);

  // RESET: doble pulsación borra los efectos; mantenido 1 s borra también ganancia y región.
  if (last_reset_button_state == HIGH && reset_button == LOW) {
    unsigned long currentTime = millis();
    reset_press_start_time = currentTime;
    reset_long_fired = false;
    if (currentTime - last_reset_press_time < DOUBLE_PRESS_TIME_MS) reset_press_count++; else reset_press_count = 1;
    last_reset_press_time = currentTime;
    if (reset_press_count == 2) {
      resetSystem();
      reset_press_count = 0;
    }
  }
  if (reset_button == LOW && !reset_long_fired && (millis() - reset_press_start_time > LONG_PRESS_TIME_MS)) {
    resetSystemFull();
    reset_long_fired = true;
    reset_press_count = 0;
  }
  last_reset_button_state = reset_button;

  // ENC1, pulsador: cambia entre pitch, pasa altos y pasa bajos; cada modo conserva su valor.
  if (last_enc1_sw_state == HIGH && digitalRead(ENC1_SW_PIN) == LOW) {

    switch (enc1_mode) {
      case PITCH:    knob1_pitch_val = enc1_counter; break;
      case HIGHPASS: knob1_hpass_val = enc1_counter; break;
      case LOWPASS:  knob1_lpass_val = enc1_counter; break;
    }

    if (enc1_mode == PITCH) enc1_mode = HIGHPASS; else if (enc1_mode == HIGHPASS) enc1_mode = LOWPASS; else enc1_mode = PITCH;

    int restored_val;
    switch (enc1_mode) {
      case PITCH:    restored_val = knob1_pitch_val; break;
      case HIGHPASS: restored_val = knob1_hpass_val; break;
      case LOWPASS:  restored_val = knob1_lpass_val; break;
    }
    noInterrupts(); enc1_counter = restored_val; interrupts();

    if (enc1_mode == HIGHPASS) {
      g_highpass_filter->Init(DAISY.AudioSampleRate());
      g_highpass_filter->SetRes(0.7f); g_highpass_filter->SetDrive(0.7f);
      g_highpass_filter->SetFreq(20.0f * powf(500.0f, (float)restored_val / 100.0f));
    } else if (enc1_mode == LOWPASS) {
      g_lowpass_filter->Init(DAISY.AudioSampleRate());
      g_lowpass_filter->SetRes(0.7f); g_lowpass_filter->SetDrive(0.7f);
      g_lowpass_filter->SetFreq(200.0f * powf(100.0f, (float)restored_val / 100.0f));
    }
  }
  last_enc1_sw_state = digitalRead(ENC1_SW_PIN);

  // REC: se mantiene pulsado para grabar (STOPPED) o hacer overdub (PLAYING); al soltar vuelve a PLAYING.
  bool rec_button_is_pressed = (rec_button == LOW);
  bool rec_button_was_pressed = (last_rec_button_state == LOW);
  if (rec_button_is_pressed && !rec_button_was_pressed) {
    if (looper_state == STOPPED) {
      memset(buffer, 0, sizeof(float) * kBufferLengthSamples);
      looper.StartRecording(); looper_state = RECORDING;
      recorded_samples = 0; record_counter = 0; rec_fade_counter = 0;
      waveform_ready = false;
    } else if (looper_state == PLAYING) {
      looper.StartOverdub(); looper_state = OVERDUB; rec_fade_counter = 0;
    }
  }
  if (!rec_button_is_pressed && rec_button_was_pressed) {
    if (looper_state == RECORDING) {
      looper.StopRecording(); recorded_samples = record_counter;
      loop_start_sample = 0; loop_end_sample = recorded_samples > 0 ? recorded_samples - 1 : 0;
      looper.SetLoopRegion(loop_start_sample, loop_end_sample);
      looper_state = PLAYING;
    } else if (looper_state == OVERDUB) {
      looper.StopOverdub(); looper_state = PLAYING;
    }
  }
  last_rec_button_state = rec_button;

  // PLAY / PAUSA: doble pulsación borra el loop; una pulsación alterna PLAYING y PAUSED.
  if (last_play_button_state == HIGH && play_button == LOW) {
    play_button_press_time = millis(); play_button_long_press_actioned = false;
    unsigned long currentTime = millis();
    if (currentTime - lastPlayPressTime < DOUBLE_PRESS_TIME_MS) playPressCount++; else playPressCount = 1;
    lastPlayPressTime = currentTime;
    if (playPressCount == 2) {
      looper.Restart(); if (looper_state == RECORDING) looper.StopRecording();
      looper_state = STOPPED; recorded_samples = 0;
      noInterrupts(); record_counter = 0; interrupts();
      waveform_ready = false; playPressCount = 0;
    }
  }
  if (play_button == LOW && !play_button_long_press_actioned) {
    if (millis() - play_button_press_time > 500) { play_button_long_press_actioned = true; }
  }
  if (playPressCount == 1 && (millis() - lastPlayPressTime > DOUBLE_PRESS_TIME_MS)) {
    if (!play_button_long_press_actioned) {
      if (looper_state == PAUSED) looper_state = PLAYING;
      else if (looper_state == PLAYING) looper_state = PAUSED;
    }
    playPressCount = 0;
  }
  last_play_button_state = play_button;

  // REVERSA: activa y desactiva la reproducción hacia atrás.
  if (reverse_button == LOW && last_reverse_button_state == HIGH) {
    reverse_mode = !reverse_mode; looper.SetReverse(reverse_mode);
  }
  last_reverse_button_state = reverse_button;

  // Pantalla: se dibuja en el lienzo y se envía cada 30 ms (unos 33 cuadros por segundo).
  static unsigned long last_draw = 0;
  if (millis() - last_draw > 30) {
    drawScreen(); tft.drawRGBBitmap(0, 0, canvas->getBuffer(), SCREEN_WIDTH, SCREEN_HEIGHT);
    last_draw = millis();
  }
}
