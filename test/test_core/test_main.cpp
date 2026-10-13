// Testes do núcleo (lib/core) — sem framework: g++ + asserts.
// Rodar: sim/build_core.sh && sim/build/test_core
#include "lf_core.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

static int fails = 0, checks = 0;
#define CHECK(cond) do { checks++; if (!(cond)) { fails++; printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define NEAR(a, b, tol) do { checks++; double _a = (a), _b = (b); if (fabs(_a - _b) > (tol)) { fails++; \
  printf("  FAIL %s:%d: %s=%g vs %s=%g (tol %g)\n", __FILE__, __LINE__, #a, _a, #b, _b, (double)(tol)); } } while (0)

static lf_frame_t frame(const uint16_t* s, float gz = 0, float ay = 0) {
  lf_frame_t f; memset(&f, 0, sizeof(f));
  for (int i = 0; i < 8; i++) f.s[i] = s[i];
  f.gyro_z = gz; f.acc_y = ay; f.dt = 0.001f;
  return f;
}
static const uint16_t S_CENTER[8] = {0, 0, 0, 900, 900, 0, 0, 0};
static const uint16_t S_RIGHT[8]  = {0, 0, 0, 0, 0, 900, 900, 0};
static const uint16_t S_FARL[8]   = {900, 900, 0, 0, 0, 0, 0, 0};
static const uint16_t S_FARR[8]   = {0, 0, 0, 0, 0, 0, 900, 900};
static const uint16_t S_ALL[8]    = {900, 900, 900, 900, 900, 900, 900, 900};
static const uint16_t S_NONE[8]   = {0, 0, 0, 0, 0, 0, 0, 0};

static void run(lf_handle_t h, const uint16_t* s, int n, lf_cmd_t* last = 0, float gz = 0, float ay = 0) {
  lf_cmd_t c; lf_frame_t f = frame(s, gz, ay);
  for (int i = 0; i < n; i++) lf_step(h, &f, &c);
  if (last) *last = c;
}
static lf_telem_t telem(lf_handle_t h) { lf_telem_t t; lf_get_telem(h, &t); return t; }
static lf_handle_t started() { lf_handle_t h = lf_create(); lf_start(h); return h; }

static void test_struct_sizes() {
  int o[4]; lf_struct_sizes(o);
  CHECK(o[0] == (int)sizeof(lf_gains_t)); CHECK(o[1] == (int)sizeof(lf_telem_t));
  CHECK(o[2] == (int)sizeof(lf_map_entry_t)); CHECK(o[3] == (int)sizeof(lf_frame_t));
}

static void test_line_position_and_steer_sign() {
  lf_handle_t h = started(); lf_cmd_t c;
  run(h, S_RIGHT, 50, &c);
  lf_telem_t t = telem(h);
  NEAR(t.line_pos, (5500.0 - 3500.0) / 3500.0, 0.01);     // sensores 5 e 6 pretos
  CHECK(t.mode == LF_MODE_FOLLOW);
  CHECK(c.left > c.right);                                   // linha à direita -> vira à direita
  run(h, S_CENTER, 50, &c);
  NEAR(telem(h).line_pos, 0.0, 0.01);
  lf_destroy(h);
}

static void test_heading_integration() {
  lf_handle_t h = started();
  run(h, S_CENTER, 1000, 0, 1.0f);                           // 1 s a 1 rad/s
  NEAR(telem(h).heading, 1.0, 0.02);
  lf_destroy(h);
}

static void test_gyro_bias_estimated_when_idle() {
  lf_handle_t h = lf_create();
  run(h, S_CENTER, 3000, 0, 0.05f);                          // parado, giro lendo 0.05 rad/s
  NEAR(lf_get_gyro_bias(h), 0.05, 0.002);
  lf_start(h);
  run(h, S_CENTER, 1000, 0, 0.05f);
  NEAR(telem(h).heading, 0.0, 0.02);                         // bias compensado
  lf_destroy(h);
}

static void test_bar_event_and_cross_mode() {
  lf_handle_t h = started();
  run(h, S_CENTER, 200);
  run(h, S_ALL, 25);
  lf_telem_t t = telem(h);
  CHECK(t.bars == 1); CHECK(t.mode == LF_MODE_CROSS);
  run(h, S_CENTER, 200);
  CHECK(telem(h).mode == LF_MODE_FOLLOW);
  CHECK(telem(h).bars == 1);                                 // sem evento duplicado
  lf_destroy(h);
}

static void test_lost_line_arc_and_recover() {
  lf_handle_t h = started(); lf_cmd_t c;
  run(h, S_RIGHT, 200);
  run(h, S_NONE, 250, &c);                                   // > lost_time (0,15 s) sem linha
  CHECK(telem(h).mode == LF_MODE_LOST);
  CHECK(c.left > c.right);                                   // arco para a direita (último lado visto)
  CHECK(c.left > 0);                                         // avança procurando, não pivota parado
  run(h, S_CENTER, 50);
  CHECK(telem(h).mode == LF_MODE_FOLLOW);
  lf_destroy(h);
}

// linha varrendo o array continuamente (onda triangular), como num zigue-zague de verdade
static void sweep_line(lf_handle_t h, int half_periods, int ms_per_step) {
  for (int k = 0; k < half_periods; k++)
    for (int i = 0; i <= 12; i++) {
      float pos = (k & 1) ? 6.5f - 0.5f * i : 0.5f + 0.5f * i;      // 0.5 .. 6.5 e volta
      uint16_t s[8] = {0, 0, 0, 0, 0, 0, 0, 0};
      int lo = (int)pos; float fr = pos - lo;
      s[lo] = (uint16_t)(900 * (1 - fr)); if (lo + 1 < 8) s[lo + 1] = (uint16_t)(900 * fr);
      run(h, s, ms_per_step);
    }
}

static void test_zigzag_detect_hold_and_exit() {
  lf_handle_t h = started();
  run(h, S_CENTER, 300);
  sweep_line(h, 8, 6);                                       // 8 meios-períodos de ~78 ms
  lf_telem_t t = telem(h);
  CHECK(t.mode == LF_MODE_ZIGZAG); CHECK(t.seg == LF_SEG_ZIGZAG);
  run(h, S_CENTER, 600);                                     // linha parou de oscilar
  CHECK(telem(h).mode == LF_MODE_FOLLOW);
  lf_map_entry_t m[LF_MAP_MAX]; int n = lf_get_rec(h, m, LF_MAP_MAX);
  bool has_zz = false; for (int i = 0; i < n; i++) if (m[i].type == LF_SEG_ZIGZAG) has_zz = true;
  CHECK(has_zz);
  lf_destroy(h);
}

static void test_curve_segments_recorded() {
  lf_handle_t h = started();
  run(h, S_CENTER, 300);                                     // reta
  run(h, S_CENTER, 600, 0, 3.0f, 2.0f);                      // curva à esquerda 1,8 rad
  run(h, S_CENTER, 400);                                     // reta
  lf_map_entry_t m[LF_MAP_MAX]; int n = lf_get_rec(h, m, LF_MAP_MAX);
  CHECK(n >= 2);
  bool ok = false;
  for (int i = 0; i < n; i++) if (m[i].type == LF_SEG_CURVE_L && fabs(m[i].angle - 1.8) < 0.25) ok = true;
  CHECK(ok);
  lf_destroy(h);
}

static void test_map_roundtrip_and_finish() {
  lf_handle_t h = lf_create();
  lf_map_entry_t in[3] = {{LF_SEG_BAR, 0, 0, 0, 0}, {LF_SEG_STRAIGHT, 0.01f, 1.0f, 0.8f, 0}, {LF_SEG_CURVE_L, 1.57f, 0.5f, 0.3f, 5.0f}};
  lf_set_map(h, in, 3);
  lf_map_entry_t out[LF_MAP_MAX]; int n = lf_get_map(h, out, LF_MAP_MAX);
  CHECK(n == 3); CHECK(out[2].type == LF_SEG_CURVE_L); NEAR(out[1].dist, 0.8, 1e-6);
  lf_gains_t g; lf_get_gains(h, &g); g.laps = 1; lf_set_gains(h, &g);
  lf_start(h);
  CHECK(telem(h).has_map == 1);
  run(h, S_CENTER, 2500);                                    // ~1,2 m estimados (> 50% do mapa)
  run(h, S_ALL, 25);                                         // barra = índice 0 -> fecha volta 1 -> termina
  lf_cmd_t c; run(h, S_CENTER, 50, &c);
  CHECK(telem(h).mode == LF_MODE_FINISHED);
  CHECK(c.left == 0 && c.right == 0);
  lf_destroy(h);
}

static void test_output_saturation_keeps_differential() {
  lf_handle_t h = lf_create();
  lf_gains_t g; lf_get_gains(h, &g); g.kp = 600; g.v_base = 240; g.ramp_up = 100000; lf_set_gains(h, &g);
  lf_start(h);
  lf_cmd_t c; run(h, S_FARR, 100, &c);
  CHECK(c.left == 255); CHECK(c.right < 0);
  lf_destroy(h);
}

int main() {
  test_struct_sizes();
  test_line_position_and_steer_sign();
  test_heading_integration();
  test_gyro_bias_estimated_when_idle();
  test_bar_event_and_cross_mode();
  test_lost_line_arc_and_recover();
  test_zigzag_detect_hold_and_exit();
  test_curve_segments_recorded();
  test_map_roundtrip_and_finish();
  test_output_saturation_keeps_differential();
  printf("%d checks, %d failures\n", checks, fails);
  return fails ? 1 : 0;
}
