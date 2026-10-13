// lf_core.cpp — ver lf_core.h para convenções e unidades.
#include "lf_core.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline float absf(float x) { return x < 0 ? -x : x; }
inline float ema(float prev, float x, float dt, float tau) { float a = dt / (tau + dt); return prev + a * (x - prev); }
inline float wrap_pi(float a) { while (a > (float)M_PI) a -= 2 * (float)M_PI; while (a < -(float)M_PI) a += 2 * (float)M_PI; return a; }
inline bool is_curve(uint8_t t) { return t == LF_SEG_CURVE_L || t == LF_SEG_CURVE_R; }
inline uint8_t curve_of(float w) { return w > 0 ? (uint8_t)LF_SEG_CURVE_L : (uint8_t)LF_SEG_CURVE_R; }
const uint8_t PROV_NONE = 255;

class Core {
 public:
  Core() { lf_default_gains(&g_); bias_ = 0; reset(); }

  void reset() {
    memset(&st_, 0, sizeof(st_));
    st_.mode = LF_MODE_IDLE;
    st_.k = g_.k_pwm2v;
    st_.lap = 1;
    st_.prov = PROV_NONE;
    has_map_ = false; map_n_ = 0; map_idx_ = 0; rec_n_ = 0; first_bar_ = -1;
    pwm_cmd_ = 0; mismatches_ = 0; last_l_ = last_r_ = 0;
    ext_enable_ = false; ext_steer_ = 0; ext_scale_ = 1;
    boost_ = 1.0f; lap_lost_t_ = 0; lap_backed_ = false;
  }

  void set_external(float steer, float scale, bool en) { ext_steer_ = steer; ext_scale_ = scale; ext_enable_ = en; }

  void start() {
    if (st_.mode != LF_MODE_IDLE && st_.mode != LF_MODE_FINISHED) return;
    st_.mode = LF_MODE_FOLLOW;
    st_.t_lap = 0; st_.dist_lap = 0;
    if (!has_map_) { rec_n_ = 0; first_bar_ = -1; st_.lap = 1; }
    map_idx_ = 0; mismatches_ = 0;
    seg_begin(LF_SEG_STRAIGHT, 0);
    pwm_cmd_ = 0; st_.t_mode = 0; st_.ie = 0;
    st_.heading = 0; st_.heading_lp = 0; st_.hold_target = 0; st_.hold_rate = 0;   // rumo de referência = largada
    boost_ = 1.0f; lap_lost_t_ = 0;
    if (st_.k <= 0) st_.k = g_.k_pwm2v;
  }

  void stop() { st_.mode = LF_MODE_IDLE; pwm_cmd_ = 0; last_l_ = last_r_ = 0; }

  lf_cmd_t step(const lf_frame_t& f) {
    const float dt = clampf(f.dt, 1e-4f, 0.05f);
    st_.t += dt;
    sense(f, dt);
    lf_cmd_t out = {0, 0};
    if (st_.mode == LF_MODE_IDLE) {
      bias_ = ema(bias_, f.gyro_z, dt, 0.5f);   // parado: estima o bias do giroscópio
      st_.v_est = 0; last_l_ = last_r_ = 0;
      return out;
    }
    if (st_.mode == LF_MODE_FINISHED) { last_l_ = last_r_ = 0; return out; }

    st_.t_lap += dt;
    if (st_.mode == LF_MODE_LOST) {
      lap_lost_t_ += dt;
      if (has_map_ && g_.lap_boost > 0 && lap_lost_t_ > g_.lost_tol && !lap_backed_) {   // recua já, não espera a volta acabar
        boost_ = clampf(boost_ * (1.0f - g_.lap_backoff), g_.boost_min, g_.boost_max); lap_backed_ = true;
      }
    }
    estimate_speed(dt);
    detect_bar(dt);
    detect_zigzag(dt);
    classify(dt);
    update_modes(dt);
    float steer = steering(dt);
    float v = speed_cmd(dt);
    mix(v, steer, out);
    last_l_ = out.left; last_r_ = out.right;
    st_.steer = steer;
    return out;
  }

  void telem(lf_telem_t& t) const {
    t.line_pos = st_.e; t.heading = st_.heading; t.omega_f = st_.w_f; t.v_est = st_.v_est;
    t.dist_lap = st_.dist_lap; t.t_lap = st_.t_lap; t.steer = st_.steer; t.k_pwm2v = st_.k;
    t.seg_angle = st_.seg_angle; t.v_target = pwm_cmd_; t.zz_target = st_.hold_target;
    t.zz_half = st_.zz_half; t.zz_mid = st_.zz_mid; t.zz_t_far = st_.zz_t_far; t.zz_swaps = st_.zz_swaps_n; t.boost = boost_;
    t.mode = st_.mode; t.seg = st_.seg_type; t.map_idx = (uint8_t)map_idx_; t.map_len = (uint8_t)map_n_;
    t.lap = st_.lap; t.has_map = has_map_ ? 1 : 0; t.on_line = st_.on_line; t.bars = st_.bars; t.n_black = st_.n_black;
    t.map_seg = (has_map_ && map_n_) ? map_[cur_index()].type : 255;
  }

  int get_map(lf_map_entry_t* out, int max) const {
    int n = has_map_ ? map_n_ : rec_n_;
    const lf_map_entry_t* src = has_map_ ? map_ : rec_;
    if (n > max) n = max;
    memcpy(out, src, n * sizeof(lf_map_entry_t));
    return n;
  }
  int get_rec(lf_map_entry_t* out, int max) const {
    int n = rec_n_ > max ? max : rec_n_;
    memcpy(out, rec_, n * sizeof(lf_map_entry_t));
    return n;
  }
  void set_map(const lf_map_entry_t* in, int n) {
    if (n <= 0 || !g_.use_map) { has_map_ = false; map_n_ = 0; return; }
    if (n > LF_MAP_MAX) n = LF_MAP_MAX;
    memcpy(map_, in, n * sizeof(lf_map_entry_t));
    map_n_ = n; has_map_ = true; map_idx_ = 0; mismatches_ = 0;
    map_dist_ = 0; for (int i = 0; i < n; i++) map_dist_ += map_[i].dist;
  }
  void lap_boundary() { if (!has_map_) close_lap_at(rec_n_, 0); else { lap_tick(); map_idx_ = 0; } }

  lf_gains_t g_;
  float bias_;

 private:
  struct State {
    float t, t_lap, dist_lap;
    float heading, w, w_f, ay_f, heading_lp;
    float e, e_prev, e_dot, ie, e_lp;
    float v_est, k, pwm_f;
    uint8_t mode, seg_type, on_line, bars, n_black, lap;
    float seg_angle, seg_t, seg_dist, seg_kmax, kappa_f;
    uint8_t prov; float prov_t0, prov_d0, prov_a0;      // trecho provisório dentro do segmento corrente
    float t_above, t_below, t_noline;
    uint8_t bar_cnt, bar_active; float t_since_bar;
    int8_t zz_dir; float zz_ext, zz_max_last, zz_min_last, zz_mid, zz_e_lp; uint8_t zz_nrev, zz_fresh;
    float zz_swaps_t[8], zz_swaps_h[8], zz_swaps_hi[8]; uint8_t zz_swaps_n; float zz_last_swap, zz_half;
    float h_int; uint8_t zz_detected, zz_hint_blocked;
    float hold_target, hold_rate, t_mode, steer, zz_t_far;
  } st_;

  int16_t last_l_, last_r_;
  float pwm_cmd_;
  bool ext_enable_; float ext_steer_, ext_scale_;
  float boost_; float lap_lost_t_; bool lap_backed_;
  lf_map_entry_t rec_[LF_MAP_MAX]; int rec_n_; int first_bar_;
  lf_map_entry_t map_[LF_MAP_MAX]; int map_n_; int map_idx_; bool has_map_; int mismatches_; float map_dist_;

  // ------------------------------------------------------------------ percepção
  void sense(const lf_frame_t& f, float dt) {
    st_.w = f.gyro_z - bias_;
    st_.w_f = ema(st_.w_f, st_.w, dt, 0.02f);
    st_.ay_f = ema(st_.ay_f, f.acc_y, dt, 0.05f);
    st_.heading += st_.w * dt;
    st_.h_int += st_.heading * dt;                       // integral do rumo: média exata entre dois instantes
    st_.heading_lp = ema(st_.heading_lp, st_.heading, dt, 0.08f);
    { float v = st_.v_est > 0.3f ? st_.v_est : 0.3f; st_.kappa_f = ema(st_.kappa_f, absf(st_.w) / v, dt, 0.05f); }
    if (st_.kappa_f > st_.seg_kmax) st_.seg_kmax = st_.kappa_f;

    uint32_t sum = 0, wsum = 0; uint16_t mx = 0; uint8_t nb = 0;
    for (int i = 0; i < LF_N_SENS; i++) {
      uint16_t v = f.s[i] > 1000 ? 1000 : f.s[i];
      sum += v; wsum += (uint32_t)v * (uint32_t)(i * 1000);
      if (v > mx) mx = v;
      if (v > 500) nb++;
    }
    st_.n_black = nb;
    // histerese: perde a linha só quando o sinal cai bem (mx < on_line_lo) e reconhece de volta
    // quando sobe (mx > on_line_hi). Limiares baixos para reconhecer a linha mesmo com calibração mediana,
    // senão ele cruza a linha no modo perdido e não a reconhece (fica girando preso).
    if (mx > g_.on_line_hi) st_.on_line = 1;
    else if (mx < g_.on_line_lo) st_.on_line = 0;
    if (st_.on_line && sum > 0) st_.e = ((float)wsum / (float)sum - 3500.0f) / 3500.0f;
    float de = (st_.e - st_.e_prev) / dt;
    st_.e_prev = st_.e;
    st_.e_dot = ema(st_.e_dot, de, dt, 0.008f);
    st_.e_lp = ema(st_.e_lp, st_.e, dt, 0.15f);
  }

  void estimate_speed(float dt) {
    float pwm = 0.5f * ((float)last_l_ + (float)last_r_);
    st_.pwm_f = ema(st_.pwm_f, pwm, dt, g_.tau_motor);
    st_.v_est = st_.k * st_.pwm_f;
    if (st_.v_est < 0) st_.v_est = 0;
    st_.dist_lap += st_.v_est * dt;
    st_.seg_dist += st_.v_est * dt;
    // calibração pwm->v em curva sustentada: v = a_lat / ω (só com IMU, sem encoder)
    if (st_.mode == LF_MODE_FOLLOW && absf(st_.w_f) > 1.5f && absf(st_.ay_f) > 0.8f && st_.pwm_f > 20.0f) {
      float v_meas = absf(st_.ay_f) / absf(st_.w_f);
      if (v_meas > 0.1f && v_meas < 5.0f) {
        st_.k = ema(st_.k, v_meas / st_.pwm_f, dt, 0.6f);
        st_.k = clampf(st_.k, 0.25f * g_.k_pwm2v, 4.0f * g_.k_pwm2v);
      }
    }
  }

  void detect_bar(float dt) {
    st_.t_since_bar += dt;
    bool black = st_.n_black >= g_.bar_min_black;
    if (black) { if (st_.bar_cnt < 255) st_.bar_cnt++; } else st_.bar_cnt = 0;
    bool was = st_.bar_active != 0;
    st_.bar_active = (st_.bar_cnt >= 3) ? 1 : (black ? st_.bar_active : 0);
    if (!was && st_.bar_active && st_.t_since_bar > 0.05f) {
      st_.t_since_bar = 0;
      st_.bars++;
      on_bar_event();
    }
  }

  void detect_zigzag(float dt) {
    // reversões de e (pico/vale com histerese): não depende de onde está o centro da oscilação
    float e = st_.e;
    bool rev = false;
    if (st_.zz_dir == 0) { st_.zz_dir = 1; st_.zz_ext = e; }
    if (st_.zz_dir > 0) {
      if (e > st_.zz_ext) st_.zz_ext = e;
      else if (e < st_.zz_ext - g_.zz_hyst) { st_.zz_max_last = st_.zz_ext; st_.zz_dir = -1; st_.zz_ext = e; rev = true; }
    } else {
      if (e < st_.zz_ext) st_.zz_ext = e;
      else if (e > st_.zz_ext + g_.zz_hyst) { st_.zz_min_last = st_.zz_ext; st_.zz_dir = 1; st_.zz_ext = e; rev = true; }
    }
    if (rev) {
      if (st_.zz_nrev < 255) st_.zz_nrev++;
      float amp = absf(st_.zz_max_last - st_.zz_min_last);
      if (st_.zz_nrev >= 2 && amp >= g_.zz_amp) {              // reversão larga: linha varreu o array
        if (st_.zz_fresh < 255) st_.zz_fresh++;
        if (st_.zz_fresh >= 2) st_.zz_mid = 0.5f * (st_.zz_max_last + st_.zz_min_last);   // centro do vai-e-vem = linha média
        if (st_.zz_swaps_n < 8) { uint8_t n = st_.zz_swaps_n++; st_.zz_swaps_t[n] = st_.t; st_.zz_swaps_h[n] = st_.heading; st_.zz_swaps_hi[n] = st_.h_int; }
        else {
          memmove(st_.zz_swaps_t, st_.zz_swaps_t + 1, 7 * sizeof(float)); st_.zz_swaps_t[7] = st_.t;
          memmove(st_.zz_swaps_h, st_.zz_swaps_h + 1, 7 * sizeof(float)); st_.zz_swaps_h[7] = st_.heading;
          memmove(st_.zz_swaps_hi, st_.zz_swaps_hi + 1, 7 * sizeof(float)); st_.zz_swaps_hi[7] = st_.h_int;
        }
        st_.zz_half = st_.t - st_.zz_last_swap;
        st_.zz_last_swap = st_.t;
      }
    }
    uint8_t keep = 0;
    for (uint8_t i = 0; i < st_.zz_swaps_n; i++)
      if (st_.t - st_.zz_swaps_t[i] <= g_.zz_window) { st_.zz_swaps_t[keep] = st_.zz_swaps_t[i]; st_.zz_swaps_h[keep] = st_.zz_swaps_h[i]; st_.zz_swaps_hi[keep++] = st_.zz_swaps_hi[i]; }
    st_.zz_swaps_n = keep;
    if (keep == 0 && st_.mode != LF_MODE_ZIGZAG) st_.zz_half = 0;   // período desconhecido até ver reversões de novo
    // reversões da mesma fase (um período de distância) têm o mesmo rumo no zigue-zague; numa curva, não
    bool straightish = true;
    if (keep >= 3) straightish = absf(st_.zz_swaps_h[keep - 1] - st_.zz_swaps_h[keep - 3]) < g_.zz_max_turn;
    else if (keep == 2) straightish = absf(st_.zz_swaps_h[1] - st_.zz_swaps_h[0]) < 2.0f * g_.zz_max_turn;
    // posição da linha filtrada num período inteiro: centro da oscilação, contínuo
    { float tau = 2.0f * st_.zz_half; if (tau < 0.15f) tau = 0.15f; if (tau > 0.6f) tau = 0.6f; st_.zz_e_lp = ema(st_.zz_e_lp, st_.e, dt, tau); }
    uint8_t need = g_.zz_min_swaps;
    if (!st_.zz_hint_blocked && zz_imminent()) need = 1;   // mapa avisa que o zigue-zague vem: reage na 1ª troca
    st_.zz_detected = (keep >= need && straightish) ? 1 : 0;
  }

  // ------------------------------------------------------------------ mapa: consultas
  int cur_index() const {                    // entrada (não-barra) do segmento em andamento
    for (int k = 0; k < 4; k++) { int j = (map_idx_ + k) % map_n_; if (map_[j].type != LF_SEG_BAR) return j; }
    return map_idx_ % map_n_;
  }
  int next_index(int from) const {           // próxima entrada não-barra depois de from
    for (int k = 1; k < 5; k++) { int j = (from + k) % map_n_; if (map_[j].type != LF_SEG_BAR) return j; }
    return (from + 1) % map_n_;
  }
  bool zz_imminent() const {
    if (!has_map_ || !map_n_) return false;
    int ci = cur_index();
    const lf_map_entry_t& cur = map_[ci];
    if (cur.type == LF_SEG_ZIGZAG) return true;
    const lf_map_entry_t& nx = map_[next_index(ci)];
    if (nx.type != LF_SEG_ZIGZAG) return false;
    if (cur.type == LF_SEG_STRAIGHT) return (cur.dist - st_.seg_dist) < 0.25f;
    if (is_curve(cur.type)) return absf(st_.seg_angle) > 0.8f * absf(cur.angle);
    return true;
  }

  // ------------------------------------------------------------------ segmentos
  void seg_begin(uint8_t type, float carry_angle) {
    st_.seg_type = type; st_.seg_angle = carry_angle; st_.seg_t = 0; st_.seg_dist = 0; st_.seg_kmax = st_.kappa_f;
    st_.t_above = 0; st_.t_below = 0; st_.prov = PROV_NONE;
  }

  void seg_end(uint8_t next_type, bool match = true) {
    float carry = 0;
    if (st_.seg_t >= g_.min_seg_time) push_entry(st_.seg_type, st_.seg_angle, st_.seg_t, st_.seg_dist, st_.seg_kmax, match);
    else carry = st_.seg_angle;
    seg_begin(next_type, carry);
  }

  void prov_start(uint8_t type) {
    float back = (type == LF_SEG_STRAIGHT) ? g_.t_straight : g_.t_curve;   // o trecho começou antes de ser confirmado
    if (back > st_.seg_t) back = st_.seg_t;
    st_.prov = type; st_.prov_t0 = st_.seg_t - back; st_.prov_d0 = st_.seg_dist - st_.v_est * back;
    st_.prov_a0 = st_.seg_angle - ((type == LF_SEG_STRAIGHT) ? 0.0f : st_.w_f * back);
    if (st_.prov_d0 < 0) st_.prov_d0 = 0;
  }

  // o trecho provisório virou segmento de verdade: fecha a parte anterior e continua com o provisório
  void prov_confirm() {
    uint8_t t = st_.prov;
    if (st_.prov_t0 >= g_.min_seg_time) {
      push_entry(st_.seg_type, st_.prov_a0, st_.prov_t0, st_.prov_d0, st_.seg_kmax, true);
      st_.seg_t -= st_.prov_t0; st_.seg_dist -= st_.prov_d0; st_.seg_angle -= st_.prov_a0;
    }
    st_.seg_type = t; st_.seg_kmax = st_.kappa_f;
    st_.prov = PROV_NONE;
  }

  void push_entry(uint8_t type, float angle, float dur, float dist, float kmax, bool match) {
    lf_map_entry_t e; e.type = type; e.angle = angle; e.duration = dur; e.dist = dist; e.kappa_max = kmax;
    if (!has_map_) {
      if (rec_n_ < LF_MAP_MAX) rec_[rec_n_++] = e;
      if (type == LF_SEG_BAR) { if (first_bar_ < 0) first_bar_ = rec_n_ - 1; }
      else check_cycle();
    } else if (type == LF_SEG_BAR) {
      sync_bar();
    } else if (match) {
      match_map(e);
    }
  }

  // Classificador com estado provisório: curva só confirma com >= min_curve_angle acumulado,
  // reta só confirma com >= min_straight_dist percorrido. Balanços do PID não viram segmento.
  void classify(float dt) {
    st_.seg_t += dt;
    st_.seg_angle += st_.w * dt;
    float kap = kappa();
    if (kap > g_.kappa_curve) st_.t_above += dt; else st_.t_above = 0;
    if (kap < g_.kappa_straight) st_.t_below += dt; else st_.t_below = 0;
    uint8_t sgn = curve_of(st_.w_f);
    uint8_t cur = st_.seg_type;
    if (st_.mode == LF_MODE_ZIGZAG) return;   // no zigue-zague o ω alternante não vira curva

    if (cur == LF_SEG_STRAIGHT) {
      if (st_.prov == PROV_NONE) {
        if (st_.t_above >= g_.t_curve) prov_start(sgn);
      } else {
        float da = st_.seg_angle - st_.prov_a0;
        bool confirmed = (st_.prov == LF_SEG_CURVE_L) ? (da >= g_.min_curve_angle) : (da <= -g_.min_curve_angle);
        if (confirmed) prov_confirm();
        else if (st_.t_below >= g_.t_straight) st_.prov = PROV_NONE;          // só um balanço
        else if (st_.t_above >= g_.t_curve && sgn != st_.prov) prov_start(sgn);
      }
    } else if (is_curve(cur)) {
      float da = (st_.prov == PROV_NONE) ? 0 : st_.seg_angle - st_.prov_a0;
      bool resumed = (cur == LF_SEG_CURVE_L) ? (da >= g_.min_curve_angle) : (da <= -g_.min_curve_angle);
      if (st_.prov == PROV_NONE) {
        if (st_.t_below >= g_.t_straight) prov_start(LF_SEG_STRAIGHT);
        else if (st_.t_above >= g_.t_curve && sgn != cur) prov_start(sgn);
      } else if (st_.prov == LF_SEG_STRAIGHT) {
        if (resumed) st_.prov = PROV_NONE;                                      // a curva continuou de verdade
        else if (st_.seg_dist - st_.prov_d0 >= g_.min_straight_dist) prov_confirm();
        else if (st_.t_above >= g_.t_curve && sgn != cur) st_.prov = sgn;       // virou para o outro lado (S)
      } else {                                                                  // curva no sentido oposto (S)
        bool confirmed = (st_.prov == LF_SEG_CURVE_L) ? (da >= g_.min_curve_angle) : (da <= -g_.min_curve_angle);
        if (confirmed) prov_confirm();
        else if (resumed) st_.prov = PROV_NONE;
        else if (st_.t_below >= g_.t_straight) st_.prov = LF_SEG_STRAIGHT;
      }
    }
  }

  // ao confirmar zigue-zague, as "curvinhas" recém-gravadas eram o próprio zigue-zague: absorve-as.
  // Um ZIGZAG gravado logo antes (saída precoce) também é absorvido, para virar um só segmento.
  void absorb_recent(float window) {
    if (has_map_) return;
    float acc_t = 0;
    while (rec_n_ > 0 && rec_[rec_n_ - 1].type != LF_SEG_BAR) {
      const lf_map_entry_t& r = rec_[rec_n_ - 1];
      if (r.type != LF_SEG_ZIGZAG && acc_t + r.duration > window) break;
      acc_t += r.duration; st_.seg_t += r.duration; st_.seg_dist += r.dist; st_.seg_angle += r.angle;
      rec_n_--;
      if (r.type == LF_SEG_ZIGZAG) break;
    }
  }

  void on_bar_event() {
    uint8_t cur = (st_.mode == LF_MODE_ZIGZAG) ? (uint8_t)LF_SEG_ZIGZAG : st_.seg_type;
    uint8_t prov = st_.prov;
    if (prov == LF_SEG_STRAIGHT && is_curve(st_.seg_type)) prov_confirm();   // a barra é um marco: a reta até ela é real
    seg_end(LF_SEG_BAR);                       // fecha o segmento corrente
    st_.seg_t = g_.min_seg_time;
    push_entry(LF_SEG_BAR, 0, 0, 0, 0, true);  // registra/sincroniza a barra
    (void)prov;
    seg_begin(cur == LF_SEG_ZIGZAG ? (uint8_t)LF_SEG_ZIGZAG : (uint8_t)LF_SEG_STRAIGHT, 0);   // reabre depois da barra
    if (cur != LF_SEG_ZIGZAG && kappa() > g_.kappa_curve) prov_start(curve_of(st_.w_f));
    if (st_.mode == LF_MODE_FOLLOW) enter_cross();
  }

  // ------------------------------------------------------------------ mapa: gravação, fechamento e casamento
  bool entry_match(const lf_map_entry_t& a, const lf_map_entry_t& b) const {
    if (a.type != b.type) return false;
    float dmax = a.dist > b.dist ? a.dist : b.dist;
    bool dist_ok = absf(a.dist - b.dist) < 0.35f * dmax + 0.05f;
    if (is_curve(a.type)) return absf(a.angle - b.angle) < g_.angle_tol && absf(a.dist - b.dist) < 0.5f * dmax + 0.08f;
    if (a.type == LF_SEG_STRAIGHT) return absf(a.angle) < 0.45f && absf(b.angle) < 0.45f && dist_ok;
    return true;
  }

  void check_cycle() {
    if (has_map_ || st_.t_lap < g_.min_lap_time) return;
    bool no_bar = first_bar_ < 0;
    if (no_bar && !g_.cycle_no_bar) return;
    int b0 = no_bar ? 1 : first_bar_;                 // sem barra: referência é o 1º segmento inteiro
    if (b0 >= rec_n_) return;
    int K = g_.cycle_k;
    for (int b = rec_n_ - K; b > b0; b--) {
      if (!no_bar && rec_[b].type != LF_SEG_BAR) continue;
      if (b0 + K > b) break;
      bool ok = true;
      for (int i = 0; i < K && ok; i++) ok = entry_match(rec_[b + i], rec_[b0 + i]);
      if (ok) { close_lap_at(b, K); return; }
    }
  }

  void close_lap_at(int b, int k_done) {
    int b0 = first_bar_ < 0 ? 1 : first_bar_;
    if (b0 >= rec_n_) b0 = 0;
    int n = b - b0;
    if (n <= 0) return;
    if (n > LF_MAP_MAX) n = LF_MAP_MAX;
    memcpy(map_, rec_ + b0, n * sizeof(lf_map_entry_t));
    map_n_ = n; has_map_ = g_.use_map != 0; map_dist_ = 0; for (int i = 0; i < n; i++) map_dist_ += map_[i].dist;
    map_idx_ = k_done % map_n_;
    st_.lap = 2; st_.t_lap = 0; st_.dist_lap = 0; mismatches_ = 0;
    for (int i = 0; i < k_done && b + i < rec_n_; i++) { st_.t_lap += rec_[b + i].duration; st_.dist_lap += rec_[b + i].dist; }
    rec_n_ = 0;
    if (g_.laps && st_.lap > g_.laps) { st_.mode = LF_MODE_FINISHED; pwm_cmd_ = 0; }   // laps=1: só mapear
  }

  // segmento ao vivo terminou: casa com a entrada corrente (ou até 3 à frente) e avança
  void match_map(const lf_map_entry_t& e) {
    if (!map_n_) return;
    for (int k = 0; k <= 3; k++) {
      int j = (map_idx_ + k) % map_n_;
      if (map_[j].type == LF_SEG_BAR) continue;
      if (entry_match(map_[j], e)) { advance_past(j); mismatches_ = 0; st_.zz_hint_blocked = 0; return; }
      if (map_[j].type == LF_SEG_ZIGZAG) break;      // zigue-zague só é ultrapassado pelo próprio modo zigue-zague
    }
    if (++mismatches_ >= 2) { if (map_[cur_index()].type != LF_SEG_ZIGZAG) advance_past(cur_index()); mismatches_ = 0; }
  }

  void advance_past(int j) {
    int nxt = j + 1;
    if (nxt >= map_n_) { nxt = 0; if (map_[0].type != LF_SEG_BAR && st_.dist_lap >= 0.5f * map_dist_) lap_tick(); }
    map_idx_ = nxt;
  }

  void sync_bar() {
    if (!map_n_) return;
    for (int k = 0; k <= 6; k++) {
      int j = (map_idx_ + k) % map_n_;
      if (map_[j].type == LF_SEG_BAR) {
        if (j == 0 || j < map_idx_) { if (st_.dist_lap < 0.5f * map_dist_) return; lap_tick(); }   // volta só fecha com distância plausível
        map_idx_ = (j + 1) % map_n_;
        mismatches_ = 0; st_.zz_hint_blocked = 0;
        return;
      }
    }
  }

  void lap_tick() {
    // volta a volta: limpa -> acelera um pouco; perdeu linha -> recua mais do que avançou
    if (has_map_ && g_.lap_boost > 0 && st_.lap >= 2) {
      if (lap_lost_t_ <= g_.lost_tol) boost_ = clampf(boost_ * (1.0f + g_.lap_boost), g_.boost_min, g_.boost_max);   // volta limpa: sobe
    }
    lap_lost_t_ = 0; lap_backed_ = false;
    st_.lap++;
    st_.t_lap = 0; st_.dist_lap = 0;
    if (g_.laps && st_.lap > g_.laps) { st_.mode = LF_MODE_FINISHED; pwm_cmd_ = 0; }
  }

  // ------------------------------------------------------------------ modos
  // Cruzamento: mantém o rumo E a taxa de giro que tinha (se estava em curva, continua a curva).
  void enter_cross() { st_.mode = LF_MODE_CROSS; st_.hold_target = st_.heading; st_.hold_rate = st_.w_f; st_.t_mode = 0; }
  void enter_zigzag(float target, bool match) {
    if (st_.seg_type != LF_SEG_ZIGZAG) {
      absorb_recent(g_.zz_window + g_.t_curve);
      // o zigue-zague começou ~meio período antes da 1ª reversão contada: divide o segmento ali
      float back = (st_.zz_swaps_n ? st_.t - st_.zz_swaps_t[0] : 0) + st_.zz_half;
      if (back > st_.seg_t) back = st_.seg_t;
      float d_back = st_.v_est * back; if (d_back > st_.seg_dist) d_back = st_.seg_dist;
      float t_before = st_.seg_t - back;
      uint8_t before_type = (is_curve(st_.seg_type) && absf(st_.seg_angle) < 0.45f) ? (uint8_t)LF_SEG_STRAIGHT : st_.seg_type;
      if (t_before >= g_.min_seg_time) push_entry(before_type, st_.seg_angle, t_before, st_.seg_dist - d_back, st_.seg_kmax, match);
      seg_begin(LF_SEG_ZIGZAG, 0);
      st_.seg_t = back; st_.seg_dist = d_back;
    }
    st_.mode = LF_MODE_ZIGZAG; st_.hold_target = target; st_.hold_rate = 0; st_.t_mode = 0;
    st_.zz_t_far = 0; st_.zz_fresh = 0; st_.zz_mid = 0; st_.zz_e_lp = st_.e_lp;   // centro só vale com reversões vistas em linha reta
    st_.zz_max_last = st_.zz_min_last = st_.zz_ext = st_.e; st_.zz_last_swap = st_.t;   // extremos antigos não valem mais
    if (has_map_ && map_n_)                                                   // índice do mapa cai na entrada ZIGZAG
      for (int k = 0; k <= 3; k++) { int j = (map_idx_ + k) % map_n_; if (map_[j].type == LF_SEG_ZIGZAG) { map_idx_ = j; mismatches_ = 0; break; } }
  }
  void exit_zigzag(bool lost) {
    st_.mode = lost ? (uint8_t)LF_MODE_LOST : (uint8_t)LF_MODE_FOLLOW; st_.t_mode = 0; st_.ie = 0;
    st_.zz_swaps_n = 0; st_.zz_detected = 0;       // precisa de reversões novas para detectar de novo
    if (lost && st_.seg_t < 0.15f) st_.zz_hint_blocked = 1;   // entrou por dica do mapa e perdeu a linha: dica errada
    seg_end(LF_SEG_STRAIGHT);
    if (kappa() > g_.kappa_curve) prov_start(curve_of(st_.w_f));
  }

  void update_modes(float dt) {
    st_.t_mode += dt;
    if (!st_.on_line) st_.t_noline += dt; else st_.t_noline = 0;
    switch (st_.mode) {
      case LF_MODE_FOLLOW:
        if (st_.t_noline > g_.lost_time) { st_.mode = LF_MODE_LOST; st_.t_mode = 0; break; }
        if (st_.zz_detected) {
          // rumo alvo = média do rumo ao longo de >= 1 período inteiro (sem viés da fase do balanço);
          // com poucas reversões (dica do mapa), só entra assentado e usa o rumo atual
          bool settled = absf(st_.w_f) < 1.5f && absf(st_.e) < 0.4f;
          float span = st_.t - st_.zz_swaps_t[0];
          if (st_.zz_swaps_n >= 3 && span > 0.05f) enter_zigzag((st_.h_int - st_.zz_swaps_hi[0]) / span, true);   // média exata do rumo na janela
          else if (settled) enter_zigzag(st_.heading, true);
        } else if (has_map_ && map_n_ && !st_.zz_hint_blocked && map_[cur_index()].type == LF_SEG_ZIGZAG
                   && st_.seg_type != LF_SEG_ZIGZAG && absf(st_.w_f) < 1.5f && absf(st_.e) < 0.4f) {
          enter_zigzag(st_.heading, false);   // mapa diz que o zigue-zague começa agora
        }
        break;
      case LF_MODE_ZIGZAG: {
        { float lost_lim = 1.2f * st_.zz_half; if (lost_lim < 0.1f) lost_lim = 0.1f;
          if (st_.t_noline > lost_lim) { exit_zigzag(true); break; } }       // some num vértice é normal; some por um período não
        float quiet = st_.t - st_.zz_last_swap;
        float lim = g_.zz_exit_time > 1.5f * st_.zz_half ? g_.zz_exit_time : 1.5f * st_.zz_half;
        // linha fugindo para um lado por mais de 1,5 meio-período: o zigue-zague acabou numa curva
        st_.zz_t_far = (absf(st_.e - zz_center()) > 0.7f) ? st_.zz_t_far + dt : 0.0f;
        bool drifting = st_.zz_half > 0 && st_.zz_t_far > 1.5f * st_.zz_half && st_.t_mode > 0.05f;
        // com mapa: distância do zigue-zague cumprida
        bool done_by_map = has_map_ && map_n_ && map_[cur_index()].type == LF_SEG_ZIGZAG
                           && map_[cur_index()].dist > 0.1f && st_.seg_dist >= map_[cur_index()].dist - 0.03f;
        if (absf(wrap_pi(st_.hold_target - st_.heading)) > 0.7f) { exit_zigzag(false); break; }   // alvo errado: volta a seguir linha
        if ((quiet > lim && st_.t_mode > lim) || drifting || done_by_map) exit_zigzag(false);
        st_.hold_target -= g_.ki_zz * zz_center() * dt;   // centro deslocado p/ direita -> rumo alvo gira p/ direita
        break;
      }
      case LF_MODE_CROSS:
        st_.hold_target += st_.hold_rate * dt;
        if ((!st_.bar_active && st_.t_since_bar > g_.cross_hold_time && st_.on_line && st_.n_black <= 3)
            || st_.t_mode > g_.cross_max_time) {
          st_.mode = LF_MODE_FOLLOW; st_.t_mode = 0;
        }
        break;
      case LF_MODE_LOST:
        if (st_.on_line) { st_.mode = LF_MODE_FOLLOW; st_.t_mode = 0; st_.ie = 0; }
        break;
      default: break;
    }
  }

  // + = virar à direita (esquerda acelera, direita freia)
  float steering(float dt) {
    switch (st_.mode) {
      case LF_MODE_FOLLOW: {
        st_.ie = clampf(st_.ie + st_.e * dt, -0.5f, 0.5f);
        if (ext_enable_) return ext_steer_;
        float kp = g_.kp + g_.kp_v * pwm_cmd_;
        return kp * st_.e + g_.ki * st_.ie + g_.kd * st_.e_dot + g_.kgyro * st_.w_f;
      }
      case LF_MODE_ZIGZAG: {
        float he = clampf(wrap_pi(st_.hold_target - st_.heading), -0.5f, 0.5f);
        float s = -g_.kh * he + g_.kh_gyro * st_.w_f + g_.kp_zz * zz_center();
        if (!st_.on_line) s += g_.kp_zz * st_.e;        // linha saiu do array: puxa para o lado em que sumiu
        return s;
      }
      case LF_MODE_CROSS: {
        float he = wrap_pi(st_.hold_target - st_.heading);
        return -g_.kh * he + g_.kh_gyro * (st_.w_f - st_.hold_rate);
      }
      case LF_MODE_LOST:
        return (st_.e >= 0 ? 1.0f : -1.0f) * g_.v_lost;
      default: return 0;
    }
  }

  float k_eff() const { return st_.k > 1e-5f ? st_.k : g_.k_pwm2v; }
  float zz_center() const { return st_.zz_fresh >= 2 ? 0.5f * (st_.zz_mid + st_.zz_e_lp) : st_.zz_e_lp; }   // desvio do centro da oscilação
  float kappa() const { float v = st_.v_est > 0.3f ? st_.v_est : 0.3f; return absf(st_.w_f) / v; }   // curvatura estimada 1/m

  float curve_speed(const lf_map_entry_t& m) const {      // m/s (boost escala a aceleração lateral admitida)
    float a_lat = g_.a_lat_max * boost_;
    if (m.kappa_max > 0.5f) return sqrtf(a_lat / m.kappa_max);
    float a = absf(m.angle);
    if (a < 0.05f || m.dist <= 0) return g_.v_straight * boost_ * k_eff();
    return sqrtf(a_lat * (m.dist / a));
  }
  float seg_speed(const lf_map_entry_t& m) const {        // m/s alvo numa entrada
    if (m.type == LF_SEG_ZIGZAG) return g_.v_zigzag;
    if (is_curve(m.type)) return curve_speed(m);
    return g_.v_straight * boost_ * k_eff();
  }

  float speed_cmd(float dt) {
    float target;
    if (st_.mode == LF_MODE_LOST) target = g_.v_lost;   // avança procurando a linha em arco, não para
    else if (!has_map_ || !map_n_) {
      target = g_.v_base;
      if (st_.mode == LF_MODE_ZIGZAG) { float z = g_.v_zigzag / k_eff(); if (z < target) target = z; }
    } else {
      int ci = cur_index();
      const lf_map_entry_t& cur = map_[ci];
      float v_t = seg_speed(cur);
      if (cur.type == LF_SEG_STRAIGHT) {
        // reta: só o que dá pra frear antes do próximo segmento lento
        float remaining = cur.dist - st_.seg_dist - g_.brake_margin;
        if (remaining < 0) remaining = 0;
        float v_next = seg_speed(map_[next_index(ci)]);
        float v_allow = sqrtf(v_next * v_next + 2.0f * g_.a_brake * remaining);
        if (v_allow < v_t) v_t = v_allow;
      }
      target = clampf(v_t / k_eff(), g_.v_min_curve, g_.v_max);
    }
    if (st_.mode == LF_MODE_FOLLOW) target *= (1.0f - g_.kv_e * absf(st_.e));
    if (g_.react_kappa > 0 && st_.mode == LF_MODE_FOLLOW && st_.kappa_f > 0.5f) {   // curva sentida no giro: respeita a aderência
      float cap = sqrtf(g_.a_lat_max / st_.kappa_f) / k_eff();
      if (cap < g_.v_min_curve) cap = g_.v_min_curve;
      if (target > cap) target = cap;
    }
    if (ext_enable_ && st_.mode == LF_MODE_FOLLOW) target = clampf(target * ext_scale_, 0.0f, g_.v_max);
    float d = target - pwm_cmd_;
    float lim = d > 0 ? g_.ramp_up * dt : g_.ramp_down * dt;
    pwm_cmd_ += clampf(d, -lim, lim);
    return pwm_cmd_;
  }

  void mix(float v, float steer, lf_cmd_t& out) const {
    float l = v + steer, r = v - steer, vm = g_.v_max;
    if (l > vm) { r -= (l - vm); l = vm; }      // satura preservando o diferencial
    if (r > vm) { l -= (r - vm); r = vm; }
    l = clampf(l, -vm, vm); r = clampf(r, -vm, vm);
    out.left = (int16_t)lrintf(l); out.right = (int16_t)lrintf(r);
  }
};

}  // namespace

// ---------------------------------------------------------------------- C API
extern "C" {

void lf_default_gains(lf_gains_t* g) {
  memset(g, 0, sizeof(*g));
  g->kp = 40.0f; g->ki = 5.0f; g->kd = 1.2f; g->kgyro = 6.0f; g->kp_v = 1.0f; g->kv_e = 0.35f; g->react_kappa = 1.0f;
  g->kh = 220.0f; g->kh_gyro = 10.0f; g->kp_zz = 60.0f; g->ki_zz = 3.0f;
  g->v_base = 80; g->v_max = 255; g->v_straight = 240; g->v_min_curve = 45; g->v_zigzag = 1.2f; g->v_lost = 90;
  g->ramp_up = 1200; g->ramp_down = 5000;
  g->a_lat_max = 6.5f; g->a_brake = 5.0f; g->brake_margin = 0.12f; g->k_pwm2v = 0.0075f; g->tau_motor = 0.06f;
  g->lap_boost = 0.08f; g->lap_backoff = 0.15f; g->boost_max = 1.25f; g->boost_min = 0.7f; g->lost_tol = 0.1f;
  g->kappa_curve = 2.0f; g->kappa_straight = 1.0f; g->t_curve = 0.04f; g->t_straight = 0.08f; g->min_seg_time = 0.03f;
  g->min_curve_angle = 0.40f; g->min_straight_dist = 0.10f;
  g->zz_hyst = 0.35f; g->zz_amp = 0.7f; g->zz_max_turn = 0.35f; g->zz_window = 0.6f; g->zz_exit_time = 0.12f;
  g->cross_hold_time = 0.05f; g->cross_max_time = 0.25f; g->lost_time = 0.15f;
  g->on_line_hi = 280; g->on_line_lo = 150;
  g->min_lap_time = 8.0f; g->angle_tol = 0.5f;
  g->zz_min_swaps = 3; g->bar_min_black = 7; g->laps = 0; g->cycle_k = 7; g->cycle_no_bar = 1; g->use_map = 1;
}

lf_handle_t lf_create(void) { return new Core(); }
void lf_destroy(lf_handle_t h) { delete (Core*)h; }
void lf_reset(lf_handle_t h) { ((Core*)h)->reset(); }
void lf_set_gains(lf_handle_t h, const lf_gains_t* g) { ((Core*)h)->g_ = *g; }
void lf_get_gains(lf_handle_t h, lf_gains_t* g) { *g = ((Core*)h)->g_; }
void lf_start(lf_handle_t h) { ((Core*)h)->start(); }
void lf_stop(lf_handle_t h) { ((Core*)h)->stop(); }
void lf_step(lf_handle_t h, const lf_frame_t* f, lf_cmd_t* out) { *out = ((Core*)h)->step(*f); }
void lf_get_telem(lf_handle_t h, lf_telem_t* t) { ((Core*)h)->telem(*t); }
int lf_get_map(lf_handle_t h, lf_map_entry_t* out, int max) { return ((Core*)h)->get_map(out, max); }
int lf_get_rec(lf_handle_t h, lf_map_entry_t* out, int max) { return ((Core*)h)->get_rec(out, max); }
void lf_set_map(lf_handle_t h, const lf_map_entry_t* in, int n) { ((Core*)h)->set_map(in, n); }
void lf_lap_boundary(lf_handle_t h) { ((Core*)h)->lap_boundary(); }
void lf_set_gyro_bias(lf_handle_t h, float b) { ((Core*)h)->bias_ = b; }
float lf_get_gyro_bias(lf_handle_t h) { return ((Core*)h)->bias_; }
void lf_set_external(lf_handle_t h, float steer, float scale, int en) { ((Core*)h)->set_external(steer, scale, en != 0); }
void lf_struct_sizes(int* o) { o[0] = (int)sizeof(lf_gains_t); o[1] = (int)sizeof(lf_telem_t); o[2] = (int)sizeof(lf_map_entry_t); o[3] = (int)sizeof(lf_frame_t); }

}
