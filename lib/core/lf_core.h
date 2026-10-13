// lf_core.h — núcleo de controle do seguidor de linha, sem dependência de Arduino.
// Mesmo código roda no ESP32-S3 (firmware) e no PC (testes nativos + simulador Python via ctypes).
//
// Convenções:
//   sensores s[0..7]  : 0 = esquerda, 7 = direita; valor calibrado 0..1000 (1000 = preto)
//   gyro_z            : rad/s, positivo = girando para a esquerda (anti-horário visto de cima)
//   acc_y             : m/s², positivo = para a esquerda (aceleração lateral = v·ω em curva)
//   saída left/right  : pwm -v_max..+v_max (negativo = ré)
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { LF_N_SENS = 8, LF_MAP_MAX = 96 };

typedef enum { LF_SEG_STRAIGHT = 0, LF_SEG_CURVE_L = 1, LF_SEG_CURVE_R = 2, LF_SEG_ZIGZAG = 3, LF_SEG_BAR = 4 } lf_seg_t;
typedef enum { LF_MODE_IDLE = 0, LF_MODE_FOLLOW = 1, LF_MODE_ZIGZAG = 2, LF_MODE_CROSS = 3,
               LF_MODE_LOST = 4, LF_MODE_FINISHED = 5 } lf_mode_t;

typedef struct {
  uint16_t s[LF_N_SENS];
  float gyro_z;   // rad/s
  float acc_x;    // m/s² (frente)
  float acc_y;    // m/s² (esquerda)
  float dt;       // s desde o frame anterior
} lf_frame_t;

typedef struct { int16_t left, right; } lf_cmd_t;

// ATENÇÃO: a ordem dos campos é espelhada em sim/lfsim/core_bridge.py (ctypes). Mudou aqui, muda lá.
typedef struct {
  // --- seguidor de linha (modo FOLLOW) ---
  float kp, ki, kd, kgyro;
  float kp_v;            // ganho P extra proporcional ao pwm comandado (pure pursuit: Δv ∝ v)
  float kv_e;            // redução de velocidade proporcional a |erro| (0..1)
  float react_kappa;     // 1 = limita v a sqrt(a_lat_max/κ_medida) mesmo sem mapa (freio reativo por curvatura)
  // --- heading hold (ZIGZAG / CROSS) ---
  float kh, kh_gyro, kp_zz;
  float ki_zz;           // rad/s por unidade de desvio do centro: adapta o rumo alvo no zigue-zague
  // --- velocidades (pwm) ---
  float v_base;          // volta de mapeamento / sem mapa
  float v_max;           // teto absoluto
  float v_straight;      // reta com mapa
  float v_min_curve;     // piso em curva com mapa
  float v_zigzag;        // no zigue-zague
  float v_lost;          // pivô ao perder a linha
  float ramp_up, ramp_down;  // pwm/s
  // --- perfil de velocidade com mapa ---
  float a_lat_max;       // m/s² lateral permitido (v_curva = sqrt(a_lat_max·R))
  float a_brake;         // m/s² de frenagem assumida
  float brake_margin;    // m de folga na frenagem
  float k_pwm2v;         // m/s por unidade de pwm (chute inicial; recalibrado nas curvas via a_lat/ω)
  float tau_motor;       // s, atraso do motor assumido na estimativa de v
  // --- aprendizado volta a volta (só com mapa) ---
  float lap_boost;       // volta limpa (sem perder linha): a_lat_max e v_straight sobem esta fração (0 = desliga)
  float lap_backoff;     // volta com perda de linha: recuam esta fração
  float boost_max;       // teto do multiplicador (1.5 = até +50 %)
  float boost_min;       // piso do multiplicador
  float lost_tol;        // s perdido na volta até onde ela ainda conta como limpa
  // --- classificador de segmento ---
  float kappa_curve, kappa_straight;   // 1/m: |ω|/v acima de kappa_curve = curva, abaixo de kappa_straight = reta (invariante à velocidade)
  float t_curve, t_straight;   // s de confirmação
  float min_seg_time;          // s, segmentos menores são descartados (ângulo é carregado)
  float min_curve_angle;       // rad: curva só é confirmada depois de acumular isso (filtra balanço do PID)
  float min_straight_dist;     // m: reta só é confirmada depois de percorrer isso
  float zz_hyst;               // histerese (em e) para reconhecer pico/vale da posição da linha
  float zz_amp;                // amplitude pico-a-pico mínima (em e) para a reversão contar como zigue-zague
  float zz_max_turn;           // rad: rumo líquido máximo entre a 1ª e a última reversão (curva de verdade gira)
  float zz_window;             // s, janela de contagem de trocas
  float zz_exit_time;          // s sem troca de lado para sair do zigue-zague
  float cross_hold_time;       // s de heading hold após a barra sumir
  float cross_max_time;        // s máximo em CROSS
  float lost_time;             // s sem linha para entrar em LOST
  float min_lap_time;          // s mínimo antes de aceitar fechamento de volta
  float angle_tol;             // rad de tolerância ao casar curvas com o mapa
  uint8_t zz_min_swaps;        // trocas de lado para confirmar zigue-zague
  uint8_t bar_min_black;       // sensores pretos para evento "barra"
  uint8_t laps;                // voltas a correr (0 = sem fim)
  uint8_t cycle_k;             // entradas comparadas para detectar fechamento de volta
  uint8_t cycle_no_bar;        // 1 = fecha volta por repetição mesmo sem barra/cruzamento na pista
  uint8_t use_map;             // 0 = ignora o mapa (só PID na velocidade base), 1 = perfil de velocidade e dicas
} lf_gains_t;

typedef struct {
  uint8_t type;     // lf_seg_t
  float angle;      // rad acumulado no segmento (+ esquerda)
  float duration;   // s (na volta de mapeamento)
  float dist;       // m estimado
  float kappa_max;  // 1/m, curvatura máxima vista no segmento (define a velocidade da curva)
} lf_map_entry_t;

typedef struct {
  float line_pos;   // -1..1 (+ = linha à direita do centro)
  float heading;    // rad (sem wrap)
  float omega_f;    // rad/s filtrado
  float v_est;      // m/s estimado por pwm·k
  float dist_lap;   // m desde o início da volta
  float t_lap;      // s desde o início da volta
  float steer;      // esforço de direção (pwm, + = direita)
  float k_pwm2v;    // calibração atual
  float seg_angle;  // ângulo acumulado no segmento ao vivo
  float v_target;   // pwm alvo
  float zz_target;  // rumo alvo do heading hold
  float zz_half;    // s, meio-período medido do zigue-zague
  float zz_mid;     // centro da oscilação (em e)
  float zz_t_far;   // s com a linha longe do centro
  float boost;      // multiplicador atual do perfil de velocidade (aprendizado por volta)
  uint8_t mode, seg, map_idx, map_len, lap, has_map, on_line, bars, n_black, map_seg, zz_swaps;
} lf_telem_t;

typedef void* lf_handle_t;

lf_handle_t lf_create(void);
void   lf_destroy(lf_handle_t h);
void   lf_reset(lf_handle_t h);                    // zera tudo (mapa inclusive)
void   lf_default_gains(lf_gains_t* g);
void   lf_set_gains(lf_handle_t h, const lf_gains_t* g);
void   lf_get_gains(lf_handle_t h, lf_gains_t* g);
void   lf_start(lf_handle_t h);                    // IDLE -> FOLLOW; começa volta/mapeamento
void   lf_stop(lf_handle_t h);                     // -> IDLE
void   lf_step(lf_handle_t h, const lf_frame_t* f, lf_cmd_t* out);
void   lf_get_telem(lf_handle_t h, lf_telem_t* t);
int    lf_get_map(lf_handle_t h, lf_map_entry_t* out, int max);   // retorna n (mapa fechado) ou gravação em curso
int    lf_get_rec(lf_handle_t h, lf_map_entry_t* out, int max);   // segmentos gravados na volta atual
void   lf_set_map(lf_handle_t h, const lf_map_entry_t* in, int n);// mapa pré-programado / salvo em NVS
void   lf_lap_boundary(lf_handle_t h);             // força fechamento de volta agora (sync manual)
void   lf_set_gyro_bias(lf_handle_t h, float bias);
float  lf_get_gyro_bias(lf_handle_t h);
void   lf_struct_sizes(int* out4);                 // sizeof de gains, telem, map_entry, frame (checagem do ctypes)
// controlador externo (ex.: política aprendida) substitui o PID de direção no modo FOLLOW e escala a velocidade alvo
void   lf_set_external(lf_handle_t h, float steer, float speed_scale, int enable);

#ifdef __cplusplus
}
#endif
