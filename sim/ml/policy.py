"""Política neural pequena (MLP em numpy) que substitui o PID de direção no modo FOLLOW.
Entradas: 8 sensores, posição da linha, derivada, giro, v estimada, steer anterior, linha visível.
Saídas: steer (pwm, ±steer_max) e escala da velocidade alvo do mapa (0,6..1,4)."""
import numpy as np

N_IN = 14


class MLPPolicy:
    def __init__(self, n_hidden=16, steer_max=220.0):
        self.nh, self.steer_max = n_hidden, steer_max
        self.shapes = [(n_hidden, N_IN), (n_hidden,), (2, n_hidden), (2,)]
        self.n_params = sum(int(np.prod(s)) for s in self.shapes)
        self.set_params(np.zeros(self.n_params))
        self.reset()

    def reset(self):
        self.prev_steer, self.prev_e = 0.0, 0.0

    def set_params(self, vec):
        vec = np.asarray(vec, float)
        assert vec.size == self.n_params
        i, out = 0, []
        for s in self.shapes:
            n = int(np.prod(s)); out.append(vec[i:i + n].reshape(s)); i += n
        self.W1, self.b1, self.W2, self.b2 = out

    def get_params(self):
        return np.concatenate([p.ravel() for p in (self.W1, self.b1, self.W2, self.b2)])

    @staticmethod
    def pid_init(n_hidden=16, steer_max=220.0, kp=150.0, kd=1.2, kgyro=6.0):
        """Pesos que reproduzem (aprox.) o PID linear: ponto de partida 'quente' para o ES."""
        p = MLPPolicy(n_hidden, steer_max)
        W1 = np.zeros((n_hidden, N_IN)); b1 = np.zeros(n_hidden); W2 = np.zeros((2, n_hidden)); b2 = np.zeros(2)
        a = 0.5                                   # tanh(a·x) ≈ a·x na região linear
        W1[0, 8] = a;  W2[0, 0] = kp / (steer_max * a)            # e
        W1[1, 9] = a;  W2[0, 1] = kd * 50.0 / (steer_max * a)     # e_dot (x9 = e_dot/50)
        W1[2, 10] = a; W2[0, 2] = kgyro * 10.0 / (steer_max * a)  # giro (x10 = ω/10)
        p.W1, p.b1, p.W2, p.b2 = W1, b1, W2, b2
        return p

    def __call__(self, s, gyro, ay, tm, dt):
        x = np.empty(N_IN)
        x[:8] = np.asarray(s, float) / 1000.0
        e = float(tm.line_pos)
        x[8] = e
        x[9] = (e - self.prev_e) / max(dt, 1e-3) / 50.0
        x[10] = gyro / 10.0
        x[11] = float(tm.v_est) / 2.0
        x[12] = self.prev_steer / self.steer_max
        x[13] = float(tm.on_line)
        h = np.tanh(self.W1 @ x + self.b1)
        o = self.W2 @ h + self.b2
        steer = self.steer_max * float(np.tanh(o[0]))
        scale = 1.0 + 0.4 * float(np.tanh(o[1]))
        self.prev_steer, self.prev_e = steer, e
        return steer, scale


# ---- vetor de ganhos do controlador clássico, normalizado em [0,1] para o ES ----
GAIN_BOUNDS = {
    'kp': (10.0, 150.0), 'kp_v': (0.2, 2.5), 'kd': (0.0, 5.0), 'kgyro': (0.0, 20.0), 'kv_e': (0.0, 0.8),
    'a_lat_max': (3.0, 9.5), 'a_brake': (2.0, 9.0), 'brake_margin': (0.0, 0.25), 'v_straight': (120.0, 255.0),
    'v_zigzag': (0.6, 2.0), 'kh': (80.0, 400.0), 'kp_zz': (10.0, 150.0),
}
GAIN_NAMES = list(GAIN_BOUNDS)


def gains_from_vec(vec01):
    """vec em R^n (sem limite) -> dict de ganhos dentro dos limites (sigmoide)."""
    out = {}
    for name, v in zip(GAIN_NAMES, vec01):
        lo, hi = GAIN_BOUNDS[name]
        out[name] = lo + (hi - lo) / (1.0 + np.exp(-float(v)))
    return out


def vec_from_gains(g):
    """inversa de gains_from_vec (para inicializar o ES nos ganhos padrão)."""
    v = []
    for name in GAIN_NAMES:
        lo, hi = GAIN_BOUNDS[name]
        p = np.clip((g[name] - lo) / (hi - lo), 0.02, 0.98)
        v.append(np.log(p / (1 - p)))
    return np.array(v)
