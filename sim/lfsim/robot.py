"""Modelo do carrinho: diferencial 2D, motores com atraso, limite de aderência, IMU e array de sensores."""
from dataclasses import dataclass

import numpy as np


@dataclass
class RobotParams:
    wheelbase: float = 0.14      # m entre as rodas
    sensor_fwd: float = 0.09     # m do eixo ao array de sensores
    pitch: float = 0.009525      # m entre sensores (QTR-8A)
    length: float = 0.16         # m, só para desenhar
    v_max_motor: float = 2.2     # m/s com pwm 255 e bateria cheia
    v_nom: float = 8.4           # V bateria cheia (2S)
    batt_sag: float = 0.004      # V/s de queda ao longo da corrida
    tau_motor: float = 0.06      # s, atraso de 1ª ordem do motor
    mu_g: float = 10.0           # m/s², aceleração lateral máxima (μ·g); acima disso derrapa
    gyro_bias: float = 0.02      # rad/s
    gyro_noise: float = 0.015    # rad/s (σ)
    acc_noise: float = 0.4       # m/s² (σ)
    sens_noise: float = 15.0     # σ em unidades 0..1000


class Robot:
    def __init__(self, p=None, rng=None):
        self.p = p or RobotParams()
        self.rng = rng or np.random.default_rng(0)
        self.x = self.y = self.th = 0.0
        self.vl = self.vr = 0.0
        self.v = self.w = self.v_prev = 0.0
        self.t = 0.0
        self.slip = False

    def set_pose(self, x, y, th):
        self.x, self.y, self.th = float(x), float(y), float(th)

    def kv(self):
        vbat = max(self.p.v_nom - self.p.batt_sag * self.t, 6.0)
        return self.p.v_max_motor / 255.0 * vbat / self.p.v_nom

    def step(self, pwm_l, pwm_r, dt):
        kv = self.kv()
        a = dt / (self.p.tau_motor + dt)
        self.vl += a * (kv * pwm_l - self.vl)
        self.vr += a * (kv * pwm_r - self.vr)
        v = 0.5 * (self.vl + self.vr)
        w = (self.vr - self.vl) / self.p.wheelbase
        w_lim = self.p.mu_g / max(abs(v), 0.05)
        self.slip = abs(w) > w_lim
        w = float(np.clip(w, -w_lim, w_lim))       # sem aderência: sub-esterça
        self.th += w * dt
        self.x += v * np.cos(self.th) * dt
        self.y += v * np.sin(self.th) * dt
        self.v_prev, self.v, self.w = self.v, v, w
        self.t += dt

    def sensor_center(self):
        return np.array([self.x + self.p.sensor_fwd * np.cos(self.th), self.y + self.p.sensor_fwd * np.sin(self.th)])

    def sensor_positions(self):
        c = self.sensor_center()
        n_left = np.array([-np.sin(self.th), np.cos(self.th)])
        offs = (3.5 - np.arange(8)) * self.p.pitch          # sensor 0 = mais à esquerda
        return c[None, :] + offs[:, None] * n_left[None, :]

    def imu(self, dt):
        p, r = self.p, self.rng
        gyro = self.w + p.gyro_bias + r.normal(0, p.gyro_noise)
        acc_y = self.v * self.w + r.normal(0, p.acc_noise)
        acc_x = (self.v - self.v_prev) / dt + r.normal(0, p.acc_noise)
        return float(gyro), float(acc_x), float(acc_y)

    def footprint(self):
        L, W = self.p.length, self.p.wheelbase
        body = np.array([[-0.05, -W / 2], [L - 0.05, -W / 2], [L - 0.05, W / 2], [-0.05, W / 2]])
        c, s = np.cos(self.th), np.sin(self.th)
        R = np.array([[c, -s], [s, c]])
        return body @ R.T + np.array([self.x, self.y])
