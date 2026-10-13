"""Ponte ctypes para lib/core (lf_core.h). Compila a .so automaticamente se faltar ou estiver velha."""
import ctypes as C
import os
import subprocess

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.dirname(HERE)
ROOT = os.path.dirname(SIM_DIR)
CORE_DIR = os.path.join(ROOT, 'lib', 'core')
LIB_PATH = os.path.join(SIM_DIR, 'build', 'liblfcore.so')

N_SENS, MAP_MAX = 8, 96
SEG_STRAIGHT, SEG_CURVE_L, SEG_CURVE_R, SEG_ZIGZAG, SEG_BAR = range(5)
MODE_IDLE, MODE_FOLLOW, MODE_ZIGZAG, MODE_CROSS, MODE_LOST, MODE_FINISHED = range(6)
SEG_NAMES = {0: 'RETA', 1: 'CURVA_E', 2: 'CURVA_D', 3: 'ZIGZAG', 4: 'BARRA', 255: '-'}
MODE_NAMES = {0: 'IDLE', 1: 'FOLLOW', 2: 'ZIGZAG', 3: 'CROSS', 4: 'LOST', 5: 'FINISHED'}


class Frame(C.Structure):
    _fields_ = [('s', C.c_uint16 * N_SENS), ('gyro_z', C.c_float), ('acc_x', C.c_float),
                ('acc_y', C.c_float), ('dt', C.c_float)]


class Cmd(C.Structure):
    _fields_ = [('left', C.c_int16), ('right', C.c_int16)]


# ESPELHO de lf_gains_t — mesma ordem do lf_core.h
GAIN_F = ['kp', 'ki', 'kd', 'kgyro', 'kp_v', 'kv_e', 'react_kappa', 'kh', 'kh_gyro', 'kp_zz', 'ki_zz',
          'v_base', 'v_max', 'v_straight', 'v_min_curve', 'v_zigzag', 'v_lost', 'ramp_up', 'ramp_down',
          'a_lat_max', 'a_brake', 'brake_margin', 'k_pwm2v', 'tau_motor', 'lap_boost', 'lap_backoff', 'boost_max', 'boost_min', 'lost_tol',
          'kappa_curve', 'kappa_straight', 't_curve', 't_straight', 'min_seg_time', 'min_curve_angle', 'min_straight_dist',
          'zz_hyst', 'zz_amp', 'zz_max_turn', 'zz_window',
          'zz_exit_time', 'cross_hold_time', 'cross_max_time', 'lost_time', 'min_lap_time', 'angle_tol']
GAIN_U8 = ['zz_min_swaps', 'bar_min_black', 'laps', 'cycle_k', 'cycle_no_bar', 'use_map']


class Gains(C.Structure):
    _fields_ = [(n, C.c_float) for n in GAIN_F] + [(n, C.c_uint8) for n in GAIN_U8]

    def as_dict(self):
        return {n: getattr(self, n) for n in GAIN_F + GAIN_U8}


class MapEntry(C.Structure):
    _fields_ = [('type', C.c_uint8), ('angle', C.c_float), ('duration', C.c_float), ('dist', C.c_float),
                ('kappa_max', C.c_float)]

    def as_dict(self):
        return {'type': self.type, 'name': SEG_NAMES.get(self.type, '?'), 'angle': self.angle,
                'duration': self.duration, 'dist': self.dist, 'kappa_max': self.kappa_max}


TELEM_F = ['line_pos', 'heading', 'omega_f', 'v_est', 'dist_lap', 't_lap', 'steer', 'k_pwm2v',
           'seg_angle', 'v_target', 'zz_target', 'zz_half', 'zz_mid', 'zz_t_far', 'boost']
TELEM_U8 = ['mode', 'seg', 'map_idx', 'map_len', 'lap', 'has_map', 'on_line', 'bars', 'n_black', 'map_seg', 'zz_swaps']


class Telem(C.Structure):
    _fields_ = [(n, C.c_float) for n in TELEM_F] + [(n, C.c_uint8) for n in TELEM_U8]


def build_lib(force=False):
    src = os.path.join(CORE_DIR, 'lf_core.cpp')
    hdr = os.path.join(CORE_DIR, 'lf_core.h')
    if (not force and os.path.exists(LIB_PATH)
            and os.path.getmtime(LIB_PATH) >= max(os.path.getmtime(src), os.path.getmtime(hdr))):
        return LIB_PATH
    os.makedirs(os.path.dirname(LIB_PATH), exist_ok=True)
    subprocess.check_call(['g++', '-std=c++11', '-O2', '-fPIC', '-shared', '-I', CORE_DIR, src, '-o', LIB_PATH])
    return LIB_PATH


_lib = None


def lib():
    global _lib
    if _lib is not None:
        return _lib
    L = C.CDLL(build_lib())
    H = C.c_void_p
    L.lf_create.restype = H
    L.lf_destroy.argtypes = [H]
    L.lf_reset.argtypes = [H]
    L.lf_default_gains.argtypes = [C.POINTER(Gains)]
    L.lf_set_gains.argtypes = [H, C.POINTER(Gains)]
    L.lf_get_gains.argtypes = [H, C.POINTER(Gains)]
    L.lf_start.argtypes = [H]
    L.lf_stop.argtypes = [H]
    L.lf_step.argtypes = [H, C.POINTER(Frame), C.POINTER(Cmd)]
    L.lf_get_telem.argtypes = [H, C.POINTER(Telem)]
    L.lf_get_map.argtypes = [H, C.POINTER(MapEntry), C.c_int]
    L.lf_get_map.restype = C.c_int
    L.lf_get_rec.argtypes = [H, C.POINTER(MapEntry), C.c_int]
    L.lf_get_rec.restype = C.c_int
    L.lf_set_map.argtypes = [H, C.POINTER(MapEntry), C.c_int]
    L.lf_lap_boundary.argtypes = [H]
    L.lf_set_gyro_bias.argtypes = [H, C.c_float]
    L.lf_get_gyro_bias.argtypes = [H]
    L.lf_get_gyro_bias.restype = C.c_float
    L.lf_struct_sizes.argtypes = [C.POINTER(C.c_int)]
    L.lf_set_external.argtypes = [H, C.c_float, C.c_float, C.c_int]
    sz = (C.c_int * 4)()
    L.lf_struct_sizes(sz)
    expect = [C.sizeof(Gains), C.sizeof(Telem), C.sizeof(MapEntry), C.sizeof(Frame)]
    if list(sz) != expect:
        raise RuntimeError(f'structs do ctypes não batem com lf_core.h: C={list(sz)} py={expect}')
    _lib = L
    return L


class Core:
    """Instância do controlador (mesmo código que roda no ESP32)."""

    def __init__(self):
        self._lib = lib()
        self.h = self._lib.lf_create()
        self._f = Frame()
        self._c = Cmd()
        self._t = Telem()
        self._buf = (MapEntry * MAP_MAX)()

    def close(self):
        if getattr(self, 'h', None):
            self._lib.lf_destroy(self.h)
            self.h = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def default_gains(self):
        g = Gains()
        self._lib.lf_default_gains(C.byref(g))
        return g

    def set_gains(self, g):
        self._lib.lf_set_gains(self.h, C.byref(g))

    def get_gains(self):
        g = Gains()
        self._lib.lf_get_gains(self.h, C.byref(g))
        return g

    def reset(self): self._lib.lf_reset(self.h)
    def set_external(self, steer, scale, enable=True): self._lib.lf_set_external(self.h, float(steer), float(scale), int(enable))
    def start(self): self._lib.lf_start(self.h)
    def stop(self): self._lib.lf_stop(self.h)
    def lap_boundary(self): self._lib.lf_lap_boundary(self.h)

    @property
    def gyro_bias(self): return self._lib.lf_get_gyro_bias(self.h)

    @gyro_bias.setter
    def gyro_bias(self, b): self._lib.lf_set_gyro_bias(self.h, float(b))

    def step(self, s, gyro_z, acc_x, acc_y, dt):
        """s: 8 valores 0..1000 (uint16). Retorna (pwm_esq, pwm_dir)."""
        s = np.ascontiguousarray(s, dtype=np.uint16)
        C.memmove(self._f.s, s.ctypes.data, 2 * N_SENS)
        self._f.gyro_z = gyro_z; self._f.acc_x = acc_x; self._f.acc_y = acc_y; self._f.dt = dt
        self._lib.lf_step(self.h, C.byref(self._f), C.byref(self._c))
        return self._c.left, self._c.right

    def telem(self):
        self._lib.lf_get_telem(self.h, C.byref(self._t))
        return self._t

    def get_map(self):
        n = self._lib.lf_get_map(self.h, self._buf, MAP_MAX)
        return [self._buf[i].as_dict() for i in range(n)]

    def get_rec(self):
        n = self._lib.lf_get_rec(self.h, self._buf, MAP_MAX)
        return [self._buf[i].as_dict() for i in range(n)]

    def set_map(self, entries):
        n = min(len(entries), MAP_MAX)
        arr = (MapEntry * max(n, 1))()
        for i, e in enumerate(entries[:n]):
            arr[i].type = int(e['type']); arr[i].angle = float(e['angle'])
            arr[i].duration = float(e.get('duration', 0)); arr[i].dist = float(e['dist'])
            arr[i].kappa_max = float(e.get('kappa_max', 0))
        self._lib.lf_set_map(self.h, arr, n)
