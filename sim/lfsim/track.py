"""Pista: caminho ordenado (verdade de terreno) + raster da linha (o que os sensores enxergam)."""
import os

import numpy as np
from scipy import ndimage as ndi
from scipy.spatial import cKDTree

DEFAULT_NPZ = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'data', 'track.npz')


class Track:
    def __init__(self, npz=DEFAULT_NPZ, spot_sigma_mm=1.5):
        if not os.path.exists(npz):
            raise FileNotFoundError(f'{npz} não existe: rode sim/track_from_image.py primeiro')
        d = np.load(npz)
        self.path = d['path_m']
        self.raster = d['raster']
        self.res = float(d['res'])
        self.lap_len = float(d['lap_length_m'])
        self.line_w = float(d['line_width_m'])
        self.H, self.W = self.raster.shape
        # borrado leve = mancha de leitura do fototransistor (cobertura parcial nas bordas da linha)
        self.img = ndi.gaussian_filter(self.raster.astype(np.float32), spot_sigma_mm)
        self.n = len(self.path)
        self.ds = self.lap_len / self.n
        self.tree = cKDTree(self.path)
        seg = np.roll(self.path, -1, axis=0) - self.path
        self.tangent = np.arctan2(seg[:, 1], seg[:, 0])

    @property
    def extent(self):
        return [0.0, self.W * self.res, 0.0, self.H * self.res]

    def sample(self, pts):
        """Cobertura 0..1 da linha em cada ponto (k,2) em metros."""
        col = pts[:, 0] / self.res
        row = (self.H - 1) - pts[:, 1] / self.res
        return ndi.map_coordinates(self.img, [row, col], order=1, mode='constant', cval=0.0)

    def index_at_s(self, s):
        return int(round(s / self.ds)) % self.n

    def heading_at(self, i):
        return float(self.tangent[i % self.n])

    def nearest(self, x, y):
        d, i = self.tree.query((x, y))
        return int(i), float(d)
