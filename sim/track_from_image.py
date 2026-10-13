#!/usr/bin/env python3
"""
Extrai a pista a partir da imagem publicada (pistaGlobal.jpeg).

Saida: sim/data/track.npz com
  path_m      : caminho ordenado e fechado (N x 2, metros, y para cima)
  raster      : mascara da linha preta a 1 mm/px (inclui as barras de largada/chegada)
  res         : resolucao do raster (m/px)
  mm_per_px   : escala da imagem original (assume linha de 19 mm de largura)
  line_width_m: largura da linha

Uso: python3 track_from_image.py [caminho_da_imagem]
"""
import sys, os
import numpy as np
from PIL import Image
from scipy import ndimage as ndi

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_IMG = os.path.join(HERE, '..', '..', 'pistaGlobal.jpeg')
OUT = os.path.join(HERE, 'data', 'track.npz')
LINE_WIDTH_MM = 19.0          # fita padrao de competicao (3/4")
RES_M = 0.001                 # raster a 1 mm/px


def segment_track(img_gray):
    """Isola a linha preta dentro do painel claro do poster."""
    light = img_gray > 170
    lab, n = ndi.label(light)
    sizes = ndi.sum(light, lab, range(1, n + 1))
    panel = lab == (np.argmax(sizes) + 1)
    panel = ndi.binary_erosion(ndi.binary_fill_holes(panel), iterations=12)
    dark = (img_gray < 110) & panel
    lab2, n2 = ndi.label(dark)
    sizes2 = ndi.sum(dark, lab2, range(1, n2 + 1))
    return lab2 == (np.argmax(sizes2) + 1)


def zhang_suen(img):
    img = img.astype(np.uint8).copy()
    changed = True
    while changed:
        changed = False
        for step in (0, 1):
            P = np.pad(img, 1)
            p2 = P[:-2, 1:-1]; p3 = P[:-2, 2:]; p4 = P[1:-1, 2:]; p5 = P[2:, 2:]
            p6 = P[2:, 1:-1]; p7 = P[2:, :-2]; p8 = P[1:-1, :-2]; p9 = P[:-2, :-2]
            B = (p2.astype(int) + p3 + p4 + p5 + p6 + p7 + p8 + p9)
            seq = [p2, p3, p4, p5, p6, p7, p8, p9, p2]
            A = sum(((seq[i] == 0) & (seq[i + 1] == 1)).astype(int) for i in range(8))
            if step == 0:
                c1 = (p2 * p4 * p6 == 0); c2 = (p4 * p6 * p8 == 0)
            else:
                c1 = (p2 * p4 * p8 == 0); c2 = (p2 * p6 * p8 == 0)
            m = (img == 1) & (B >= 2) & (B <= 6) & (A == 1) & c1 & c2
            if m.any():
                img[m] = 0
                changed = True
    return img.astype(bool)


def neighbors_count(sk):
    P = np.pad(sk.astype(np.uint8), 1)
    return (P[:-2, :-2] + P[:-2, 1:-1] + P[:-2, 2:] + P[1:-1, :-2] + P[1:-1, 2:]
            + P[2:, :-2] + P[2:, 1:-1] + P[2:, 2:])


def prune(sk, iters):
    sk = sk.copy()
    for _ in range(iters):
        ends = sk & (neighbors_count(sk) <= 1)
        if not ends.any():
            break
        sk[ends] = False
    return sk


OFFS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def walk(sk, start, heading):
    """Percorre o esqueleto escolhendo sempre a continuacao mais reta.
    Isso resolve os cruzamentos (segue reto) sem precisar de grafo."""
    visits = np.zeros(sk.shape, np.uint8)
    pts = [start]
    cur = start
    visits[cur] = 1
    h = np.array(heading, float)
    for _ in range(200000):
        best, best_s = None, -9
        recent = set(pts[-4:])
        for dr, dc in OFFS:
            n = (cur[0] + dr, cur[1] + dc)
            if not (0 <= n[0] < sk.shape[0] and 0 <= n[1] < sk.shape[1]):
                continue
            if not sk[n] or n in recent or visits[n] >= 2:
                continue
            d = np.array([dr, dc], float); d /= np.linalg.norm(d)
            s = float(d @ h)
            if s > best_s:
                best_s, best = s, n
        if best is None:
            break
        pts.append(best)
        visits[best] += 1
        cur = best
        k = min(len(pts) - 1, 6)
        v = np.array(pts[-1], float) - np.array(pts[-1 - k], float)
        if np.linalg.norm(v) > 0:
            h = v / np.linalg.norm(v)
        if len(pts) > 500 and abs(cur[0] - start[0]) + abs(cur[1] - start[1]) <= 2:
            break
    return pts


def resample_closed(xy, spacing):
    xy = np.asarray(xy, float)
    d = np.linalg.norm(np.diff(np.vstack([xy, xy[:1]]), axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(d)])
    total = s[-1]
    n = int(round(total / spacing))
    t = np.linspace(0, total, n, endpoint=False)
    closed = np.vstack([xy, xy[:1]])
    x = np.interp(t, s, closed[:, 0]); y = np.interp(t, s, closed[:, 1])
    return np.column_stack([x, y]), total


def main(img_path=DEFAULT_IMG):
    g = np.array(Image.open(img_path).convert('L'))
    track = segment_track(g)
    ys, xs = np.where(track)
    m = 60
    r0, r1 = max(ys.min() - m, 0), min(ys.max() + m, track.shape[0])
    c0, c1 = max(xs.min() - m, 0), min(xs.max() + m, track.shape[1])
    track = track[r0:r1, c0:c1]

    dt = ndi.distance_transform_edt(track)
    sk = zhang_suen(track)
    width_px = 2.0 * np.median(dt[sk])
    mm_per_px = LINE_WIDTH_MM / width_px
    sk = prune(sk, 45)                       # remove barras e espinhos: sobra so o laco

    # ponto de partida: trecho reto inferior, entre as duas barras
    rows, cols = np.where(sk)
    target = np.array([ys.max() - r0 - 28, (xs.min() + xs.max()) / 2 - c0 - 30])
    i = np.argmin(np.hypot(rows - target[0], cols - target[1]))
    start = (int(rows[i]), int(cols[i]))
    pts = walk(sk, start, heading=(0.0, 1.0))        # sentido: para a direita na imagem
    closed = abs(pts[-1][0] - start[0]) + abs(pts[-1][1] - start[1]) <= 2
    coverage = len(set(pts)) / sk.sum()
    print(f'largura da linha: {width_px:.1f}px -> {mm_per_px:.3f} mm/px; '
          f'esqueleto {sk.sum()} px; caminho {len(pts)} px; fechado={closed}; cobertura={coverage:.2%}')
    if not closed:
        raise SystemExit('caminho nao fechou: ajuste start/heading')

    # suaviza (sigma ~ 2.5 px ~ 6 mm) com wrap, converte p/ metros (y para cima)
    P = np.array(pts, float)
    H = track.shape[0]
    xm = ndi.gaussian_filter1d(P[:, 1], 2.5, mode='wrap') * mm_per_px / 1000.0
    ym = ndi.gaussian_filter1d(H - 1 - P[:, 0], 2.5, mode='wrap') * mm_per_px / 1000.0
    path_m, total = resample_closed(np.column_stack([xm, ym]), 0.002)

    # raster a 1 mm/px (mantem barras e cruzamentos exatamente como na imagem)
    zoom = mm_per_px / (RES_M * 1000.0)
    raster = ndi.zoom(track.astype(np.float32), zoom, order=1) > 0.5
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    np.savez_compressed(OUT, path_m=path_m, raster=raster, res=RES_M, mm_per_px=mm_per_px,
                        line_width_m=LINE_WIDTH_MM / 1000.0, lap_length_m=total)
    print(f'pista: {total:.2f} m de volta, raster {raster.shape[1]}x{raster.shape[0]} px @1mm, salvo em {OUT}')
    return path_m, raster


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_IMG)
