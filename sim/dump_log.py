#!/usr/bin/env python3
"""Imprime uma janela do log da simulação em tabela (diagnóstico).
Uso: python3 dump_log.py --t0 18.3 --t1 19.2 [--every 0.02] [--laps 3] [--gain k=v ...]"""
import argparse, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lfsim import run, Track
from lfsim.core_bridge import MODE_NAMES, SEG_NAMES

ap = argparse.ArgumentParser()
ap.add_argument('--t0', type=float, required=True); ap.add_argument('--t1', type=float, required=True)
ap.add_argument('--every', type=float, default=0.02); ap.add_argument('--laps', type=int, default=3)
ap.add_argument('--seed', type=int, default=0); ap.add_argument('--gain', action='append', default=[])
a = ap.parse_args()
gains = {k: float(v) for k, v in (g.split('=') for g in a.gain)}
res = run(Track(), laps=a.laps, gains=gains, seed=a.seed, t_max=a.t1 + 0.5)
L = res.log
print(f"{'t':>7} {'x':>5} {'y':>5} {'v':>5} {'vtgt':>5} {'e':>6} {'w':>6} {'mode':8} {'seg':7} {'idx':>3} {'mapseg':7} {'L':>4} {'R':>4} {'dlat':>5} {'nb':>2} {'zmid':>5} {'zhalf':>5} {'zfar':>5} {'sw':>2} sens")
step = max(1, int(round(a.every / (L['t'][1] - L['t'][0]))))
for i in range(0, len(L['t']), step):
    t = L['t'][i]
    if t < a.t0 or t > a.t1: continue
    s = ''.join('#' if v > 500 else ('+' if v > 200 else '.') for v in res.sens[i])
    print(f"{t:7.3f} {L['x'][i]:5.2f} {L['y'][i]:5.2f} {L['v'][i]:5.2f} {L['v_target'][i]*L['k'][i]:5.2f} {L['line_pos'][i]:+6.2f} {L['omega_f'][i]:+6.1f} "
          f"{MODE_NAMES[int(L['mode'][i])]:8} {SEG_NAMES[int(L['seg'][i])]:7} {int(L['map_idx'][i]):3d} {SEG_NAMES.get(int(L['map_seg'][i]),'-'):7} "
          f"{int(L['left'][i]):4d} {int(L['right'][i]):4d} {L['dlat'][i]*100:5.1f} {int((res.sens[i]>500).sum()):2d} "
          f"{L['zz_mid'][i]:+5.2f} {L['zz_half'][i]:5.3f} {L['zz_t_far'][i]:5.3f} {int(L['zz_swaps'][i]):2d} {s}")
