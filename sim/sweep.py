#!/usr/bin/env python3
"""Varredura de ganhos no simulador (otimização offline, versão grade).
Uso: python3 sweep.py --laps 3 --grid kd=1,2,4 --grid kgyro=6,10,15 [--seeds 0,1]
Imprime tabela ordenada por tempo da melhor volta com mapa; falhas vão pro fim."""
import argparse, itertools, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lfsim import run, Track

ap = argparse.ArgumentParser()
ap.add_argument('--laps', type=int, default=3)
ap.add_argument('--grid', action='append', default=[], help='nome=v1,v2,... (repetível)')
ap.add_argument('--seeds', default='0')
ap.add_argument('--base', action='append', default=[], help='ganho fixo nome=valor')
a = ap.parse_args()
names, values = [], []
for g in a.grid:
    k, vs = g.split('='); names.append(k); values.append([float(x) for x in vs.split(',')])
base = {k: float(v) for k, v in (b.split('=') for b in a.base)}
seeds = [int(s) for s in a.seeds.split(',')]
track = Track()
rows = []
t0 = time.time()
for combo in itertools.product(*values):
    gains = dict(base); gains.update(dict(zip(names, combo)))
    laps_ok, best, lost, worst_d = 0, 1e9, 0.0, 0.0
    for sd in seeds:
        r = run(track, laps=a.laps, gains=gains, seed=sd)
        s = r.summary
        lt = s['lap_times']
        laps_ok += int(s['finished'])
        if len(lt) >= 2: best = min(best, min(lt[1:]))
        lost += s['t_lost']; worst_d = max(worst_d, s['max_dlat'])
    rows.append((laps_ok, best, lost, worst_d, combo))
    print(f"{dict(zip(names, combo))}  ok={laps_ok}/{len(seeds)} melhor_volta={best:6.2f}s lost={lost:5.2f}s maxd={worst_d*100:4.1f}cm", flush=True)
rows.sort(key=lambda r: (-r[0], r[1]))
print(f'\n--- ranking ({time.time() - t0:.0f}s) ---')
for ok, best, lost, wd, combo in rows[:10]:
    print(f"ok={ok} melhor={best:6.2f}s lost={lost:5.2f}s maxd={wd*100:4.1f}cm  {dict(zip(names, combo))}")
