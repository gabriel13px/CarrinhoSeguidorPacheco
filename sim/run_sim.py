#!/usr/bin/env python3
"""Simula o seguidor na pista real (extraída da imagem) e gera vídeo + relatório.

Exemplos:
  python3 run_sim.py                                  # 3 voltas, imprime relatório
  python3 run_sim.py --video out/sim.mp4 --png out/sim.png
  python3 run_sim.py --gain v_base=90 --gain a_lat_max=7 --laps 4
  python3 run_sim.py --live                           # playback interativo (janela matplotlib)
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from lfsim import run, Track  # noqa: E402
from lfsim.core_bridge import MODE_NAMES, SEG_NAMES  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--laps', type=int, default=3, help='voltas (1ª = mapeamento)')
    ap.add_argument('--video', help='saída MP4')
    ap.add_argument('--png', help='resumo PNG')
    ap.add_argument('--json', help='despeja resumo + mapa em JSON')
    ap.add_argument('--fps', type=int, default=25)
    ap.add_argument('--speed', type=float, default=1.0, help='velocidade do vídeo (1 = tempo real)')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--t-max', type=float, default=120.0)
    ap.add_argument('--snapshots', help='tempos (s) separados por vírgula para salvar PNGs do vídeo')
    ap.add_argument('--snap-dir', default=os.path.join(HERE, 'out', 'frames'))
    ap.add_argument('--gain', action='append', default=[], help='override: nome=valor (repetível)')
    ap.add_argument('--events', action='store_true', help='lista as trocas de modo')
    ap.add_argument('--live', action='store_true', help='playback interativo em janela')
    ap.add_argument('--policy', help='npz de política neural treinada (ml/train.py --what nn): substitui o PID')
    ap.add_argument('--gains-npz', help='npz de ganhos otimizados (ml/train.py --what gains)')
    ap.add_argument('--preset-map', help='pickle de mapa pré-gravado (out/ml/map_seed0.pkl)')
    a = ap.parse_args()

    gains = {}
    for kv in a.gain:
        k, v = kv.split('=')
        gains[k] = float(v)
    track = Track()
    policy, preset = None, None
    if a.policy:
        import numpy as np
        from ml.policy import MLPPolicy
        policy = MLPPolicy(); policy.set_params(np.load(a.policy)['theta'])
    if a.gains_npz:
        import numpy as np
        from ml.policy import gains_from_vec
        gains.update(gains_from_vec(np.load(a.gains_npz)['theta']))
    if a.preset_map:
        import pickle
        preset = pickle.load(open(a.preset_map, 'rb'))
    t0 = time.time()
    res = run(track, laps=a.laps, gains=gains, seed=a.seed, t_max=a.t_max, policy=policy, preset_map=preset)
    print(res.report())
    print(f'(simulação em {time.time() - t0:.1f}s)')
    if a.events:
        for e in res.events:
            print(f"  t={e['t']:7.3f}s volta {e['lap']} -> {MODE_NAMES[e['mode']]:8s} seg={SEG_NAMES[e['seg']]}")
    if a.json:
        with open(a.json, 'w') as f:
            json.dump({'summary': res.summary, 'map': res.map, 'gains': res.gains, 'events': res.events}, f, indent=1, default=float)
    if a.png:
        from lfsim.render import plot_summary
        print('png ->', plot_summary(res, track, a.png))
    if a.video:
        from lfsim.render import render_video
        snaps = [float(x) for x in a.snapshots.split(',')] if a.snapshots else None
        t0 = time.time()
        render_video(res, track, a.video, fps=a.fps, speed=a.speed, snapshots=snaps, snap_dir=a.snap_dir,
                     progress=lambda i, n: print(f'  frame {i}/{n}', end='\r'))
        print(f'vídeo -> {a.video} ({time.time() - t0:.0f}s)')
    if a.live:
        from lfsim.live import play
        play(res, track)


if __name__ == '__main__':
    main()
