#!/usr/bin/env python3
"""Compara PID padrão, PID com ganhos otimizados por ES e rede neural (ES), em física nominal e perturbada.
Cada cenário grava seu próprio mapa com o controlador padrão (como aconteceria no dia), depois 2 voltas por semente.
Uso: python3 ml/compare.py [--seeds 3] [--video]"""
import argparse, json, os, sys
from dataclasses import replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
sys.path.insert(0, SIM)
from lfsim import run, Track, RobotParams           # noqa: E402
from ml.policy import MLPPolicy, gains_from_vec     # noqa: E402

OUT = os.path.join(SIM, 'out', 'ml')


def load_controllers():
    ctl = {'PID padrão': {'gains': {}, 'policy': None}}
    p = os.path.join(OUT, 'gains_best.npz')
    if os.path.exists(p):
        ctl['PID + ganhos por ES'] = {'gains': gains_from_vec(np.load(p)['theta']), 'policy': None}
    for tag, label in (('nn', 'Rede neural (ES, do zero)'), ('nn_pid', 'Rede neural (ES, partindo do PID)')):
        p = os.path.join(OUT, f'{tag}_best.npz')
        if os.path.exists(p):
            pol = MLPPolicy(); pol.set_params(np.load(p)['theta'])
            ctl[label] = {'gains': {}, 'policy': pol}
    return ctl


SCENARIOS = {
    'nominal': {},
    'bateria fraca (−15 % vel.)': {'v_max_motor': 2.2 * 0.85},
    'motor lento (τ 0,10 s)': {'tau_motor': 0.10},
    'piso liso (μg 7)': {'mu_g': 7.0},
    'sensor 2 cm mais perto': {'sensor_fwd': 0.07},
    'ruído 2× nos sensores': {'sens_noise': 30.0},
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seeds', type=int, default=3)
    ap.add_argument('--laps', type=int, default=2)
    a = ap.parse_args()
    track = Track()
    ctl = load_controllers()
    results = {}
    for sc_name, over in SCENARIOS.items():
        params = replace(RobotParams(), **over)
        m = run(track, laps=1, seed=0, params=params)
        if not m.summary['has_map']:
            print(f'[{sc_name}] mapeamento falhou'); continue
        for c_name, c in ctl.items():
            laps, fins, dls, lost = [], [], [], []
            for sd in range(a.seeds):
                if c['policy'] is not None: c['policy'].reset()
                r = run(track, laps=a.laps, preset_map=m.map, gains=c['gains'], policy=c['policy'],
                        params=params, seed=100 + sd, log_every=20, t_max=60.0)
                s = r.summary
                laps += s['lap_times']; fins.append(s['finished']); dls.append(s['max_dlat']); lost.append(s['t_lost'])
            results[(sc_name, c_name)] = {'lap_mean': float(np.mean(laps)) if laps else None, 'lap_best': float(min(laps)) if laps else None,
                                          'finish_rate': float(np.mean(fins)), 'max_dlat': float(max(dls)), 't_lost': float(np.mean(lost))}
            rr = results[(sc_name, c_name)]
            print(f"{sc_name:28s} | {c_name:32s} | volta média {rr['lap_mean'] if rr['lap_mean'] is None else round(rr['lap_mean'], 2)!s:>6} s | "
                  f"completou {rr['finish_rate'] * 100:4.0f} % | afast. máx {rr['max_dlat'] * 100:4.1f} cm | perdido {rr['t_lost']:.2f} s", flush=True)
    json.dump({f'{k[0]} | {k[1]}': v for k, v in results.items()}, open(os.path.join(OUT, 'compare.json'), 'w'), indent=1)
    # tabela markdown
    names = list(ctl)
    lines = ['| cenário | ' + ' | '.join(names) + ' |', '|---|' + '---|' * len(names)]
    for sc in SCENARIOS:
        row = []
        for n in names:
            r = results.get((sc, n))
            row.append('—' if r is None else (f"{r['lap_mean']:.2f} s" if r['lap_mean'] else 'falhou') + f" ({r['finish_rate'] * 100:.0f} %)")
        lines.append(f'| {sc} | ' + ' | '.join(row) + ' |')
    md = '\n'.join(lines)
    open(os.path.join(OUT, 'compare.md'), 'w').write(md + '\n')
    print('\n' + md)


if __name__ == '__main__':
    main()
