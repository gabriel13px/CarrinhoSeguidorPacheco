#!/usr/bin/env python3
"""Treina no simulador, com o mesmo orçamento, duas versões "ML":
  --what gains : ES otimiza os ganhos do controlador clássico (PID + perfil de velocidade)
  --what nn    : ES treina uma MLP que substitui o PID de direção e escala a velocidade
Avaliação = 2 voltas com o mapa pré-gravado (volta de mapeamento clássica), física nominal, ruído por semente.
Pontuação = 2·progresso(m) − tempo(s) − 5·tempo_perdido − 100·max(0, afastamento−6 cm) + 10 se completou.
Uso: python3 ml/train.py --what nn --gens 60 --pop 32 --workers 14 [--init pid]
"""
import argparse, json, os, pickle, sys, time
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
sys.path.insert(0, SIM)
from lfsim import run, Track                       # noqa: E402
from ml.policy import MLPPolicy, gains_from_vec, vec_from_gains, GAIN_NAMES   # noqa: E402
from ml.es import OpenAIES                         # noqa: E402

OUT = os.path.join(SIM, 'out', 'ml')
_track = None
_map = None
_policy = None


def score_of(summary):
    s = summary
    return (2.0 * s['progress_m'] - s['t_total'] - 5.0 * s['t_lost'] - 100.0 * max(0.0, s['max_dlat'] - 0.06)
            + (10.0 if s['finished'] else 0.0))


def _init_worker(map_entries, what):
    global _track, _map, _policy
    _track = Track(); _map = map_entries
    _policy = MLPPolicy() if what == 'nn' else None


def evaluate(args):
    vec, seed, what = args
    if what == 'nn':
        _policy.set_params(vec); _policy.reset()
        r = run(_track, laps=2, preset_map=_map, policy=_policy, seed=seed, log_every=50, t_max=40.0)
    else:
        r = run(_track, laps=2, preset_map=_map, gains=gains_from_vec(vec), seed=seed, log_every=50, t_max=40.0)
    s = r.summary
    return score_of(s), s['lap_times'], s['finished'], s['max_dlat'], s['t_lost']


def build_map(track):
    """Volta de mapeamento com o controlador clássico padrão (igual para todas as versões)."""
    r = run(track, laps=1, seed=0)
    assert r.summary['has_map'], 'mapeamento falhou'
    return r.map


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--what', choices=['gains', 'nn'], required=True)
    ap.add_argument('--gens', type=int, default=40)
    ap.add_argument('--pop', type=int, default=32)
    ap.add_argument('--sigma', type=float, default=None)
    ap.add_argument('--lr', type=float, default=None)
    ap.add_argument('--workers', type=int, default=max(1, os.cpu_count() - 2))
    ap.add_argument('--init', choices=['zero', 'pid'], default='zero', help='nn: começa do zero ou do PID linear')
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    track = Track()
    map_path = os.path.join(OUT, 'map_seed0.pkl')
    if os.path.exists(map_path):
        map_entries = pickle.load(open(map_path, 'rb'))
    else:
        map_entries = build_map(track); pickle.dump(map_entries, open(map_path, 'wb'))

    if a.what == 'nn':
        pol = MLPPolicy.pid_init() if a.init == 'pid' else MLPPolicy()
        theta0 = pol.get_params()
        if a.init == 'zero':
            theta0 = np.random.default_rng(a.seed).normal(0, 0.1, theta0.size)
        sigma, lr = a.sigma or 0.1, a.lr or 0.03
    else:
        from lfsim.core_bridge import Core
        theta0 = vec_from_gains(Core().default_gains().as_dict())
        sigma, lr = a.sigma or 0.3, a.lr or 0.1
    es = OpenAIES(theta0, sigma=sigma, lr=lr, pop=a.pop, seed=a.seed)
    tag = f"{a.what}{'_pid' if (a.what == 'nn' and a.init == 'pid') else ''}"
    hist, best = [], (-1e9, theta0.copy(), None)
    t0 = time.time()
    with Pool(a.workers, initializer=_init_worker, initargs=(map_entries, a.what)) as pool:
        for g in range(a.gens):
            vecs, eps = es.ask()
            seed = 1000 + g                                    # mesmo ruído para toda a geração (comparação justa)
            res = pool.map(evaluate, [(v, seed, a.what) for v in vecs])
            scores = np.array([r[0] for r in res])
            es.tell(scores, eps)
            # avalia o centro (theta) em 2 sementes: é o candidato que vale guardar
            cen = pool.map(evaluate, [(es.theta.copy(), 5000 + 2 * g + k, a.what) for k in range(2)])
            cs = float(np.mean([c[0] for c in cen]))
            laps = [lt for c in cen for lt in c[1]]
            fin = all(c[2] for c in cen)
            if cs > best[0]:
                best = (cs, es.theta.copy(), {'laps': laps, 'finished': fin, 'max_dlat': max(c[3] for c in cen)})
            hist.append({'gen': g, 'pop_mean': float(scores.mean()), 'pop_max': float(scores.max()), 'center': cs,
                         'center_laps': laps, 'center_finished': fin, 't': time.time() - t0})
            print(f"[{tag}] gen {g:3d}  pop média {scores.mean():6.1f} máx {scores.max():6.1f}  centro {cs:6.1f}  "
                  f"voltas {['%.2f' % x for x in laps]} ok={fin}  ({time.time() - t0:.0f}s)", flush=True)
            np.savez(os.path.join(OUT, f'{tag}_best.npz'), theta=best[1], score=best[0], what=a.what,
                     init=a.init, names=np.array(GAIN_NAMES) if a.what == 'gains' else np.array([]))
            json.dump(hist, open(os.path.join(OUT, f'{tag}_history.json'), 'w'), indent=1)
    print(f'[{tag}] melhor centro: score {best[0]:.1f} {best[2]}')
    if a.what == 'gains':
        print('ganhos:', {k: round(v, 3) for k, v in gains_from_vec(best[1]).items()})


if __name__ == '__main__':
    main()
