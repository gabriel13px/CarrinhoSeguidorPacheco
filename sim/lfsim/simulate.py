"""Loop de simulação: sensores -> núcleo C++ -> motores -> física, com log completo."""
import numpy as np

from .core_bridge import Core, GAIN_U8, MODE_FINISHED, MODE_LOST, MODE_NAMES, SEG_NAMES
from .robot import Robot, RobotParams
from .track import Track

SCALARS = ['t', 'x', 'y', 'th', 'v', 'w', 'slip', 'idx', 'dlat', 'left', 'right', 'mode', 'seg',
           'map_idx', 'map_len', 'lap', 'has_map', 'line_pos', 'heading', 'omega_f', 'v_est', 'k',
           'v_target', 'steer', 'bars', 'zz_target', 'map_seg', 'gyro', 'acc_y', 'zz_mid', 'zz_half', 'zz_t_far', 'zz_swaps', 'boost']


class Result:
    def __init__(self, log, sens, summary, map_entries, gains, events, params):
        self.log, self.sens, self.summary = log, sens, summary
        self.map, self.gains, self.events, self.params = map_entries, gains, events, params

    def report(self):
        s = self.summary
        lines = [f"voltas completas (verdade de terreno): {len(s['lap_times'])}  tempos: "
                 + ', '.join(f'{x:.2f}s' for x in s['lap_times']),
                 f"terminou: {s['finished']}  falha: {s['fail']}  tempo total: {s['t_total']:.2f}s",
                 f"mapa fechado: {s['has_map']} ({s['map_len']} entradas)  k pwm->v final: {s['k_final']:.5f} (real ~{s['k_true']:.5f})",
                 f"max afastamento sensor-linha: {s['max_dlat'] * 100:.1f} cm   tempo em LOST: {s['t_lost']:.2f}s   "
                 f"fração derrapando: {s['slip_frac'] * 100:.1f}%",
                 f"v máx: {s['v_max']:.2f} m/s   v média por volta: " + ', '.join(f'{x:.2f}' for x in s['v_mean_laps'])]
        if self.map:
            lines.append('mapa: ' + ' '.join(f"{SEG_NAMES[m['type']][:4]}" + (f"({np.degrees(m['angle']):+.0f}°)" if m['type'] in (1, 2) else '')
                                           for m in self.map))
        return '\n'.join(lines)


def run(track=None, laps=3, gains=None, params=None, t_max=120.0, dt=0.001, seed=0, start_back=0.25,
        preset_map=None, log_every=1, verbose=False, policy=None):
    """policy: callable(sensores, gyro, acc_y, telemetria_anterior, dt) -> (steer, escala_de_velocidade).
    Substitui o PID de direção no modo FOLLOW (resto do núcleo continua igual)."""
    track = track or Track()
    rng = np.random.default_rng(seed)
    params = params or RobotParams()
    core = Core()
    g = core.default_gains()
    for k, v in (gains or {}).items():
        if not hasattr(g, k):
            raise KeyError(f'ganho desconhecido: {k}')
        setattr(g, k, int(round(v)) if k in GAIN_U8 else v)
    g.laps = laps
    core.set_gains(g)
    if preset_map:
        core.set_map(preset_map)
    robot = Robot(params, rng)
    i0 = track.index_at_s(-start_back)
    robot.set_pose(track.path[i0, 0], track.path[i0, 1], track.heading_at(i0))

    def read_sensors():
        cov = track.sample(robot.sensor_positions())
        return np.clip(1000.0 * cov + rng.normal(0, params.sens_noise, 8), 0, 1000).astype(np.uint16)

    # 1 s parado antes da largada: o núcleo estima o bias do giroscópio
    for _ in range(int(round(1.0 / dt))):
        gyro, ax, ay = robot.imu(dt)
        core.step(read_sensors(), gyro, ax, ay, dt)
    core.start()

    cols = {k: [] for k in SCALARS}
    sens_log = []
    events = []
    lap_times, lap_t0 = [], 0.0
    t, off_t, lost_t, fail, prev_mode, last_idx = 0.0, 0.0, 0.0, None, -1, i0
    started_lap = False
    n_steps = int(t_max / dt)
    tm = core.telem()
    progress_m, last_prog_idx = 0.0, i0
    for n in range(n_steps):
        s = read_sensors()
        gyro, ax, ay = robot.imu(dt)
        if policy is not None:
            st, sc = policy(s, gyro, ay, tm, dt)
            core.set_external(st, sc, True)
        left, right = core.step(s, gyro, ax, ay, dt)
        robot.step(left, right, dt)
        tm = core.telem()
        sc = robot.sensor_center()
        idx, dlat = track.nearest(sc[0], sc[1])
        if last_idx > 0.8 * track.n and idx < 0.2 * track.n:   # passou pela largada (verdade de terreno)
            if started_lap:
                lap_times.append(t - lap_t0)
            started_lap, lap_t0 = True, t
        last_idx = idx
        dprog = (idx - last_prog_idx + track.n // 2) % track.n - track.n // 2   # avanço com wrap (pode ser negativo)
        if abs(dprog) * track.ds < 0.02:          # salto maior que isso é troca de trecho mais próximo, não movimento
            progress_m += dprog * track.ds
        last_prog_idx = idx
        if tm.mode != prev_mode:
            events.append({'t': t, 'mode': int(tm.mode), 'seg': int(tm.seg), 'idx': idx, 'lap': int(tm.lap)})
            prev_mode = tm.mode
        off_t = off_t + dt if dlat > 0.08 else 0.0
        lost_t = lost_t + dt if tm.mode == MODE_LOST else 0.0
        if n % log_every == 0:
            vals = (t, robot.x, robot.y, robot.th, robot.v, robot.w, robot.slip, idx, dlat, left, right,
                    tm.mode, tm.seg, tm.map_idx, tm.map_len, tm.lap, tm.has_map, tm.line_pos, tm.heading,
                    tm.omega_f, tm.v_est, tm.k_pwm2v, tm.v_target, tm.steer, tm.bars, tm.zz_target, tm.map_seg,
                    gyro, ay, tm.zz_mid, tm.zz_half, tm.zz_t_far, tm.zz_swaps, tm.boost)
            for k, v in zip(SCALARS, vals):
                cols[k].append(v)
            sens_log.append(s)
        t += dt
        if tm.mode == MODE_FINISHED:
            if started_lap and idx > 0.9 * track.n:   # parou na barra de chegada, logo antes do ponto de referência
                lap_times.append(t - lap_t0)
            break
        if off_t > 2.0 or lost_t > 3.0:
            fail = 'saiu da pista' if off_t > 2.0 else 'perdido (LOST > 3 s)'
            break
    if fail is None and tm.mode != MODE_FINISHED:
        fail = 'tempo esgotado'

    log = {k: np.asarray(v) for k, v in cols.items()}
    sens = np.asarray(sens_log)
    lap_arr = log['lap']
    v_mean_laps = [float(log['v'][lap_arr == L].mean()) for L in np.unique(lap_arr) if (lap_arr == L).sum() > 100]
    lost = log['mode'] == MODE_LOST
    summary = {
        'lap_times': lap_times, 'finished': bool(tm.mode == MODE_FINISHED), 'fail': fail, 't_total': t,
        'has_map': bool(tm.has_map), 'map_len': int(tm.map_len), 'k_final': float(tm.k_pwm2v),
        'k_true': robot.kv(), 'max_dlat': float(log['dlat'].max()), 't_lost': float(lost.sum() * dt * log_every),
        'slip_frac': float(log['slip'].mean()), 'v_max': float(log['v'].max()), 'v_mean_laps': v_mean_laps,
        'core_laps': int(tm.lap), 'bars': int(tm.bars), 'progress_m': float(progress_m),
    }
    res = Result(log, sens, summary, core.get_map(), g.as_dict(), events, params)
    if verbose:
        print(res.report())
    core.close()
    return res
