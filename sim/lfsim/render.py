"""Renderização: vídeo MP4 (ffmpeg) e resumo PNG a partir do log da simulação."""
import os
import subprocess

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402

from .core_bridge import MODE_NAMES, SEG_NAMES  # noqa: E402

MODE_COLORS = {0: '#9e9e9e', 1: '#1f77b4', 2: '#d62728', 3: '#ff7f0e', 4: '#000000', 5: '#2ca02c'}
MODE_PT = {0: 'PARADO', 1: 'SEGUINDO LINHA (PID)', 2: 'ZIGUE-ZAGUE: rumo pelo giroscópio',
           3: 'CRUZAMENTO: rumo pelo giroscópio', 4: 'PERDEU A LINHA', 5: 'CHEGOU'}
LAP_COLORS = ['#9e9e9e', '#1f77b4', '#d62728', '#2ca02c', '#9467bd']


def _footprint(x, y, th, L=0.16, W=0.14):
    body = np.array([[-0.05, -W / 2], [L - 0.05, -W / 2], [L - 0.05, W / 2], [-0.05, W / 2]])
    c, s = np.cos(th), np.sin(th)
    return body @ np.array([[c, -s], [s, c]]).T + np.array([x, y])


def _sensor_pts(x, y, th, fwd=0.09, pitch=0.009525):
    cx, cy = x + fwd * np.cos(th), y + fwd * np.sin(th)
    nl = np.array([-np.sin(th), np.cos(th)])
    offs = (3.5 - np.arange(8)) * pitch
    return np.column_stack([cx + offs * nl[0], cy + offs * nl[1]])


def plot_summary(res, track, out_png):
    log = res.log
    fig = plt.figure(figsize=(14, 13))
    gs = fig.add_gridspec(5, 1, height_ratios=[4.2, 1, 1, 1, 1], hspace=0.35)
    ax = fig.add_subplot(gs[0])
    ax.imshow(track.raster, cmap='gray_r', extent=track.extent, alpha=0.35, interpolation='nearest')
    laps = np.unique(log['lap'])
    for L in laps:
        m = log['lap'] == L
        ax.plot(log['x'][m], log['y'][m], color=LAP_COLORS[int(L) % len(LAP_COLORS)], lw=1.2,
                label=f'volta {int(L)}' + (' (mapeando)' if L == 1 else ' (com mapa)'))
    for mode, mk, lab in ((2, 's', 'zigue-zague (heading hold)'), (3, 'o', 'cruzamento (heading hold)'), (4, 'x', 'perdeu linha')):
        m = log['mode'] == mode
        if m.any():
            ax.scatter(log['x'][m][::20], log['y'][m][::20], s=10, marker=mk, color=MODE_COLORS[mode], label=lab, zorder=5)
    ax.plot(log['x'][0], log['y'][0], 'k^', ms=9, label='largada')
    s = res.summary
    ax.set_title('Trajetória — voltas: ' + ', '.join(f'{x:.2f}s' for x in s['lap_times'])
                 + f"   |   terminou={s['finished']} falha={s['fail']}   |   mapa={s['map_len']} entradas")
    ax.legend(loc='upper right', fontsize=8)
    ax.set_aspect('equal')
    t = log['t']
    ax1 = fig.add_subplot(gs[1]); ax1.plot(t, log['v'], 'k', lw=0.8, label='v real')
    ax1.plot(t, log['v_est'], 'r--', lw=0.8, label='v estimada (pwm·k)')
    ax1.plot(t, log['v_target'] * log['k'], 'g:', lw=0.8, label='v alvo'); ax1.set_ylabel('m/s'); ax1.legend(fontsize=7, loc='upper right')
    ax2 = fig.add_subplot(gs[2], sharex=ax1); ax2.plot(t, log['line_pos'], lw=0.6); ax2.set_ylabel('pos. linha'); ax2.set_ylim(-1.1, 1.1)
    ax3 = fig.add_subplot(gs[3], sharex=ax1); ax3.plot(t, log['omega_f'], lw=0.6); ax3.set_ylabel('ω (rad/s)')
    ax4 = fig.add_subplot(gs[4], sharex=ax1)
    for mode in range(6):
        m = log['mode'] == mode
        if m.any():
            ax4.scatter(t[m], np.full(m.sum(), mode), s=2, color=MODE_COLORS[mode])
    ax4.set_yticks(range(6)); ax4.set_yticklabels([MODE_NAMES[i] for i in range(6)], fontsize=7); ax4.set_xlabel('t (s)')
    for L in laps[1:]:
        t0 = t[log['lap'] == L][0]
        for a in (ax1, ax2, ax3, ax4):
            a.axvline(t0, color='gray', lw=0.6, ls='--')
    os.makedirs(os.path.dirname(os.path.abspath(out_png)), exist_ok=True)
    fig.savefig(out_png, dpi=110, bbox_inches='tight')
    plt.close(fig)
    return out_png


def render_video(res, track, out_mp4, fps=25, speed=1.0, size=(1280, 720), snapshots=None, snap_dir=None,
                 window=3.0, progress=None):
    """Gera MP4 do log. snapshots: lista de tempos (s) para salvar PNG do frame."""
    log, sens = res.log, res.sens
    t = log['t']
    dt = float(t[1] - t[0]) if len(t) > 1 else 0.001
    step = max(1, int(round(speed / (fps * dt))))
    frames = range(0, len(t), step)
    W, H = size
    dpi = 100
    fig = plt.figure(figsize=(W / dpi, H / dpi), dpi=dpi)
    gs = fig.add_gridspec(3, 2, width_ratios=[2.3, 1], left=0.02, right=0.98, top=0.87, bottom=0.07, wspace=0.12, hspace=0.45)
    axm = fig.add_subplot(gs[:, 0])
    axe = fig.add_subplot(gs[0, 1]); axw = fig.add_subplot(gs[1, 1]); axv = fig.add_subplot(gs[2, 1])
    axm.imshow(track.raster, cmap='gray_r', extent=track.extent, alpha=0.9, interpolation='nearest')
    axm.set_xlim(track.extent[0], track.extent[1]); axm.set_ylim(track.extent[2], track.extent[3])
    axm.set_aspect('equal'); axm.set_xticks([]); axm.set_yticks([])
    trail = LineCollection([], linewidths=2.0, alpha=0.8); axm.add_collection(trail)
    body = Polygon(_footprint(0, 0, 0), closed=True, fc='#ffd54f', ec='k', lw=1.2, alpha=0.9, zorder=6); axm.add_patch(body)
    nose, = axm.plot([], [], 'k-', lw=2, zorder=7)
    sens_sc = axm.scatter(np.zeros(8), np.zeros(8), s=28, c=np.zeros(8), cmap='Greys', vmin=0, vmax=1000,
                          edgecolors='red', linewidths=0.8, zorder=8)
    hud = fig.text(0.02, 0.985, '', va='top', ha='left', fontsize=9.5, family='monospace')   # faixa acima do mapa, não cobre a pista
    modetxt = axm.text(0.5, 0.01, '', transform=axm.transAxes, va='bottom', ha='center', fontsize=12, weight='bold',
                       bbox=dict(boxstyle='round', fc='white', ec='0.6', alpha=0.9), zorder=10)
    le, = axe.plot([], [], lw=1); axe.axhline(0, color='0.7', lw=0.6); axe.set_ylim(-1.1, 1.1); axe.set_ylabel('pos. linha', fontsize=8)
    lw_, = axw.plot([], [], lw=1, color='#d62728'); axw.axhline(0, color='0.7', lw=0.6); axw.set_ylabel('ω giro (rad/s)', fontsize=8)
    lv, = axv.plot([], [], 'k', lw=1, label='v real'); lve, = axv.plot([], [], 'r--', lw=1, label='v est.')
    lvt, = axv.plot([], [], 'g:', lw=1, label='v alvo'); axv.set_ylabel('m/s', fontsize=8); axv.set_xlabel('t (s)', fontsize=8)
    axv.legend(fontsize=7, loc='upper left')
    for a in (axe, axw, axv):
        a.tick_params(labelsize=7)
    # cor do rastro por modo
    xs, ys = log['x'], log['y']
    seg_pts = np.column_stack([xs, ys])
    cmd = ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}',
           '-r', str(fps), '-i', '-', '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '22',
           '-preset', 'veryfast', out_mp4]
    os.makedirs(os.path.dirname(os.path.abspath(out_mp4)), exist_ok=True)
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    snaps = sorted(snapshots or [])
    if snaps and snap_dir:
        os.makedirs(snap_dir, exist_ok=True)
    trail_step = max(1, int(0.01 / dt))
    segs, cols = [], []
    last_trail = 0
    try:
        for fi, i in enumerate(frames):
            # rastro incremental (um segmento a cada 10 ms, cor = modo)
            while last_trail + trail_step <= i:
                j = last_trail
                segs.append([seg_pts[j], seg_pts[j + trail_step]])
                cols.append(MODE_COLORS[int(log['mode'][j + trail_step])])
                last_trail += trail_step
            trail.set_segments(segs); trail.set_color(cols)
            x, y, th = xs[i], ys[i], log['th'][i]
            body.set_xy(_footprint(x, y, th))
            nose.set_data([x, x + 0.05 * np.cos(th)], [y, y + 0.05 * np.sin(th)])
            sp = _sensor_pts(x, y, th); sens_sc.set_offsets(sp); sens_sc.set_array(sens[i].astype(float))
            mode = int(log['mode'][i]); L = int(log['lap'][i]); hm = int(log['has_map'][i])
            hud.set_text(
                f"t {t[i]:6.2f}s   volta {L}   {'COM MAPA idx %d/%d' % (log['map_idx'][i], log['map_len'][i]) if hm else 'MAPEANDO (volta 1)'}\n"
                f"segmento ao vivo: {SEG_NAMES[int(log['seg'][i])]:8s} ang {np.degrees(log['heading'][i] - log['zz_target'][i]) if mode == 2 else 0:+5.0f}°"
                f"   mapa: {SEG_NAMES.get(int(log['map_seg'][i]), '-')}\n"
                f"v real {log['v'][i]:.2f} m/s  v est {log['v_est'][i]:.2f}  k {log['k'][i]:.4f}  pwm alvo {log['v_target'][i]:4.0f}  L/R {int(log['left'][i]):4d}/{int(log['right'][i]):4d}\n"
                f"pos linha {log['line_pos'][i]:+.2f}  ω {log['omega_f'][i]:+5.1f} rad/s  barras {int(log['bars'][i])}"
                + ('   DERRAPANDO' if log['slip'][i] else ''))
            modetxt.set_text(MODE_PT[mode]); modetxt.set_color(MODE_COLORS[mode])
            t0 = max(0.0, t[i] - window); m = (t >= t0) & (t <= t[i])
            le.set_data(t[m], log['line_pos'][m]); lw_.set_data(t[m], log['omega_f'][m])
            lv.set_data(t[m], log['v'][m]); lve.set_data(t[m], log['v_est'][m]); lvt.set_data(t[m], log['v_target'][m] * log['k'][m])
            for a in (axe, axw, axv):
                a.set_xlim(t0, max(t0 + window, t[i]))
            axw.set_ylim(-12, 12); axv.set_ylim(0, 2.5)
            fig.canvas.draw()
            buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8).reshape(H, W, 4)[:, :, :3]
            proc.stdin.write(buf.tobytes())
            if snaps and t[i] >= snaps[0]:
                if snap_dir:
                    fig.savefig(os.path.join(snap_dir, f'frame_{snaps[0]:06.2f}s.png'), dpi=dpi)
                snaps.pop(0)
            if progress and fi % 100 == 0:
                progress(fi, len(frames))
    finally:
        proc.stdin.close()
        proc.wait()
        plt.close(fig)
    return out_mp4
