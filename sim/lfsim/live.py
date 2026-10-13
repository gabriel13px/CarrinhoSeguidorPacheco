"""Playback interativo do log (janela matplotlib). Não é usado pelos testes."""
import matplotlib
import numpy as np

from .render import MODE_COLORS, MODE_PT, _footprint, _sensor_pts
from .core_bridge import SEG_NAMES


def play(res, track, speed=1.0, fps=30):
    for backend in ('QtAgg', 'TkAgg', 'GTK3Agg'):
        try:
            matplotlib.use(backend)
            break
        except Exception:
            continue
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Polygon
    log, sens = res.log, res.sens
    t = log['t']
    step = max(1, int(round(speed / (fps * (t[1] - t[0])))))
    fig, ax = plt.subplots(figsize=(11, 7.5))
    ax.imshow(track.raster, cmap='gray_r', extent=track.extent, alpha=0.9, interpolation='nearest')
    ax.set_aspect('equal')
    body = Polygon(_footprint(0, 0, 0), closed=True, fc='#ffd54f', ec='k', zorder=6); ax.add_patch(body)
    sc = ax.scatter(np.zeros(8), np.zeros(8), s=30, c=np.zeros(8), cmap='Greys', vmin=0, vmax=1000, edgecolors='red', zorder=8)
    trail, = ax.plot([], [], lw=1.5, color='#1f77b4', alpha=0.6)
    txt = ax.text(0.01, 0.99, '', transform=ax.transAxes, va='top', family='monospace', fontsize=9,
                  bbox=dict(boxstyle='round', fc='white', alpha=0.9))

    def upd(i):
        x, y, th = log['x'][i], log['y'][i], log['th'][i]
        body.set_xy(_footprint(x, y, th))
        sc.set_offsets(_sensor_pts(x, y, th)); sc.set_array(sens[i].astype(float))
        trail.set_data(log['x'][:i:10], log['y'][:i:10])
        m = int(log['mode'][i])
        txt.set_text(f"t {t[i]:.2f}s volta {int(log['lap'][i])} {'mapa' if log['has_map'][i] else 'mapeando'}\n"
                     f"{MODE_PT[m]}\nseg {SEG_NAMES[int(log['seg'][i])]}  v {log['v'][i]:.2f} m/s")
        txt.set_color(MODE_COLORS[m])
        return body, sc, trail, txt

    FuncAnimation(fig, upd, frames=range(0, len(t), step), interval=1000 / fps, blit=False, repeat=False)
    plt.show()
