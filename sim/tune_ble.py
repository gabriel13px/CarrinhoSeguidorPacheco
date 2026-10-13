#!/usr/bin/env python3
"""Ajuste fino dos ganhos no carrinho REAL por BLE (laço fechado, semi-automático).

Fluxo por tentativa: manda ganhos (G:nome=valor) -> você põe o carrinho na largada e liga (botão ou ON) ->
script coleta as linhas LAP:volta,tempo,perdas até N voltas (ou OF/timeout) -> nota = tempo médio + penalidade
por perda de linha -> estratégia evolutiva escolhe o próximo chute. Salva histórico em CSV.

Requisitos: pip install bleak. Carrinho anunciando como "SeguidorPacheco" (Nordic UART).
Uso: python3 tune_ble.py --gains kp,kd,kgyro,a_lat_max,brake_margin --laps 2 --iters 20
     python3 tune_ble.py --monitor            # só mostra telemetria R:/LAP: enquanto corre
"""
import argparse, asyncio, csv, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


NUS_RX = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'   # escrita (comandos)
NUS_TX = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'   # notificação (respostas)
BOUNDS = {'kp': (10, 150), 'kp_v': (0.2, 2.5), 'kd': (0, 5), 'kgyro': (0, 20), 'kv_e': (0, 0.8),
          'a_lat_max': (3, 9.5), 'a_brake': (2, 9), 'brake_margin': (0, 0.25), 'v_straight': (120, 255),
          'v_zigzag': (0.6, 2.0), 'v_base': (50, 140)}


class Car:
    def __init__(self, client):
        self.c, self.buf, self.lines = client, '', asyncio.Queue()

    def _rx(self, _, data):
        self.buf += data.decode(errors='ignore')
        while '\n' in self.buf:
            line, self.buf = self.buf.split('\n', 1)
            if line:
                self.lines.put_nowait(line)

    async def start(self):
        await self.c.start_notify(NUS_TX, self._rx)

    async def send(self, cmd):
        await self.c.write_gatt_char(NUS_RX, (cmd + '\n').encode(), response=False)
        await asyncio.sleep(0.05)

    async def set_gains(self, g):
        for k, v in g.items():
            await self.send(f'G:{k}={v:.4g}')

    async def collect_laps(self, n_laps, timeout):
        """Espera n linhas LAP:volta,tempo,perdas. Retorna (tempos, perdas)."""
        t0, laps, lost = time.time(), [], 0
        while len(laps) < n_laps and time.time() - t0 < timeout:
            try:
                line = await asyncio.wait_for(self.lines.get(), timeout=1.0)
            except asyncio.TimeoutError:
                continue
            if line.startswith('R:'):
                print('\r  ' + line, end='', flush=True)
            elif line.startswith('LAP:'):
                _, rest = line.split(':', 1)
                v, t, p = rest.split(',')
                if int(v) >= 2:                      # volta 1 é mapeamento; não pontua
                    laps.append(float(t)); lost += int(p)
                print(f'\n  volta {v}: {t} s, perdas {p}')
            elif line.startswith(('STATE:OFF', 'STATE:FINISHED')):
                break
            else:
                print('\n  ' + line)
        return laps, lost


def score(laps, lost, n_laps):
    if not laps:
        return 100.0
    return float(np.mean(laps)) + 3.0 * lost + 10.0 * (n_laps - len(laps))   # menor é melhor


async def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--gains', default='kp,kd,kgyro,a_lat_max,brake_margin')
    ap.add_argument('--laps', type=int, default=2, help='voltas pontuadas por tentativa (além da de mapeamento)')
    ap.add_argument('--iters', type=int, default=20)
    ap.add_argument('--sigma', type=float, default=0.15, help='passo relativo ao intervalo de cada ganho')
    ap.add_argument('--timeout', type=float, default=90)
    ap.add_argument('--name', default='SeguidorPacheco')
    ap.add_argument('--monitor', action='store_true')
    ap.add_argument('--csv', default='tune_ble_log.csv')
    ap.add_argument('--apply', help='npz do ES (out/ml/gains_best.npz): manda os ganhos ao carrinho, grava (GS) e sai')
    ap.add_argument('--print', dest='print_only', help='npz do ES: só imprime os comandos G:... para colar num terminal BLE')
    a = ap.parse_args()
    if a.print_only or a.apply:
        from ml.policy import gains_from_vec
        es_gains = gains_from_vec(np.load(a.print_only or a.apply)['theta'])
        if a.print_only:
            for k, v in es_gains.items():
                print(f'G:{k}={v:.4g}')
            print('GS')
            return

    try:
        from bleak import BleakClient, BleakScanner
    except ImportError:
        sys.exit('instale: pip install bleak')
    dev = await BleakScanner.find_device_by_name(a.name, timeout=10)
    if not dev:
        sys.exit(f'carrinho "{a.name}" não encontrado')
    async with BleakClient(dev) as client:
        car = Car(client); await car.start()
        if a.apply:
            await car.set_gains(es_gains); await car.send('GS')
            print('ganhos do ES aplicados e gravados:', {k: round(v, 3) for k, v in es_gains.items()})
            return
        await car.send('TL=1')
        if a.monitor:
            print('monitorando (Ctrl+C sai)')
            while True:
                print(await car.lines.get())

        names = a.gains.split(',')
        await car.send('GG'); await asyncio.sleep(1.0)
        cur = {}
        while not car.lines.empty():
            line = car.lines.get_nowait()
            if line.startswith('G:') and '=' in line:
                k, v = line[2:].split('='); cur[k] = float(v)
        theta = np.array([cur.get(n, np.mean(BOUNDS[n])) for n in names])
        lo = np.array([BOUNDS[n][0] for n in names]); hi = np.array([BOUNDS[n][1] for n in names])
        rng = np.random.default_rng(0)
        best = (1e9, theta.copy())
        with open(a.csv, 'a', newline='') as f:
            w = csv.writer(f)
            for it in range(a.iters):
                # (1+1)-ES com passo adaptativo: um chute por tentativa, aceita se melhorou
                cand = np.clip(theta + rng.normal(0, a.sigma, theta.size) * (hi - lo), lo, hi) if it else theta.copy()
                g = dict(zip(names, cand))
                print(f'\n=== tentativa {it + 1}/{a.iters}: ' + ', '.join(f'{k}={v:.3g}' for k, v in g.items()))
                await car.set_gains(g)
                input('  carrinho na largada -> Enter e ligue (botão ou ON)... ')
                laps, lost = await car.collect_laps(a.laps, a.timeout)
                await car.send('OF')
                sc = score(laps, lost, a.laps)
                w.writerow([time.strftime('%H:%M:%S'), sc, laps, lost] + list(cand)); f.flush()
                print(f'  nota {sc:.2f} (melhor {best[0]:.2f})')
                if sc < best[0]:
                    best = (sc, cand.copy()); theta = cand.copy(); a.sigma *= 1.2
                else:
                    a.sigma *= 0.85
        print('\nmelhor: ' + ', '.join(f'{k}={v:.3g}' for k, v in zip(names, best[1])))
        await car.set_gains(dict(zip(names, best[1]))); await car.send('GS')
        print('ganhos gravados na NVS (GS)')


if __name__ == '__main__':
    asyncio.run(main())
