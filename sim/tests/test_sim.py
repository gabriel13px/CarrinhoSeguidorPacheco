"""Testes de integração do simulador (rodar: python3 -m unittest discover -s sim/tests -v)."""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from lfsim import run, Track  # noqa: E402
from lfsim.core_bridge import Core, MODE_ZIGZAG, MODE_CROSS, MODE_LOST, SEG_ZIGZAG, SEG_BAR  # noqa: E402


class TestBridge(unittest.TestCase):
    def test_structs_match_c(self):
        Core().close()   # lib() valida os sizeof das structs ao carregar


class TestFullRun(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.track = Track()
        cls.res = run(cls.track, laps=3, seed=1)

    def test_finishes_three_laps(self):
        s = self.res.summary
        self.assertTrue(s['finished'], s)
        self.assertGreaterEqual(len(s['lap_times']), 3)

    def test_map_closed_after_first_lap(self):
        s = self.res.summary
        self.assertTrue(s['has_map'])
        types = [m['type'] for m in self.res.map]
        self.assertIn(SEG_ZIGZAG, types)
        self.assertGreaterEqual(types.count(SEG_BAR), 6)
        lap1 = self.res.log['t'][self.res.log['has_map'] == 1][0]
        self.assertLess(lap1, s['lap_times'][0] + 4.0)   # fechou o mapa logo depois da 1ª volta

    def test_mapped_laps_are_faster(self):
        lt = self.res.summary['lap_times']
        self.assertLess(lt[1], 0.85 * lt[0])   # volta 2 (com mapa) bem mais rápida que a de mapeamento
        self.assertLess(lt[2], 0.85 * lt[0])

    def test_zigzag_mode_every_lap(self):
        laps_with_zz = {e['lap'] for e in self.res.events if e['mode'] == MODE_ZIGZAG}
        self.assertTrue({1, 2, 3} <= laps_with_zz, laps_with_zz)

    def test_crossings_handled(self):
        n_cross = sum(1 for e in self.res.events if e['mode'] == MODE_CROSS)
        self.assertGreaterEqual(n_cross, 3 * 6)

    def test_never_far_from_line(self):
        self.assertLess(self.res.summary['max_dlat'], 0.06)
        self.assertLess(self.res.summary['t_lost'], 0.5)

    def test_speed_calibration_converges(self):
        s = self.res.summary
        self.assertLess(abs(s['k_final'] - s['k_true']) / s['k_true'], 0.15)


class TestNoMapLap(unittest.TestCase):
    def test_single_lap_without_map(self):
        res = run(Track(), laps=1, seed=2, t_max=60)
        self.assertGreaterEqual(len(res.summary['lap_times']), 1, res.summary)
        self.assertLess(res.summary['max_dlat'], 0.06)


if __name__ == '__main__':
    unittest.main()
