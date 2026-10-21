"""Behavioral checks of the reverse-capable standoff and its evidence."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from head_on_core import Settings, solve
from recording import Recording
from run_deadlock import run_deadlock
from verify_bundle import audit_case


class Deadlock(unittest.TestCase):
    def setUp(self):
        self.scene = json.loads((ROOT/'config/deadlock.json').read_text())

    def test_mutual_wait_with_feasible_retreats(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)/'head_on_deadlock'
            result = run_deadlock(directory, self.scene)
            self.assertEqual(result['status'], 'deadlock', result)
            self.assertGreaterEqual(result['deadlock_detected_at_s'], self.scene['deadlock_window_s'])
            self.assertEqual(result['rejected_solves'], 0)
            self.assertTrue(all(p['feasible'] for p in result['initial_retreat_probes'].values()))
            self.assertLess(result['initial_retreat_probes']['A']['first_acceleration'], 0)
            self.assertGreater(result['initial_retreat_probes']['B']['first_acceleration'], 0)
            self.assertTrue(audit_case(directory, video=False)['passed'])
            recording = Recording(directory)
            self.assertEqual(recording.config['road']['bays'], [])
            for car in ('A', 'B'):
                final = recording.sample(car, recording.end)
                self.assertAlmostEqual(final['x'], self.scene['initial_states_x_v'][car][0], places=5)
                self.assertEqual(final['phase'], 'deadlock_detected')
                self.assertTrue(recording.plan(car, recording.end))
            events = json.loads((directory/'events.json').read_text())
            self.assertEqual(events[0]['wait_for'], {'A': 'B', 'B': 'A'})

    def test_reverse_is_admissible_even_with_forward_goal(self):
        cfg = Settings(**self.scene['settings'])
        for car, other, side, reverse_speed in (('A', 'B', 'front', -.5), ('B', 'A', 'rear', .5)):
            state = [self.scene['initial_states_x_v'][car][0], reverse_speed]
            peer = np.tile([*self.scene['initial_states_x_v'][other], 0.],
                           (cfg.horizon*cfg.substeps, 1))
            kwargs = {side: peer, 'cfg': cfg}
            _, enabled = solve(state, 0., np.zeros(cfg.horizon), cfg.epoch,
                               self.scene['goals_x'][car], allow_reverse=True, **kwargs)
            _, disabled = solve(state, 0., np.zeros(cfg.horizon), cfg.epoch,
                                self.scene['goals_x'][car], allow_reverse=False, **kwargs)
            self.assertTrue(enabled['accepted'], enabled)
            self.assertFalse(disabled['accepted'])

    def test_peer_retreat_allows_forward_progress(self):
        cfg = Settings(**self.scene['settings'])
        ticks = np.arange(1, cfg.horizon*cfg.substeps+1)*cfg.dt/cfg.substeps
        for car, other, side, sign in (('A', 'B', 'front', 1), ('B', 'A', 'rear', -1)):
            state = self.scene['initial_states_x_v'][car]
            peer = np.tile([*self.scene['initial_states_x_v'][other], 0.], (len(ticks), 1))
            peer[:, 0] += sign*ticks  # peer retreats at 1 m/s
            u, report = solve(state, 0., np.zeros(cfg.horizon), cfg.epoch,
                              self.scene['goals_x'][car], cfg=cfg, allow_reverse=True, **{side: peer})
            self.assertTrue(report['accepted'], report)
            self.assertGreater(sign*u[0], .01)

    def test_parking_at_goals_is_not_deadlock(self):
        scene = copy.deepcopy(self.scene)
        scene['goals_x'] = {c: s[0] for c, s in scene['initial_states_x_v'].items()}
        with tempfile.TemporaryDirectory() as folder:
            summary = run_deadlock(Path(folder)/'case', scene)
            self.assertEqual(summary['status'], 'inconclusive')
            self.assertIsNone(summary['deadlock_detected_at_s'])


if __name__ == '__main__':
    unittest.main()
