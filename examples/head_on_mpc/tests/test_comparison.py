"""Paired policy behavior, common budgets and switching guards."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from head_on_core import Settings, solve
from run_comparison import (POLICIES, choose_role, make_scene, recovery_feasible,
                            run_trial, switching_allowed)


class Comparison(unittest.TestCase):
    def setUp(self):
        self.base = json.loads((ROOT/'config/comparison.json').read_text())
        self.scene = make_scene(self.base, -3., 0., 20., .5)
        self.cfg = Settings(**self.scene['settings'])

    def test_selection_and_ineligible_shorter_retreat(self):
        states = {c: np.array(s) for c, s in self.scene['initial_states_x_v'].items()}
        self.assertEqual(choose_role(states, self.scene, self.cfg)[0], 'A')
        self.assertEqual(choose_role(states, self.scene, self.cfg, eligible=['B'])[0], 'B')
        self.assertIsNone(choose_role(states, self.scene, self.cfg, eligible=[])[0])
        self.assertTrue(recovery_feasible('A', self.scene, self.cfg))
        self.scene['goals_x']['B'] = self.scene['retreat_x']['A']
        self.assertFalse(recovery_feasible('A', self.scene, self.cfg))

    def test_dwell_and_improvement_prevent_chatter(self):
        self.assertFalse(switching_allowed(4., 0., 10., 1., self.scene))
        self.assertFalse(switching_allowed(6., 0., 10., 9.5, self.scene))
        self.assertTrue(switching_allowed(6., 0., 10., 8., self.scene))

    def test_terminal_stop_and_road_bounds(self):
        cfg = self.cfg
        u, report = solve([0., 1.], 0., np.zeros(cfg.horizon), cfg.epoch, 5.,
                          cfg=cfg, allow_reverse=True, road=self.scene['road'], terminal_stop=True)
        self.assertTrue(report['accepted'], report)
        self.assertAlmostEqual(1.+cfg.dt*sum(u), 0., places=5)
        _, report = solve([self.scene['road']['xmax'], 1.], 0., np.zeros(cfg.horizon),
                          cfg.epoch, 30., cfg=cfg, road=self.scene['road'], terminal_stop=True)
        self.assertFalse(report['accepted'])

    def test_moving_bootstrap_rejection_is_separate_from_timeout(self):
        scene = make_scene(self.base, -3., .5, 20., .5)
        with tempfile.TemporaryDirectory() as folder:
            summary = run_trial(Path(folder)/'moving', scene, 'checked')
            self.assertEqual(summary['failure'], 'optimization_rejected')
            self.assertFalse(summary['timeout'])
            self.assertEqual(summary['simulation_seconds'], 0.)
            self.assertIsNone(summary['task_completion_time_s'])
            self.assertIsNone(summary['recovery_time_s'])

    def test_all_policies_shared_budget_and_raw_safety(self):
        with tempfile.TemporaryDirectory() as folder:
            for policy in POLICIES:
                path = Path(folder)/policy
                summary = run_trial(path, self.scene, policy)
                self.assertEqual(summary['status'], 'timeout' if policy == 'dmpc' else 'completed', summary)
                self.assertGreaterEqual(summary['minimum_sampled_gap_m'], self.scene['sampled_gap_min_m'])
                self.assertEqual(summary['rejected_execution_solves'], 0)
                reports = json.loads((path/'solves.json').read_text())
                for cycle in {r['cycle'] for r in reports}:
                    group = [r for r in reports if r['cycle'] == cycle]
                    self.assertEqual(sorted(r['purpose'] for r in group), ['execute', 'execute', 'probe', 'probe'])
                    self.assertTrue(all(r['iterations'] <= self.cfg.solver_maxiter for r in group))
                if policy == 'dmpc':
                    self.assertFalse(summary['spontaneous_reverse'])
                    self.assertIsNotNone(summary['stall_detected_at_s'])
                else:
                    self.assertIsNotNone(summary['recovery_time_s'])
                    self.assertEqual(summary['role_switch_count'], 0)


if __name__ == '__main__':
    unittest.main()
