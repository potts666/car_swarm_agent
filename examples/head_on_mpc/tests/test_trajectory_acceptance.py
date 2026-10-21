import json
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from head_on_core import Settings, build_longitudinal_qp, solve
from run_comparison import make_scene
from run_exit_screening import generate_exit
from trajectory_acceptance import audit_controls


class Acceptance(unittest.TestCase):
    def setUp(self):
        base = json.loads((ROOT/'config/comparison.json').read_text())
        self.scene = make_scene(base, -3., 0., 20., 1.)
        self.cfg = Settings(**self.scene['settings'])

    def test_qp_is_strictly_convex_and_matches_direct_residual(self):
        cfg = self.cfg
        state, target = self.scene['initial_states_x_v']['A'], self.scene['retreat_x']['A']
        H, q, A, b = build_longitudinal_qp(state, 0., np.zeros(cfg.horizon), cfg.epoch, target,
                                         cfg=cfg, road=self.scene['road'], allow_reverse=True, terminal_stop=True)
        self.assertGreater(np.linalg.eigvalsh(H)[0], 0.)
        np.testing.assert_allclose(H, H.T, atol=1e-12)
        u, report = solve(state, 0., np.zeros(cfg.horizon), cfg.epoch, target,
                          cfg=cfg, road=self.scene['road'], allow_reverse=True, terminal_stop=True)
        audit = audit_controls(state, u, target, cfg, self.scene['road'])
        self.assertTrue(audit['passed'])
        self.assertLessEqual(max(0., np.max(b-A@u)), cfg.tolerance)
        bad = np.full(cfg.horizon, 2*cfg.max_acceleration)
        audit = audit_controls(state, bad, target, cfg, self.scene['road'])
        self.assertFalse(audit['passed'])
        self.assertGreater(audit['residuals']['acceleration_m_s2'], cfg.tolerance)
        self.assertGreater(audit['residuals']['terminal_velocity_m_s'], cfg.tolerance)
        self.assertFalse(audit_controls(state, np.full(cfg.horizon, np.nan), target, cfg, self.scene['road'])['passed'])

    def test_entire_exit_acceptance_independent_of_solver_status(self):
        def flagged(*args, **kwargs):
            u, report = solve(*args, **kwargs)
            report.update(solver_success=False, accepted=False, message='forced nonconverged status on unchanged solution')
            return u, report
        with tempfile.TemporaryDirectory() as folder, patch('run_exit_screening.solve', side_effect=flagged):
            result = generate_exit(Path(folder)/'candidate', self.scene, 'A')
            self.assertTrue(result['feasible'], result)
            self.assertEqual(result['nonconverged_accepted_solves'], result['solve_count'])
            self.assertEqual(result['retry_count'], 0)

    @unittest.skipUnless(importlib.util.find_spec('osqp'), 'optional OSQP dependency unavailable')
    def test_equivalent_qp_completes_long_weak_braking_exit(self):
        base = json.loads((ROOT/'config/comparison.json').read_text())
        scene = make_scene(base, -3., 0., 28., .5)
        with tempfile.TemporaryDirectory() as folder:
            result = generate_exit(Path(folder)/'candidate', scene, 'B', solver_backend='osqp')
            self.assertTrue(result['feasible'], result)
            self.assertEqual(result['rejected_solves'], 0)
            self.assertEqual(result['retry_count'], 0)

    def test_success_status_cannot_override_invalid_controls(self):
        def unsafe(*args, **kwargs):
            u, report = solve(*args, **kwargs)
            return np.full_like(u, 10.), {**report, 'solver_success': True, 'accepted': True}
        with tempfile.TemporaryDirectory() as folder, patch('run_exit_screening.solve', side_effect=unsafe):
            result = generate_exit(Path(folder)/'candidate', self.scene, 'A')
            self.assertFalse(result['feasible'])
            self.assertEqual(result['attempted_time_s'], 0.)


if __name__ == '__main__':
    unittest.main()
