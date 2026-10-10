#!/usr/bin/env python3
"""Physical direction and feedback regression checks for the new baseline."""
import json
import unittest
from pathlib import Path

import numpy as np
import mpc
import run

ROOT = Path(__file__).resolve().parent


class SignedMPC(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((ROOT / 'config.json').read_text())
        self.controls = np.array(json.loads((ROOT / 'reference_A_controls.json').read_text()))
        self.ref = mpc.reference(self.config['starts'][0], self.controls, self.config)

    def test_signed_reference_is_model_consistent_and_reaches_original_goal(self):
        ref = self.ref
        self.assertLess(np.min(ref['states'][:, 3]), -.4)
        self.assertLess(np.min(np.diff(ref['states'][:, 0])), 0)
        self.assertGreater(np.max(ref['states'][:, 2]), .8)
        self.assertTrue(run.arrived(ref['states'][-1], self.config['goals'][0], self.config))
        self.assertTrue(np.all(ref['gear'] <= 0))
        for cycle, control in enumerate(self.controls):
            state = ref['states'][cycle * self.config['time']['model_substeps']]
            _, violations = run.executed_checks(state, control, self.config, ref['gear'][cycle])
            self.assertLessEqual(max(violations.values()), 1e-5)

    def test_solver_responds_to_measured_state_not_just_reference_playback(self):
        cycle = 10
        state = self.ref['states'][cycle * self.config['time']['model_substeps']].copy()
        normal, report = mpc.solve(state, cycle, self.ref, self.config)
        self.assertTrue(report['accepted'], report)
        state[0] += .08
        disturbed, changed = mpc.solve(state, cycle, self.ref, self.config)
        self.assertTrue(changed['accepted'], changed)
        self.assertGreater(np.linalg.norm(normal - disturbed), 1e-3)

    def test_gear_change_requires_zero_at_boundary(self):
        state = np.array([0., 0., 0., .2, 0.])
        # A direct forward->reverse crossing inside one 0.4 s control is invalid.
        controls = np.tile([-.8, 0.], (2, 1))
        _, violations = run.executed_checks(state, controls[0], self.config, 1)
        self.assertGreater(violations['gear'], .1)
        constraints, zeros = mpc.residuals(state, controls, self.config, np.array([1, -1]))
        self.assertGreater(np.max(abs(zeros)), .1)
        self.assertLess(np.min(constraints), 0)

    def test_wrong_orientation_does_not_count_as_arrival(self):
        state = np.r_[self.config['goals'][0], 0., 0.]
        self.assertTrue(run.arrived(state, self.config['goals'][0], self.config))
        state[2] = np.pi
        self.assertFalse(run.arrived(state, self.config['goals'][0], self.config))

    def test_geometry_mismatch_is_rejected(self):
        self.config['vehicle']['width_m'] = 2.
        with self.assertRaises(ValueError):
            mpc.residuals(self.config['starts'][0], self.controls[:2], self.config, np.array([0, 0]))


if __name__ == '__main__':
    unittest.main()
