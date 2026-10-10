#!/usr/bin/env python3
"""Regression tests for exact snapshot replay and constraint diagnostics."""
import copy
import json
import unittest
from pathlib import Path

import numpy as np
import diagnose as d


class LocalProblemDiagnosis(unittest.TestCase):
    def setUp(self):
        self.problem = json.loads((d.ROOT / 'results/problem.json').read_text())

    def test_frozen_inputs_and_reference_window_are_exact(self):
        current = d.extract()
        for key in ['state', 'peer_input', 'reference_states', 'reference_controls',
                    'warm_input', 'gear', 'config']:
            self.assertEqual(current[key], self.problem[key])
        target, controls, gear = d.mpc.window(d.reference_container(self.problem), 15,
                                            self.problem['config'])
        np.testing.assert_array_equal(target, self.problem['reference_states'])
        np.testing.assert_array_equal(controls, self.problem['reference_controls'])
        np.testing.assert_array_equal(gear, self.problem['gear'])

    def test_bounded_braking_integrates_to_stop_without_state_reset(self):
        controls = d.braking(self.problem, 1.)
        prediction = d.mpc.rollout(self.problem['state'], controls, self.problem['config'])
        self.assertAlmostEqual(prediction[0, 3], -.4191708215668613)
        self.assertLess(prediction[4, 3], -.019)
        self.assertAlmostEqual(prediction[8, 3], 0., places=12)
        self.assertGreater(np.linalg.norm(prediction[-1, :2] - prediction[0, :2]), .08)
        self.assertTrue(d.evaluate(self.problem, controls)['feasible_same_local_constraints'])
        self.assertLessEqual(np.max(abs(controls[:, 0])), 1.)

    def test_original_seeds_violate_predicted_clearance(self):
        for key in ['reference_controls', 'warm_input']:
            checked = d.evaluate(self.problem, np.array(self.problem[key]))
            self.assertFalse(checked['feasible_same_local_constraints'])
            self.assertGreater(checked['constraints']['predicted_peer_body_gap']['maximum_violation'], .06)

    def test_collision_and_input_constraints_are_not_omitted(self):
        problem = copy.deepcopy(self.problem)
        controls = d.braking(problem, 1.)
        future = d.mpc.rollout(problem['state'], controls, problem['config'])[1:]
        problem['peer_input'] = future.tolist()
        checked = d.evaluate(problem, controls)
        self.assertFalse(checked['feasible_same_local_constraints'])
        self.assertGreater(checked['constraints']['predicted_peer_body_gap']['maximum_violation'], 2.)
        controls[0, 0] = 2.
        checked = d.evaluate(self.problem, controls)
        self.assertGreater(checked['constraints']['acceleration_bounds']['maximum_violation'], .9)

    def test_this_horizon_has_no_fixed_stop_equation_and_allows_waiting(self):
        checked = d.evaluate(self.problem, d.braking(self.problem, .5))
        self.assertEqual(checked['stop_equalities'], [])
        self.assertTrue(all(gear == -1 for gear in self.problem['gear']))
        self.assertGreaterEqual(checked['constraints']['reference_direction']['min_residual'], -1e-12)


if __name__ == '__main__':
    unittest.main()
