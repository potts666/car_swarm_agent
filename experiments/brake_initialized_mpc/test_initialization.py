#!/usr/bin/env python3
"""Initialization-only scope, current-state seeds, and unchanged reject gate."""
import ast
import importlib.util
import json
import unittest
from pathlib import Path

import numpy as np
import mpc
import initialization
import run

ROOT = Path(__file__).resolve().parent
BASE = ROOT.parent / 'reverse_parking_mpc'


def function(module, name):
    return next(node for node in module.body if isinstance(node, ast.FunctionDef) and node.name == name)


class InitializationOnly(unittest.TestCase):
    def setUp(self):
        self.config = json.loads((ROOT / 'config.json').read_text())

    def test_objective_constraints_model_and_solver_options_are_unchanged(self):
        old = ast.parse((BASE / 'mpc.py').read_text())
        new = ast.parse((ROOT / 'mpc.py').read_text())
        for name in ['rollout', 'pair_gap', 'wrap', 'reference', 'window', 'residuals']:
            self.assertEqual(ast.dump(function(old, name)), ast.dump(function(new, name)))
        old_solve = function(old, 'solve')
        new_solve = function(new, 'solve')
        for name in ['objective', 'inequality', 'equality']:
            self.assertEqual(ast.dump(function(old_solve, name)), ast.dump(function(new_solve, name)))
        old_call = next(node for node in ast.walk(old_solve) if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'minimize')
        new_call = next(node for node in ast.walk(new_solve) if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == 'minimize')
        self.assertEqual(ast.dump(old_call), ast.dump(new_call))
        for filename in ['config.json', 'reference_A_controls.json', 'reference_B_controls.json']:
            self.assertEqual((ROOT / filename).read_bytes(), (BASE / filename).read_bytes())
        run.verify()

    def test_seeds_are_generated_from_each_actual_state_for_both_directions(self):
        initial_a = np.array([4., -.4, .4, -.42, -.5])
        later_a = np.array([3.8, -.5, .4, -.125, -.1])
        u = initialization.braking_seed(initial_a, self.config, mpc.rollout, 1.)
        changed = initialization.braking_seed(later_a, self.config, mpc.rollout, 1.)
        self.assertFalse(np.array_equal(u, changed))
        for state in [initial_a, later_a, np.array([-5., 0., 0., .895, .1])]:
            seed = initialization.braking_seed(state, self.config, mpc.rollout, 1.)
            path = mpc.rollout(state, seed, self.config)
            self.assertAlmostEqual(path[0, 3], state[3])
            self.assertAlmostEqual(path[-1, 3], 0., places=12)
            self.assertLessEqual(np.max(abs(seed[:, 0])), 1.)
            self.assertGreater(np.linalg.norm(path[-1, :2] - state[:2]), 0)
            self.assertGreaterEqual(np.min(path[:, 3] * np.sign(state[3])), -1e-12)

    def test_rejected_optimizer_with_legal_braking_is_not_executed(self):
        real = mpc.minimize
        from types import SimpleNamespace
        def rejected(fun, x0, **kwargs):
            return SimpleNamespace(x=np.array(x0), success=False, message='injected rejection',
                                   nit=0, fun=float(fun(x0)))
        mpc.minimize = rejected
        try:
            state = np.array(self.config['starts'][0], dtype=float)
            controls = np.array(json.loads((ROOT / 'reference_A_controls.json').read_text()))
            ref = mpc.reference(state, controls, self.config)
            u, report = mpc.solve(state, 0, ref, self.config)
            self.assertFalse(report['accepted'])
            self.assertFalse(report['fallback_executed'])
            self.assertTrue(any(seed['feasible_against_old_peer_prediction'] for seed in report['braking_initializations'].values()))
        finally:
            mpc.minimize = real

    def test_retries_reproduce_known_six_second_solution_without_phase_one(self):
        problem = json.loads((ROOT.parent / 'local_problem_diagnosis/results/problem.json').read_text())
        controls = np.array(json.loads((ROOT / 'reference_A_controls.json').read_text()))
        ref = mpc.reference(self.config['starts'][0], controls, self.config)
        u, report = mpc.solve(np.array(problem['state']), 15, ref, self.config,
                              np.array(problem['warm_input']), np.array(problem['peer_input']))
        self.assertTrue(report['accepted'], report)
        self.assertEqual(report['attempts'][-1]['initialization'], 'max_brake_hold_steer')
        self.assertFalse(report['fallback_executed'])
        self.assertTrue(all(not seed['joint_current_execution_safety_certified'] for seed in report['braking_initializations'].values()))
        np.testing.assert_array_equal(u, report['attempts'][-1]['result_controls'])


if __name__ == '__main__':
    unittest.main()
