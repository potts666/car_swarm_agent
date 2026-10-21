import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from head_on_core import Settings
from run_comparison import make_scene
from run_exit_screening import check_path, generate_exit, select_baselines, validate_start


class ExitScreening(unittest.TestCase):
    def setUp(self):
        base = json.loads((ROOT/'config/comparison.json').read_text())
        self.scene = make_scene(base, -3., 0., 20., 1.)
        self.cfg = Settings(**self.scene['settings'])

    def test_rejects_moving_or_unsafe_start(self):
        self.scene['initial_states_x_v']['A'][1] = .1
        with self.assertRaises(ValueError):
            validate_start(self.scene, self.cfg)
        self.scene['initial_states_x_v']['A'] = self.scene['initial_states_x_v']['B'].copy()
        with self.assertRaises(ValueError):
            validate_start(self.scene, self.cfg)

    def test_safe_endpoints_do_not_hide_intermediate_road_violation(self):
        a = np.tile([*self.scene['initial_states_x_v']['A'], 0.], (3, 1))
        b = np.tile([*self.scene['initial_states_x_v']['B'], 0.], (3, 1))
        a[1, 2] = -3. # Outside road, endpoints unchanged and safe.
        self.assertTrue(check_path({'A': a[[0, 2]], 'B': b[[0, 2]]},
                                  {'A': [0.], 'B': [0.]}, self.scene, self.cfg, 'B', b[0, 0])['passed'])
        audit = check_path({'A': a, 'B': b}, {'A': [0.], 'B': [0.]}, self.scene, self.cfg, 'B', b[0, 0])
        self.assertFalse(audit['passed'])
        self.assertIn('A:road_body_containment', audit['reasons'])

    def test_filter_uses_full_exit_result_even_if_endpoint_passes(self):
        candidates = {'A': {'feasible': False, 'terminal_passed': True},
                      'B': {'feasible': True, 'terminal_passed': True}}
        result = select_baselines(candidates, self.scene)
        self.assertFalse(result['fixed_A']['feasible'])
        self.assertEqual(result['screened']['role'], 'B')
        candidates['B']['feasible'] = False
        self.assertIsNone(select_baselines(candidates, self.scene)['screened']['role'])

    def test_both_complete_exits_keep_waiter_still(self):
        with tempfile.TemporaryDirectory() as folder:
            for role in ('A', 'B'):
                path = Path(folder)/role
                result = generate_exit(path, self.scene, role)
                self.assertTrue(result['feasible'], result)
                self.assertTrue(result['terminal_passed'])
                audits = json.loads((path/'path_checks.json').read_text())
                self.assertTrue(all(a['passed'] for a in audits))
                self.assertTrue(any(a.get('kind') == 'whole_prediction' for a in audits))
                waiter = result['waiter']
                np.testing.assert_allclose(result['final_states_x_v'][waiter],
                                           self.scene['initial_states_x_v'][waiter], atol=.01)
                self.assertGreater(result['solve_count'], 2) # Multiple receding horizons.


if __name__ == '__main__':
    unittest.main()
