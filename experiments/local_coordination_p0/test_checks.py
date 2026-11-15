import copy
import json
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'examples/head_on_mpc/scripts'))
from bicycle_core import rollout
from checks import (F, shortest_straight, straight_source_certificate,
                    check_seam, independent_rollout)


class P0Checks(unittest.TestCase):
    def setUp(self):
        self.scene = json.loads((ROOT/'examples/head_on_mpc/config/m2b_fourcar.json').read_text())
        self.state = np.array([-40., 3., 0., .5, 0.])
        self.receiver = np.column_stack((self.state-.01, self.state+.01))
        self.context = dict(version=1, owner='A1', deadline_cycle=33, certificate='fixture-v1')

    def seam(self, **changes):
        args = dict(end=self.state, start=self.state, receiver=self.receiver,
                    tail=[.1, 0], required_tail=[.1, 0], context=self.context,
                    expected=self.context, tolerance=1e-10)
        args.update(changes)
        return check_seam(**args)

    def test_discrete_shortest_profiles_and_lower_bounds(self):
        for distance, seconds in [(2, 3), (4, 4)]:
            p = shortest_straight(distance, self.scene)
            self.assertEqual(p['seconds'], seconds)
            self.assertLess(p['shorter_distance_bound'], distance)
            h = F('0.2'); x = v = F(0)
            for a in p['accelerations']:
                self.assertLessEqual(abs(a), 1)
                x += h*v+h*h*a/2; v += h*a
                self.assertTrue(0 <= v <= 2)
            self.assertEqual((x, v), (distance, 0))

    def test_whole_straight_source_box(self):
        source = [[-40.01, -39.99], [2.99, 3.01], [0, 0], [.5, .51], [0, 0]]
        cert = straight_source_certificate(source, [.1]*32, [-52, -10, -5.5, 5.5], self.scene)
        self.assertTrue(cert['passed'])
        self.assertEqual(cert['integration_error'], 0)
        for x in source[0]:
            for y in source[1]:
                for v in source[3]:
                    z = rollout([x, y, 0, v, 0], [(.1, 0., 'BOX')]*32, self.scene)[-1]
                    box = np.array(cert['terminal_box'], dtype=float)
                    self.assertTrue(np.all(z >= box[:, 0]-1e-10))
                    self.assertTrue(np.all(z <= box[:, 1]+1e-10))

    def test_shortest_time_rejects_unsupported_asymmetric_limits(self):
        self.scene['vehicle']['a_min'] = -.5
        with self.assertRaises(ValueError):
            shortest_straight(4, self.scene)

    def test_source_certificate_fails_closed(self):
        box = [[-40, -40], [3, 3], [0, 0], [.5, .5], [0, 0]]
        with self.assertRaises(ValueError):
            straight_source_certificate(box, [2], [-52, -10, -5.5, 5.5], self.scene)
        box[2] = [0, .01]
        with self.assertRaises(ValueError):
            straight_source_certificate(box, [.1], [-52, -10, -5.5, 5.5], self.scene)
        box[2] = [0, 0]
        self.assertFalse(straight_source_certificate(box, [.1], [-41, -39, 2, 4], self.scene)['passed'])

    def test_moving_terminal_shift_is_32_steps(self):
        u = [(.1, 0., 'MOVE')]*32
        z = rollout(self.state, u, self.scene)
        n = round(self.scene['h']/self.scene['sample_s'])
        shifted = u[1:]+[(.1, 0., 'CONTINUE')]
        shifted_z = rollout(z[n], shifted, self.scene)
        self.assertEqual(len(shifted), 32)
        np.testing.assert_allclose(shifted_z[:-n], z[n:], atol=1e-12, rtol=0)
        self.assertGreater(shifted_z[-1, 3], 0)
        wrong = rollout(z[n], u[1:]+[(0., 0., 'ZERO')], self.scene)
        self.assertGreater(abs(wrong[-1, 3]-shifted_z[-1, 3]), .01)

    def test_nonzero_speed_steering_and_continuation(self):
        u = [(0., .05, 'RAMP')]*10+[(0., 0., 'CONTINUE')]*5
        start = [-40, 3, 0, 1, 0]
        n = round(self.scene['h']/self.scene['sample_s'])
        expected = independent_rollout(start, u, self.scene)
        np.testing.assert_allclose(rollout(start, u, self.scene)[::n], expected, atol=1e-9, rtol=0)
        self.assertTrue(np.all(expected[:, 3] > 0))
        self.assertGreater(expected[-1, 2], 0)

    def test_constant_curvature_analytic_calibration(self):
        start = [-40, 3, 0, 1, .2]
        u = [(0., 0., 'ARC')]*10
        z = independent_rollout(start, u, self.scene)[-1]
        k = np.tan(.2)/self.scene['vehicle']['L']; length = 2.
        exact = [-40+np.sin(k*length)/k, 3+(1-np.cos(k*length))/k, k*length, 1, .2]
        np.testing.assert_allclose(z, exact, atol=1e-11, rtol=0)

    def test_legal_full_state_seam(self):
        self.seam()

    def test_rejections_preserve_committed_fixture(self):
        committed = dict(state=self.state.tolist(), controls=[[.1, 0]]*32, context=self.context)
        before = copy.deepcopy(committed)
        jumps = self.state.copy(); jumps[3] += .1
        outside = self.receiver.copy(); outside[0] += 1
        cases = [dict(start=jumps), dict(tail=[0, 0]), dict(receiver=outside),
                 dict(context={**self.context, 'version':0}),
                 dict(context={**self.context, 'owner':'B1'}),
                 dict(context={**self.context, 'deadline_cycle':32})]
        for case in cases:
            with self.subTest(case=case), self.assertRaises(ValueError):
                self.seam(**case)
            self.assertEqual(committed, before)
        # The still-committed input can be integrated; no controller fallback is claimed.
        next_state = rollout(committed['state'], [( .1, 0., 'COMMITTED')], self.scene)[-1]
        self.assertGreater(next_state[0], self.state[0])

    def test_reject_nonfinite_state(self):
        invalid = self.state.copy(); invalid[0] = float('nan')
        with self.assertRaises(ValueError):
            self.seam(start=invalid)


if __name__ == '__main__':
    unittest.main()
