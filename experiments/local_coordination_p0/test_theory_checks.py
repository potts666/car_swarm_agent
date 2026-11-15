import unittest
from fractions import Fraction as F
import json
from pathlib import Path
import sys

import numpy as np

from theory_checks import radius_step, curved_fixture_certificate
from checks import independent_rollout

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'examples/head_on_mpc/scripts'))
from bicycle_core import rollout


class MotionTheoryChecks(unittest.TestCase):
    def test_radius_example_matches_rational_upper_bounds(self):
        rp, rpsi = radius_step(F(5,1000),F(1,1000),F(2,100),F(1,1000))
        self.assertLessEqual(rp,F(7535,1000000))
        self.assertLessEqual(rpsi,F(1505,1000000))
        with self.assertRaises(ValueError):
            radius_step(F(-1),F(0),F(0),F(0))

    def test_full_domain_and_six_pair_analytic_bounds(self):
        c = curved_fixture_certificate()
        self.assertTrue(c['passed'])
        self.assertEqual(len(c['pair_lower_bounds']),6)
        self.assertGreater(c['minimum_road_margin'], F(23,100))
        self.assertGreaterEqual(c['minimum_pair_margin'], 0)
        self.assertEqual(c['nominal_body_displacement_bound'],F(75883,298500))
        self.assertEqual(c['terminal_continuation'],[0,0])
        self.assertFalse(c['full_task_route_certified'])
        self.assertFalse(c['same_nominal_source_as_current_task'])
        kappa=F(11,20)/F(27,10);sec=F(64,49)
        mc=1+4*kappa+F(1807,500)*(kappa+2*sec/F(27,10)+4*kappa*kappa)
        self.assertLess(mc,15)  # validates the continuous chord allowance used by the interval checkers

    def test_feedback_domain_auxiliary_boundary_samples(self):
        # Auxiliary debugging only: the whole-domain claim uses the inequalities,
        # not these finitely many boundary samples or the numerical integrator.
        c = curved_fixture_certificate()
        scene = json.loads((ROOT/'examples/head_on_mpc/config/m2b_fourcar.json').read_text())
        nominal = [(float(a),float(w),'REF') for a,w in c['reference_inputs']]
        centers = independent_rollout([0,0,0,.5,0],nominal,scene)
        for signs in [(-1,-1,-1,-1),(1,1,1,1),(-1,1,-1,1),(1,-1,1,-1)]:
            z = np.array([.003*signs[0],.004*signs[1],.001*signs[2],.5+.02*signs[3],.001*signs[0]])
            for j,(a,w,_) in enumerate(nominal):
                u = [a-(z[3]-centers[j,3])/scene['h'], w-(z[4]-centers[j,4])/scene['h'],'FEEDBACK']
                self.assertLessEqual(max(abs(u[0]),abs(u[1])),1+1e-11)
                z = rollout(z,[u],scene)[-1]
                radius = c['stages'][j]['receiver_radii']
                self.assertLessEqual(np.linalg.norm(z[:2]-centers[j+1,:2]),float(radius['position_ball'])+1e-10)
                self.assertLessEqual(abs(z[2]-centers[j+1,2]),float(radius['heading'])+1e-10)
                self.assertLessEqual(abs(z[3]-centers[j+1,3]),1e-11)
                self.assertLessEqual(abs(z[4]-centers[j+1,4]),1e-11)
            hold = rollout(z,[(0,0,'HOLD')],scene)[-1]
            np.testing.assert_allclose(z,hold,atol=1e-12,rtol=0)


if __name__ == '__main__':
    unittest.main()
