"""Failure mechanisms of continuous handoff, actual clearance and release."""
import copy
import csv
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from head_on_core import Settings
from run_comparison import make_scene
from run_exit_screening import generate_exit
from pass_release_core import (body_cleared, continuity, geometry, load_checkpoint, parked_check,
                               release_check, run_branch)
from run_pass_release import failed_exit_branch


class GeometryAndRelease(unittest.TestCase):
    def setUp(self):
        base=json.loads((ROOT/'config/comparison.json').read_text())
        self.scene=make_scene(base,-3.,0.,20.,1.)
        self.cfg=Settings(**self.scene['settings'])
        self.options=json.loads((ROOT/'config/pass_release.json').read_text())
        self.geo=geometry(self.scene,self.cfg,self.options)
        self.actual={'A':[-14.7,0.,3.], 'B':[-21.75,0.,0.]}

    def gate(self, actual=None, **kwargs):
        return release_check(actual or self.actual,'B','B',-21.75,self.geo['conflict_region'],
            self.scene,self.cfg,'A',np.array(self.actual['A']),self.options,
            kwargs.get('latest_prediction_passed',True),kwargs.get('no_failure',True),
            kwargs.get('hold_seconds',.8),kwargs.get('latest_prediction_stays_clear',True))

    def test_center_crossing_and_future_clearance_cannot_release(self):
        actual=copy.deepcopy(self.actual);actual['B']=[-10.1,0.,0.]
        self.assertFalse(body_cleared(actual['B'],'B',self.geo['conflict_region'],self.cfg,self.options))
        self.assertFalse(self.gate(actual, latest_prediction_stays_clear=True)['passed'])
        actual['B'][0]=-12.84 # Rear still short of the release boundary (-10.5).
        self.assertFalse(self.gate(actual)['passed'])
        self.assertTrue(self.gate()['passed'])
        self.assertFalse(self.gate(latest_prediction_passed=False)['passed'])
        self.assertFalse(self.gate(hold_seconds=.4)['passed'])
        self.assertFalse(self.gate(latest_prediction_stays_clear=False)['passed'])

    def test_positive_direction_uses_rear_not_center(self):
        self.assertFalse(body_cleared([10.1,0.,0.],'A',self.geo['conflict_region'],self.cfg,self.options))
        self.assertTrue(body_cleared([12.85,0.,0.],'A',self.geo['conflict_region'],self.cfg,self.options))

    def test_invalid_target_does_not_expand_road(self):
        original=copy.deepcopy(self.scene['road'])
        self.scene['goals_x']['A']=100.
        with self.assertRaisesRegex(ValueError,'geometry_insufficient'):
            geometry(self.scene,self.cfg,self.options)
        self.assertEqual(original,self.scene['road'])

    def test_drift_or_middle_reentry_fails_despite_safe_endpoints(self):
        ref=np.array(self.actual['A']);motion=np.tile(ref,(3,1))
        motion[1,0]+=.011
        self.assertFalse(parked_check(motion,ref,'A',self.scene,self.cfg,self.options)['passed'])
        motion=np.tile(ref,(3,1));motion[1,2]=0.
        self.assertFalse(parked_check(motion,ref,'A',self.scene,self.cfg,self.options)['passed'])
        self.assertTrue(parked_check(motion[[0,2]],ref,'A',self.scene,self.cfg,self.options)['passed'])
        self.assertFalse(self.gate(no_failure=False)['passed'])


@unittest.skipUnless(importlib.util.find_spec('osqp'),'optional OSQP unavailable')
class ContinuousBranch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.folder=tempfile.TemporaryDirectory()
        cls.root=Path(cls.folder.name)
        base=json.loads((ROOT/'config/comparison.json').read_text())
        cls.scene=make_scene(base,-3.,0.,20.,1.)
        cls.options=json.loads((ROOT/'config/pass_release.json').read_text())
        cls.exit=cls.root/'exit'
        generate_exit(cls.exit,cls.scene,'A',solver_backend='osqp')

    @classmethod
    def tearDownClass(cls):cls.folder.cleanup()

    def test_actual_handoff_preserves_entire_controller_state_and_no_task_completion(self):
        result=run_branch(self.root/'success',self.exit,self.scene,'fixed_A',self.options,.7)
        self.assertTrue(result['released'],result)
        self.assertFalse(result['both_tasks_completed'])
        self.assertTrue(result['continuity_check']['passed'])
        self.assertTrue(all(v==0. for v in result['continuity_check']['residuals'].values()))
        events=[json.loads(s) for s in (self.root/'success'/'events.jsonl').read_text().splitlines()]
        grant=next(e for e in events if e['kind']=='grant')
        original=load_checkpoint(self.exit,self.scene)
        for c in ('A','B'):
            np.testing.assert_array_equal(grant['before']['states'][c][:2],original[1][c])
            self.assertEqual(grant['before']['previous_acceleration'][c],original[4][c])
            np.testing.assert_array_equal(grant['before']['warm_controls'][c],original[5][c])
        self.assertEqual(grant['before'],grant['after'])
        self.assertLess(result['first_actual_body_cleared_time_s'],result['release_time_s'])
        self.assertGreaterEqual(result['stopped_confirmation_time_s']-result['first_stop_candidate_time_s'],.8-1e-8)
        with (self.root/'success'/'actual.csv').open() as stream:
            rows=list(csv.DictReader(stream))
        for car in ('A','B'):
            samples=[r for r in rows if r['car']==car]
            for a,b in zip(samples,samples[1:]):
                dt=float(b['absolute_time_s'])-float(a['absolute_time_s'])
                self.assertGreaterEqual(dt,-1e-9)
                if dt<1e-9:
                    for key in ('x','vx','y'):self.assertAlmostEqual(float(a[key]),float(b[key]),places=9)

    def test_timeout_never_reselects_or_relabels_success(self):
        options={**self.options,'pass_timeout_s':.2}
        path=self.root/'timeout'
        with patch('run_exit_screening.select_baselines',side_effect=AssertionError('no reselection')):
            result=run_branch(path,self.exit,self.scene,'screened',options,.7)
        self.assertFalse(result['released'])
        self.assertFalse(result['overall_success'])
        self.assertTrue(result['exit_success'])
        self.assertEqual(result['role'],'A')
        self.assertEqual(result['failure'],'pass_stage_timeout')
        self.assertEqual(result['phase_primary_calls']['PASS'],0)
        events=[json.loads(s) for s in (path/'events.jsonl').read_text().splitlines()]
        self.assertFalse(any(e['kind']=='release' for e in events))

    def test_invalid_return_status_success_still_stops_without_execution(self):
        from pass_release_core import solve_qp
        def bad(*args,**kwargs):
            u,r=solve_qp(*args,**kwargs)
            return np.full_like(u,10.),{**r,'solver_success':True,'accepted':True}
        with patch('pass_release_core.solve_qp',side_effect=bad):
            result=run_branch(self.root/'invalid',self.exit,self.scene,'fixed_A',self.options,.7)
        self.assertFalse(result['released'])
        self.assertEqual(result['failure'],'primary_independent_acceptance_failed')
        self.assertEqual(result['role'],'A')
        self.assertEqual(result['pass_plus_hold_simulation_s'],0.)

    def test_middle_execution_violation_cannot_be_hidden_by_safe_endpoint(self):
        from pass_release_core import integrate
        def corrupt(state, acceleration, times):
            motion=integrate(state,acceleration,times)
            if abs(state[0]-self.scene['initial_states_x_v']['B'][0])<.01:
                motion[2,0]=self.scene['road']['xmax']+10.
            return motion
        with patch('pass_release_core.integrate',side_effect=corrupt):
            result=run_branch(self.root/'middle_violation',self.exit,self.scene,'fixed_A',self.options,.7)
        self.assertFalse(result['released'])
        self.assertFalse(result['overall_success'])
        self.assertEqual(result['pass_plus_hold_simulation_s'],0.)
        self.assertEqual(result['failure'],'actual_path_failed')
        checks=json.loads((self.root/'middle_violation'/'path_checks.json').read_text())
        self.assertFalse(checks[-1]['passed'])
        self.assertFalse(checks[-1]['applied_to_simulation'])

    def test_nonfinite_return_is_rejected_and_logged(self):
        from pass_release_core import solve_qp
        def nonfinite(*args,**kwargs):
            u,r=solve_qp(*args,**kwargs)
            return np.full_like(u,np.nan),r
        with patch('pass_release_core.solve_qp',side_effect=nonfinite):
            result=run_branch(self.root/'nonfinite',self.exit,self.scene,'fixed_A',self.options,.7)
        self.assertFalse(result['released'])
        records=json.loads((self.root/'nonfinite'/'solves.json').read_text())
        self.assertFalse(records[-1]['independent_acceptance']['finite'])
        self.assertTrue(all(v is None for v in records[-1]['controls']))

    def test_exit_timeout_does_not_proceed_to_pass(self):
        scene={**self.scene,'duration_s':.4}
        directory=self.root/'exit_timeout'
        result=generate_exit(directory,scene,'A',solver_backend='osqp')
        self.assertFalse(result['feasible'])
        summary=failed_exit_branch(self.root/'exit_failed_branch',directory,scene,'fixed_A','A',.7)
        self.assertEqual(summary['failure_phase'],'EXIT')
        self.assertFalse(summary['pass_success'])
        self.assertFalse(summary['released'])


if __name__=='__main__':unittest.main()
