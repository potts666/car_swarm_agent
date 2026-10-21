"""Holding constraints, pre-entry feasibility, symmetry and stop crosscheck."""
import csv
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from head_on_core import Settings, matrices
from run_comparison import make_scene
from run_exit_screening import generate_exit
from trajectory_acceptance import audit_controls, solve_qp
from pass_release_core import holding_bounds, hold_path_check, run_branch


@unittest.skipUnless(importlib.util.find_spec('osqp'),'optional OSQP unavailable')
class Holding(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();cls.root=Path(cls.tmp.name)
        base=json.loads((ROOT/'config/comparison.json').read_text())
        cls.scene=make_scene(base,-3.,0.,20.,.5);cls.cfg=Settings(**cls.scene['settings'])
        cls.options=json.loads((ROOT/'config/pass_release.json').read_text())
        cls.exit=cls.root/'exit';generate_exit(cls.exit,cls.scene,'A',solver_backend='osqp')

    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()

    def test_low_speed_hard_constraints_are_symmetric(self):
        cfg=self.cfg;target=0.
        for sign in (-1,1):
            # Motion initially away from the target, as in the old failed hold.
            state=np.array([sign*.000316,sign*.023411])
            bounds=holding_bounds(target,self.scene)
            u,r=solve_qp(state,sign*.405,np.zeros(cfg.horizon),cfg.epoch,target,cfg,self.scene['road'],hold_bounds=bounds)
            self.assertTrue(r['independent_acceptance']['passed'],r)
            ticks=np.arange(1,cfg.horizon*cfg.substeps+1)*cfg.dt/cfg.substeps
            p,v=matrices(ticks,cfg)
            motion=np.column_stack((state[0]+ticks*state[1]+p@u,state[1]+v@u,np.zeros(len(ticks))))
            # QP residual tolerance remains separate from the strict external
            # hold check used for entry and actual execution. No threshold is relaxed.
            self.assertLessEqual(float(np.max(np.abs(motion[:,1])))-.025,cfg.tolerance)
            self.assertLessEqual(float(np.max(np.abs(motion[:,0])))-.08,cfg.tolerance)
            self.assertLessEqual(sign*u[0],(.025-abs(state[1]))/cfg.dt+1e-7)

    def test_stop_candidate_must_also_satisfy_target_no_overshoot(self):
        cfg=self.cfg;state=np.array([-.0001,.023411])
        u=np.zeros(cfg.horizon);u[0]=-state[1]/cfg.dt
        audit=audit_controls(state,u,0.,cfg,self.scene['road'],hold_bounds=holding_bounds(0.,self.scene))
        self.assertLess(abs(u[0]),cfg.max_acceleration)
        self.assertEqual(audit['residuals']['hold_speed_m_s'],0.)
        self.assertEqual(audit['residuals']['hold_position_m'],0.)
        self.assertGreater(audit['residuals']['target_overshoot_m'],cfg.tolerance)
        self.assertFalse(audit['passed'])

    def test_precheck_failure_does_not_enter_holding_or_reset_timer(self):
        def invalid_hold(*args,**kwargs):
            u,r=solve_qp(*args,**kwargs)
            if kwargs.get('hold_bounds') is not None:return np.full_like(u,10.),r
            return u,r
        with patch('pass_release_core.solve_qp',side_effect=invalid_hold):
            result=run_branch(self.root/'precheck_failed',self.exit,self.scene,'fixed_A',self.options,.7,hold_constraints=True)
        self.assertEqual(result['failure'],'hold_mode_precheck_failed')
        self.assertEqual(result['failure_phase'],'PASS')
        self.assertEqual(result['hold_precheck_failures'],1)
        self.assertIsNone(result['first_stop_candidate_time_s'])
        events=[json.loads(s) for s in (self.root/'precheck_failed'/'events.jsonl').read_text().splitlines()]
        self.assertFalse(any(e.get('to')=='CLEARED_HOLD' for e in events))
        self.assertFalse(result['released'])
        self.assertEqual(result['role'],'A')

    def test_full_rerun_preserves_entry_and_makes_old_refusal_pass(self):
        old_path,new_path=self.root/'legacy',self.root/'constrained'
        old=run_branch(old_path,self.exit,self.scene,'fixed_A',self.options,.7)
        new=run_branch(new_path,self.exit,self.scene,'fixed_A',self.options,.7,hold_constraints=True)
        self.assertEqual(old['failure'],'cleared_hold_actual_condition_lost')
        self.assertTrue(new['released'],new)
        self.assertEqual(old['first_stop_candidate_time_s'],new['first_stop_candidate_time_s'])
        self.assertAlmostEqual(new['release_time_s']-new['first_stop_candidate_time_s'],.8,places=9)
        with (old_path/'actual.csv').open() as f:old_rows=list(csv.DictReader(f))
        with (new_path/'actual.csv').open() as f:new_rows=list(csv.DictReader(f))
        prefix=lambda rows:[r for r in rows if r['stage'] in ('EXIT','PARKED_READY','PASS')]
        self.assertEqual(prefix(old_rows),prefix(new_rows))
        pre=json.loads((new_path/'hold_prechecks.json').read_text())[0]
        self.assertTrue(pre['passed']);self.assertEqual(pre['before'],pre['after'])
        self.assertTrue(pre['stop_candidate']['passed']);self.assertFalse(pre['stop_candidate']['applied'])
        self.assertEqual(new['diagnostic_calls'],2);self.assertEqual(new['retries'],0)
        self.assertFalse(new['both_tasks_completed'])


if __name__=='__main__':unittest.main()
