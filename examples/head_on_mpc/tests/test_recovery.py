"""Detection classification and continuous full-task recovery mechanisms."""
import copy
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
from recovery_core import BlockingDetector
from run_comparison import make_scene
from run_recovery import run_variant


class Detector(unittest.TestCase):
    def setUp(self):
        self.base=json.loads((ROOT/'config/comparison.json').read_text())
        self.scene=make_scene(self.base,0.,0.,20.,1.)
        self.settings=json.loads((ROOT/'config/recovery.json').read_text())['detector']
        self.states={c:[*v,0.] for c,v in self.scene['initial_states_x_v'].items()}
        self.pred={c:np.tile(v,(64,1)) for c,v in self.states.items()}

    def feed(self,detector,**kw):
        result=None
        for i in range(15):result=detector.observe(i*.4,self.states,self.pred,True,**kw)
        return result

    def test_close_stationary_without_blocking_is_only_suspected(self):
        result=self.feed(BlockingDetector(self.scene,self.settings))
        self.assertEqual(result['status'],'suspected_stall')
        self.assertFalse(result['triggered'])

    def test_joint_window_and_blocking_trigger(self):
        result=self.feed(BlockingDetector(self.scene,self.settings),blocked={'A':True,'B':True})
        self.assertTrue(result['triggered'])
        self.assertFalse(result['strict_deadlock_proof'])
        self.assertGreaterEqual(result['window_observed_s'],5.)

    def test_intentional_wait_and_no_current_plan_do_not_trigger(self):
        detector=BlockingDetector(self.scene,self.settings)
        result=self.feed(detector,mode='CLEARED_HOLD',blocked={'A':True,'B':True})
        self.assertFalse(result['triggered'])
        result=detector.observe(6.,self.states,None,False)
        self.assertEqual(result['status'],'solver_failure')
        self.assertFalse(result['triggered'])
        for c in self.states:self.scene['goals_x'][c]=self.states[c][0]
        self.assertFalse(self.feed(BlockingDetector(self.scene,self.settings),blocked={'A':True,'B':True})['triggered'])


@unittest.skipUnless(importlib.util.find_spec('osqp'),'existing optional OSQP dependency unavailable')
class Recovery(unittest.TestCase):
    def setUp(self):
        base=json.loads((ROOT/'config/comparison.json').read_text())
        self.scene=make_scene(base,0.,0.,20.,1.)
        self.options=json.loads((ROOT/'config/pass_release.json').read_text())
        self.recovery=json.loads((ROOT/'config/recovery.json').read_text())

    def test_symmetric_selection_is_fixed_and_return_finishes_original_goals(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'run'
            result=run_variant(path,self.scene,self.options,self.recovery,'E3')
            self.assertTrue(result['both_tasks_completed'],result)
            self.assertTrue(result['released']);self.assertTrue(result['mutual_blocking_resolved'])
            self.assertEqual(result['role'],'A');self.assertEqual(result['role_switch_count'],0)
            selected=json.loads((path/'selection.json').read_text())
            self.assertEqual(selected['actual_checkpoint_before'],selected['actual_checkpoint_after'])
            self.assertFalse(selected['used_pass_results'])
            branch=json.loads((path/'execution'/'summary.json').read_text())
            for c,s in branch['final_actual_states'].items():
                self.assertLessEqual(abs(s[0]-self.scene['goals_x'][c]),self.scene['arrival_position_tolerance_m'])
                self.assertLessEqual(abs(s[1]),self.scene['arrival_speed_tolerance_m_s'])
                self.assertAlmostEqual(s[2],0.)
            self.assertGreaterEqual(result['minimum_executed_gap_m'],.5-1e-5)
            events=[json.loads(line) for line in (path/'events.jsonl').read_text().splitlines()]
            release=next(e for e in events if e['kind']=='release')
            returning=next(e for e in events if e.get('to')=='RETURN_MERGE')
            self.assertEqual(release['states'],returning['states'])
            self.assertEqual(release['absolute_time_s'],returning['absolute_time_s'])

    def test_both_rejected_candidates_do_not_reselect_or_claim_infeasibility(self):
        import run_recovery
        original=run_recovery.generate_exit
        def rejected(*args,**kwargs):
            result=original(*args,**kwargs)
            return {**result,'feasible':False,'failure':'forced_numerical_rejection'}
        with tempfile.TemporaryDirectory() as folder,patch('run_recovery.generate_exit',side_effect=rejected):
            path=Path(folder)/'run'
            result=run_variant(path,self.scene,self.options,self.recovery,'E3')
            self.assertEqual(result['failure'],'no_accepted_candidate')
            self.assertIsNone(result['role']);self.assertFalse(result['both_tasks_completed'])
            self.assertFalse((path/'execution').exists())
            selected=json.loads((path/'selection.json').read_text())
            self.assertEqual(set(selected['candidates']),{'A','B'})


if __name__=='__main__':unittest.main()
