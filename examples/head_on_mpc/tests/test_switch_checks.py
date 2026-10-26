import copy
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from head_on_core import Settings
from pass_release_core import snapshot
from run_comparison import make_scene
from switch_checks import TransitionGuard


class SwitchChecks(unittest.TestCase):
    def setUp(self):
        base=json.loads((ROOT/'config/comparison.json').read_text())
        self.scene=make_scene(base,-3.,0.,20.,.5);self.cfg=Settings(**self.scene['settings'])
        self.options=json.loads((ROOT/'config/pass_release.json').read_text())
        self.states={'A':np.array([-14.7,0.]),'B':np.array([-21.74,.02])}
        self.lanes={'A':[3.,3.,100.,6.],'B':[0.,0.,100.,6.]}
        self.cp=snapshot(self.states,self.lanes,130.,{'A':0.,'B':0.},{c:np.zeros(16) for c in ('A','B')})
        self.targets={'A':-14.7,'B':-21.75};self.reference=[-14.7,0.,3.]
        self.previous={c:np.zeros(16).tolist() for c in ('A','B')}
        self.previous['B'][1]=.02;self.previous['B'][2]=-.07 # terminal velocity zero, but shifted first interval exceeds .025.

    def check(self,guard,**kwargs):
        return guard.check('PASS','CLEARED_HOLD',self.cp,self.previous,self.targets,('B',),'A',self.reference,**kwargs)

    def test_new_hold_constraint_excludes_shifted_old_sequence(self):
        guard=TransitionGuard(self.scene,self.options)
        with patch('switch_checks.motion_cycle',return_value={'passed':False,'solves':[],'failure':'forced_no_safe_transition'}):
            result=self.check(guard)
        self.assertFalse(result['allowed'])
        shift=result['shifted_candidate']
        self.assertTrue(shift['zero_tail_appended'])
        self.assertIn('B:hold_speed_m_s',shift['failed_constraints'])
        self.assertGreater(shift['local']['B']['residuals']['hold_speed_m_s'],.001)

    def test_new_joint_witness_can_allow_switch_without_mutating_actual_state(self):
        before=copy.deepcopy(self.cp);brake={c:np.zeros(16) for c in ('A','B')};brake['B'][0]=-.02/.4
        result=self.check(TransitionGuard(self.scene,self.options),fresh_controls=brake)
        self.assertFalse(result['shifted_candidate']['passed'])
        self.assertTrue(result['allowed'])
        self.assertEqual(self.cp,before)
        self.assertFalse(result['applied_as_backup'])

    def test_merely_low_terminal_speed_does_not_justify_zero_tail(self):
        self.cp['states']['B'][1]=.001;self.previous={c:np.zeros(16).tolist() for c in ('A','B')}
        with patch('switch_checks.motion_cycle',return_value={'passed':False,'solves':[],'failure':'no_safe_transition'}):
            result=self.check(TransitionGuard(self.scene,self.options))
        self.assertFalse(result['shifted_candidate']['zero_tail_appended'])
        self.assertIsNone(result['shifted_candidate']['controls'])
        self.assertFalse(result['allowed'])

    def test_pair_consistency_rejects_two_incompatible_forecasts(self):
        cp=copy.deepcopy(self.cp);cp['states']={'A':[-2.6,.02,0.],'B':[2.6,-.02,0.]}
        cp['lanes']={c:[0.,0.,100.,6.] for c in ('A','B')}
        controls={c:np.zeros(16) for c in ('A','B')}
        result=TransitionGuard(self.scene,self.options).verify_controls(cp,controls,{'A':21.75,'B':-21.75},(),
            'A',[-2.6,.02,0.],False,'PASS')
        self.assertFalse(result['passed'])
        self.assertGreater(result['local']['A']['residuals']['neighbor_m'],0.)
        self.assertGreater(result['local']['B']['residuals']['neighbor_m'],0.)


if __name__=='__main__':unittest.main()
