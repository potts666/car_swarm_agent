import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from longitudinal_joint import routes, solve
from linearized_joint import BicycleController
from run_joint_e2 import run_b1

SCENE=json.loads((Path(__file__).resolve().parents[1]/'config/entry_bicycle.json').read_text())


class TimingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.path=routes(SCENE)

    def test_reference_rejection_is_timed_without_commit(self):
        ctrl=BicycleController(SCENE,self.path,3)
        acceleration,record=solve(ctrl.agents,{'A':False,'B':False},SCENE)
        record.update(accepted=False,message='maximum iterations reached')
        bus=SimpleNamespace(received={c:dict(owner_id=None,receive_cycle=0,version=1) for c in ('A','B')},log=[])
        with tempfile.TemporaryDirectory() as temporary, patch('longitudinal_joint.solve',return_value=(acceleration,record)):
            directory=Path(temporary)/'rejected'
            summary=run_b1(SCENE,self.path,directory,bus,controller=ctrl)
            planning=json.loads((directory/'planning.json').read_text())
            self.assertEqual(len(planning),1)
            self.assertEqual(planning[0]['rejection_layer'],'reference_QP')
            self.assertFalse(planning[0]['committed'])
            self.assertGreater(planning[0]['total_cycle_wall_s'],0.)
            self.assertEqual(summary['failure'],'joint_QP_rejected')

    def test_failed_future_release_is_timed_before_prediction(self):
        ctrl=BicycleController(SCENE,self.path,3)
        ctrl.agents['A'].z[0]=30.
        b=ctrl.agents['B'];b.index=3;b.mode='WAIT';b.ready=True
        b.z[:]=self.path['B'][2]['end'];b.z[4]=0.
        bus=SimpleNamespace(received={c:dict(owner_id='A',receive_cycle=0,version=1) for c in ('A','B')},log=[],
                            request=lambda requests:('A',{'A':True,'B':False}),ready=lambda car:None)
        witness=dict(check={'passed':False},wall_s=.01,solves=[],qp_calls=0)
        with tempfile.TemporaryDirectory() as temporary, patch.object(ctrl,'future_witness',return_value=witness), patch.object(ctrl,'predict') as prediction:
            directory=Path(temporary)/'release'
            summary=run_b1(SCENE,self.path,directory,bus,controller=ctrl)
            planning=json.loads((directory/'planning.json').read_text())
            prediction.assert_not_called()
            self.assertEqual(summary['failure'],'full_future_release_QP_rejected')
            self.assertEqual(len(planning),1)
            self.assertEqual(planning[0]['future_check_wall_s'],.01)
            self.assertEqual(planning[0]['rejection_layer'],'full_future_release_QP_rejected')
            self.assertFalse(planning[0]['committed'])

    def test_exception_exit_has_a_complete_cycle_record(self):
        ctrl=BicycleController(SCENE,self.path,3)
        bus=SimpleNamespace(received={c:dict(owner_id=None,receive_cycle=0,version=1) for c in ('A','B')},log=[])
        before={c:a.z.copy() for c,a in ctrl.agents.items()}
        with tempfile.TemporaryDirectory() as temporary, patch.object(ctrl,'predict',side_effect=TimeoutError('test timeout')):
            directory=Path(temporary)/'exception'
            run_b1(SCENE,self.path,directory,bus,controller=ctrl)
            planning=json.loads((directory/'planning.json').read_text())
            self.assertEqual(len(planning),1)
            self.assertEqual(planning[0]['outcome'],'exception')
            self.assertEqual(planning[0]['exception'],'test timeout')
            self.assertFalse(planning[0]['committed'])
            for c,a in ctrl.agents.items():self.assertEqual(a.z.tolist(),before[c].tolist())


if __name__=='__main__':unittest.main()
