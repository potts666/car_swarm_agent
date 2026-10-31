import copy
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from bicycle_core import step,rollout,corners
from longitudinal_joint import routes
from linearized_joint import (BicycleController,rk4_jacobians,condensed,body_support,
    STATE_TRUST,freeze_corridors)
SCENE=json.loads((Path(__file__).resolve().parents[1]/'config/entry_bicycle.json').read_text())


class LinearizedTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.path=routes(SCENE)

    def test_rk4_derivatives_match_independent_finite_difference(self):
        z=np.array([2.,1.,.4,1.2,.3]);u=np.array([.2,-.1]);h=.2;p=SCENE['vehicle']
        A,B=rk4_jacobians(z,u,h,p);eps=1e-6
        numeric=np.column_stack([(step(z+eps*np.eye(5)[j],u,h,p)-step(z-eps*np.eye(5)[j],u,h,p))/(2*eps) for j in range(5)])
        np.testing.assert_allclose(A,numeric,atol=3e-10)
        numeric=np.column_stack([(step(z,u+eps*np.eye(2)[j],h,p)-step(z,u-eps*np.eye(2)[j],h,p))/(2*eps) for j in range(2)])
        np.testing.assert_allclose(B,numeric,atol=3e-10)

    def test_affine_nominal_defect_matches_dense_reference(self):
        inputs=np.tile([.1,.01],(SCENE['N'],1));z=np.array([0.,0.,.2,.5,.2])
        dense=rollout(z,[(a,w,'TEST') for a,w in inputs],SCENE);reference=dense[::10]
        base,M,_=condensed(z,reference,inputs,SCENE)
        predicted=base+np.einsum('kij,j->ki',M,inputs.ravel())
        np.testing.assert_allclose(predicted,reference[1:],atol=1e-12)

    def test_heading_box_body_envelope_contains_rotating_corners(self):
        for heading in (0.,.7,np.pi):
            support=body_support(heading,SCENE['vehicle'])
            for yaw in np.linspace(heading-STATE_TRUST[2],heading+STATE_TRUST[2],21):
                body=corners([[0.,0.,yaw,0.,0.]],SCENE['vehicle'])[0]
                self.assertTrue(np.all(body>=support[:2]-1e-12))
                self.assertTrue(np.all(body<=support[2:]+1e-12))

    def test_deadline_rejects_instead_of_executing_reference(self):
        controller=BicycleController(SCENE,self.path,3)
        with patch('linearized_joint.DEADLINE_S',1e-12):
            _,_,record=controller.predict({'A':False,'B':False})
        self.assertFalse(record['accepted'])
        self.assertTrue(record['deadline_exceeded'])
        self.assertEqual(record['bicycle_numeric_qp_calls'],0)

    def test_rejected_reference_is_logged_before_rollout(self):
        controller=BicycleController(SCENE,self.path,1)
        solution={c:np.full(SCENE['N'],1e9) for c in ('A','B')}
        diagnostic=dict(accepted=False,message='test rejected',free_variables=64)
        with patch('longitudinal_joint.solve',return_value=(solution,diagnostic)):
            _,_,record=controller.predict({'A':False,'B':False})
        self.assertFalse(record['accepted'])
        self.assertEqual(record['message'],'reference QP rejected')
        self.assertEqual(record['bicycle_numeric_qp_calls'],0)

    def test_second_linearization_recovers_same_curve_checkpoint(self):
        records=[]
        for updates in (1,3):
            controller=BicycleController(SCENE,self.path,updates);b=controller.agents['B']
            b.index=1;b.z=np.array(self.path['B'][1]['start']);b.z[4]=self.path['B'][1]['delta']
            _,_,record=controller.predict({'A':True,'B':False});records.append(record)
        self.assertFalse(records[0]['accepted'])
        self.assertTrue(records[1]['accepted'])
        self.assertEqual(records[1]['qp_updates'],2)
        self.assertLess(records[1]['mapping_errors']['B']['position_m'],1e-5)

    def test_initial_qp_is_convex_affine_and_replayed(self):
        controller=BicycleController(SCENE,self.path,1)
        _,_,record=controller.predict({'A':False,'B':False})
        self.assertTrue(record['accepted'],record['message'])
        self.assertEqual(record['qp_updates'],1)
        self.assertGreater(record['hessian_min_eigenvalue'],0.)
        self.assertTrue(record['affine_constraints'])
        self.assertLess(record['affine_violation'],1e-5)
        self.assertEqual(record['variables'],128)


if __name__=='__main__':unittest.main()
