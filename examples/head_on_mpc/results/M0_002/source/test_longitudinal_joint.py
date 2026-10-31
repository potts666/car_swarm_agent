import copy
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from bicycle_core import audit, rollout
from longitudinal_joint import Controller, build_qp, pose, routes, solve
SCENE=json.loads((Path(__file__).resolve().parents[1]/'config/entry_bicycle.json').read_text())


class JointQPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.path=routes(SCENE)

    def test_routes_and_original_goals(self):
        for car in ('A','B'):
            np.testing.assert_allclose(self.path[car][-1]['end'][:3],SCENE['goals'][car][:3],atol=1e-9)
            self.assertTrue(all(p['sweep_check']['passed'] for p in self.path[car]))
        self.assertAlmostEqual(self.path['B'][2]['end'][1],-3.)

    def test_hessian_affine_and_terminal_stop(self):
        ctrl=Controller(SCENE,self.path);g={'A':False,'B':False}
        H,q,A,lo,hi,E,b,bounds,params=build_qp(ctrl.agents,g,SCENE)
        self.assertGreater(np.linalg.eigvalsh(H)[0],0.)
        x=np.arange(len(q))*.001;y=-x*.3
        np.testing.assert_allclose(A@(x+y),A@x+A@y,atol=1e-13)
        u,record=solve(ctrl.agents,g,SCENE)
        self.assertTrue(record['accepted'],record)
        self.assertLess(record['affine_violation'],1e-5)
        self.assertEqual(H.shape,(64,64))
        self.assertLess(params['A']['upper'],self.path['A'][0]['length'])

    def test_reverse_mapping_and_curved_execution(self):
        scene=copy.deepcopy(SCENE)
        z=np.array([0.,0.,.2,0.,.3])
        controls=[(-.2,0.,'REVERSE')]*10
        trajectory=rollout(z,controls,scene)
        length=0.5*.2*(10*scene['h'])**2
        expected=pose(z,length,.3,-1,scene['vehicle'])
        np.testing.assert_allclose(trajectory[-1,:3],expected[:3],atol=1e-8)

    def test_numerical_rest_residual_is_braked_without_state_reset(self):
        ctrl=Controller(SCENE,self.path);b=ctrl.agents['B']
        b.s=self.path['B'][0]['length']-1e-8
        b.z=pose(self.path['B'][0]['start'],b.s,0.,1,SCENE['vehicle'])
        b.z[3]=-2e-8
        before=b.z.copy();b.advance(False)
        np.testing.assert_array_equal(b.z,before)
        self.assertEqual(b.mode,'MOVE');self.assertEqual(b.stop_steps,1)
        commands,predictions,record=ctrl.predict({'A':False,'B':False})
        self.assertTrue(record['accepted'])
        ctrl.commit(commands,predictions,record)
        self.assertLess(abs(b.z[3]),1e-12)
        b.advance(False);self.assertEqual(b.mode,'STEER')

    def test_shift_uses_committed_reference_and_discards_changed_phase(self):
        ctrl=Controller(SCENE,self.path,reference_warm_start=True);N=SCENE['N']
        reference=np.linspace(-.5,.5,2*N).tolist()
        ctrl.remember_reference(dict(accepted=True,solution=[9.]*(4*N),reference_record={'solution':reference}))
        ctrl.agents['B'].mode='STEER'
        rejected=dict(accepted=False,parameters={},free_variables=64)
        with patch('longitudinal_joint.solve',return_value=({},rejected)) as mocked:
            ctrl.predict({'A':False,'B':False})
        seed=mocked.call_args.kwargs['warm']
        np.testing.assert_array_equal(seed[:N],reference[1:N]+[0.])
        np.testing.assert_array_equal(seed[N:],np.zeros(N))
        before=copy.deepcopy(ctrl.reference_previous)
        ctrl.remember_reference(dict(accepted=False))
        self.assertEqual(ctrl.reference_previous,before)

    def test_warm_seed_is_projected_without_changing_qp(self):
        ctrl=Controller(SCENE,self.path);g={'A':False,'B':False}
        before=build_qp(ctrl.agents,g,SCENE)
        _,record=solve(ctrl.agents,g,SCENE,warm=np.full(64,10.))
        after=build_qp(ctrl.agents,g,SCENE)
        for original,current in zip(before[:7],after[:7]):
            np.testing.assert_array_equal(original,current)
        self.assertEqual(before[7:],after[7:])
        self.assertTrue(record['warm_start_used'])
        limits=np.array(before[7])
        np.testing.assert_array_equal(record['warm_start_solution'],limits[:,1])
        with self.assertRaisesRegex(ValueError,'invalid reference warm-start'):
            solve(ctrl.agents,g,SCENE,warm=[float('nan')]*64)

    def test_wait_gate_holds_and_permission_changes_phase(self):
        ctrl=Controller(SCENE,self.path);b=ctrl.agents['B']
        b.index=3;b.z=np.array(self.path['B'][2]['end']);b.z[4]=0.;b.mode='WAIT';b.ready=True
        b.advance(False);self.assertEqual(b.mode,'WAIT')
        self.assertEqual(b.parameters(False)['cap'],0.)
        b.advance(True);self.assertEqual(b.mode,'STEER');self.assertTrue(b.steering)


if __name__=='__main__':unittest.main()
