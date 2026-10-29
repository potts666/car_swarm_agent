import copy
import json
from pathlib import Path
import sys
import unittest
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

    def test_wait_gate_holds_and_permission_changes_phase(self):
        ctrl=Controller(SCENE,self.path);b=ctrl.agents['B']
        b.index=3;b.z=np.array(self.path['B'][2]['end']);b.z[4]=0.;b.mode='WAIT';b.ready=True
        b.advance(False);self.assertEqual(b.mode,'WAIT')
        self.assertEqual(b.parameters(False)['cap'],0.)
        b.advance(True);self.assertEqual(b.mode,'STEER');self.assertTrue(b.steering)


if __name__=='__main__':unittest.main()
