"""Small checks for the new model and admission/release failure modes."""
import copy
import json
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from bicycle_core import (arbitrate,audit,bay_entry,completed,corners,drive,gaps,
                          rollout,shift,shift_advance,steer,clipped_area,validate_scene)
from run_entry_bicycle import detector_for

SCENE=json.loads((Path(__file__).resolve().parents[1]/'config/entry_bicycle.json').read_text())


class BicycleTests(unittest.TestCase):
    def test_no_motion_from_steering_at_rest(self):
        z=[0.,0.,0.,0.,0.]
        result=rollout(z,steer(0.,.5,SCENE,'STEER'),SCENE)
        np.testing.assert_allclose(result[:,:4],np.zeros((len(result),4)),atol=1e-12)
        self.assertAlmostEqual(result[-1,4],.5)

    def test_arc_geometry_and_return_continuity(self):
        commands=shift(SCENE);z=[-30.,0.,0.,0.,0.]
        entry=rollout(z,commands,SCENE)
        np.testing.assert_allclose(entry[-1],[-30+shift_advance(SCENE),3.,0.,0.,0.],atol=1e-8)
        result=rollout(entry[-1],shift(SCENE,returning=True),SCENE)
        np.testing.assert_allclose(result[0],entry[-1],atol=0.)
        self.assertAlmostEqual(result[-1,1],0.,places=8)
        self.assertTrue(audit({'A':entry},SCENE)['passed'])
        self.assertLessEqual(np.abs(entry[:,4]).max(),SCENE['vehicle']['delta_max'])

    def test_reverse_crosses_zero_and_curves_with_heading(self):
        controls=drive(3.,SCENE,'FORWARD')+drive(-4.,SCENE,'REVERSE')
        states=rollout([0.,0.,0.,0.,0.],controls,SCENE)
        self.assertFalse(np.any(states[:-1,3]*states[1:,3]<-1e-10))
        np.testing.assert_allclose(states[-1],[-1.,0.,0.,0.,0.],atol=1e-9)
        turning=steer(0.,.4,SCENE,'STEER')+drive(-2.,SCENE,'REVERSE')
        reverse=rollout([0.,0.,0.,0.,0.],turning,SCENE)
        self.assertLess(reverse[-1,2],0.)
        self.assertGreater(reverse[-1,1],0.)

    def test_invalid_scene_and_illegal_actuator_rejected(self):
        validate_scene(SCENE)
        bad=copy.deepcopy(SCENE);bad['vehicle']['L']=4.5
        with self.assertRaises(ValueError):
            validate_scene(bad)
        with self.assertRaises(ValueError):
            rollout([0.,0.,0.,0.,0.],[(0.,1.01,'INVALID')],SCENE)

    def test_unauthorized_entry_uses_front_and_release_uses_tail(self):
        protection=[-18.,18.,-1.4,1.4]
        before=corners([[-22.,0.,0.,0.,0.]],SCENE['vehicle'])[0]
        invaded=corners([[-21.,0.,0.,0.,0.]],SCENE['vehicle'])[0]
        self.assertEqual(clipped_area(before,protection),0.)
        self.assertGreater(clipped_area(invaded,protection),0.)
        self.assertLess(26.6-SCENE['vehicle']['rear'],26.5)
        self.assertGreater(27.6-SCENE['vehicle']['rear'],26.5)

    def test_candidate_logging_keeps_checkpoint_time_and_initial_state(self):
        from run_entry_bicycle import rows_for
        initial=np.array([0.,0.,0.,0.,0.]);motion=rollout(initial,drive(1.,SCENE,'TEST'),SCENE)
        rows=rows_for({'A':motion},{'A':drive(1.,SCENE,'TEST')},126.4,SCENE,include_initial=True)
        self.assertEqual(rows[0]['time_s'],126.4)
        self.assertEqual(len(rows),len(motion))
        self.assertEqual(rows[0]['x'],0.)

    def test_replay_detects_pose_reset(self):
        from verify_entry_bicycle import check_series
        z=rollout([0.,0.,0.,0.,0.],drive(1.,SCENE,'DRIVE'),SCENE)
        # Actual logged command convention: each row carries the preceding interval input.
        commands=drive(1.,SCENE,'DRIVE');n=round(SCENE['h']/SCENE['sample_s']);rows=[]
        for i,q in enumerate(z):
            a,w,_=commands[max(0,(i-1)//n)]
            rows.append(dict(time_s=100+i*SCENE['sample_s'],**dict(zip(('x','y','psi','v','delta'),q)),a=a,omega=w))
        self.assertTrue(check_series(rows,SCENE)['passed'])
        rows[len(rows)//2]['y']+=.1
        self.assertFalse(check_series(rows,SCENE)['passed'])

    def test_atomic_simultaneous_requests(self):
        owner,grants=arbitrate(None,[(2.,'B'),(2.,'A')])
        self.assertEqual(owner,'A');self.assertEqual(sum(grants.values()),1)
        self.assertEqual(arbitrate('B',[(3.,'A')]),('B',{'A':False}))

    def test_full_body_not_rear_axle(self):
        body=corners([[11.,0.,np.pi,0.,0.]],SCENE['vehicle'])[0]
        self.assertLess(body[:,0].min(),10.)
        a=corners([[0.,0.,.3,0.,0.]],SCENE['vehicle'])
        self.assertAlmostEqual(np.linalg.norm(a[0,1]-a[0,0]),4.5)
        self.assertEqual(gaps(a,a)[0],0.)

    def test_park_does_not_block_goal_or_mutate_checkpoint(self):
        checkpoint={'A':np.array([-30.,0.,0.,0.,0.]),'B':np.array([30.,0.,np.pi,0.,0.])}
        frozen=copy.deepcopy(checkpoint)
        commands=bay_entry(checkpoint['B'],'B',SCENE)
        parked=rollout(checkpoint['B'],commands,SCENE)[-1]
        pass_a=rollout([-20.,0.,0.,0.,0.],drive(64.,SCENE,'PASS'),SCENE)
        hold=np.tile(parked,(len(pass_a),1))
        self.assertTrue(audit({'A':pass_a,'B':hold},SCENE)['passed'])
        for c in checkpoint:
            np.testing.assert_array_equal(checkpoint[c],frozen[c])
        self.assertTrue(completed(np.array(SCENE['goals']['B']),SCENE['goals']['B'],SCENE))

    def test_detector_rejects_stale_failure_and_intentional_wait(self):
        detector=detector_for(SCENE);states={c:np.array([0.,0.]) for c in ('A','B')}
        p={c:np.zeros((32,3)) for c in states}
        for now in np.arange(100.,107.,.2):
            self.assertFalse(detector.observe(now,states,p,True,mode='WAIT')['triggered'])
        self.assertFalse(detector.observe(107.,states,p,False)['triggered'])
        self.assertFalse(detector.observe(107.2,states,p,True,blocked={'A':True,'B':True})['triggered'])


if __name__=='__main__':
    unittest.main()
