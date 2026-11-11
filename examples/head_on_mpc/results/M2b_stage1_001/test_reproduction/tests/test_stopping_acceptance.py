"""Hand-computed parking, sampled sign-policy, and forbidden-reset examples."""
import copy
import json
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from verify_entry_bicycle import check_series
from longitudinal_joint import Agent, routes

SCENE=json.loads((Path(__file__).resolve().parents[1]/'config/entry_bicycle.json').read_text())


def series(v0, accelerations):
    # Closed-form constant-acceleration data, independent of both RK4 implementations.
    x=0.;v=v0;dt=SCENE['sample_s']
    rows=[dict(time_s=0.,x=x,y=0.,psi=0.,v=v,delta=0.,a=0.,omega=0.)]
    for i,a in enumerate(accelerations,1):
        x+=dt*v+.5*dt*dt*a;v+=dt*a
        rows.append(dict(time_s=i*dt,x=x,y=0.,psi=0.,v=v,delta=0.,a=a,omega=0.))
    return rows


def cases():
    specs=[
        ('forward_stop_at_sample',.02,[-.5,-.5],True,False,'Valid braking to rest at a sample.'),
        ('reverse_stop_at_sample',-.02,[.5,.5],True,False,'Valid reverse braking to rest.'),
        ('rest_hold',0.,[0.,0.],True,False,'Rest without overwriting state.'),
        ('reversal_with_zero_sample',.01,[-.5,-.5],True,False,'Sampled rest then reversal; not a monotone parking certificate.'),
        ('smooth_unsampled_zero_crossing',.005,[-.5],False,True,'Dynamics and actuator limits valid; original sampled sign policy rejects.'),
        ('M0_tiny_overshoot',.001275969897234368,[-.06380350781514413],False,True,'Actual M0 overshoot despite speed magnitude below 1e-5.'),
        ('tiny_product_crossing',1e-6,[-1e-4],True,False,'Product threshold has a dead band; passing does not prove no reversal.'),
        ('moving_with_WAIT_label',.1,[0.],True,False,'Equation checker cannot authorize a phase transition from a label.'),
        ('actuator_limit_violation',.1,[2.],False,False,'Dynamics consistent but acceleration outside configured limits.'),
    ]
    output=[]
    for name,v0,a,passed,jump,meaning in specs:
        rows=series(v0,a)
        if name=='moving_with_WAIT_label':
            for row in rows:row['stage']='MUTEX_WAIT'
        result=check_series(rows,SCENE)
        output.append(dict(name=name,rows=rows,expected_passed=passed,expected_sign_jump=jump,meaning=meaning,result=result))
    for name,key,delta in [('velocity_reset','v',-.02),('pose_reset','x',.01)]:
        rows=series(.02,[0.]);rows[-1][key]+=delta
        output.append(dict(name=name,rows=rows,expected_passed=False,expected_sign_jump=False,
            meaning='Forbidden state overwrite; component equations must reject.',result=check_series(rows,SCENE)))
    return output


class StoppingAcceptanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.path=routes(SCENE)

    def test_hand_computed_accept_and_reject_examples(self):
        for case in cases():
            with self.subTest(case=case['name']):
                self.assertEqual(case['result']['passed'],case['expected_passed'])
                self.assertEqual(case['result']['gear_sign_jump'],case['expected_sign_jump'])

    def test_smooth_crossing_rejected_by_policy_not_dynamics(self):
        case=next(c for c in cases() if c['name']=='smooth_unsampled_zero_crossing')
        self.assertTrue(all(v<=SCENE['tolerance'] for v in case['result']['residuals'].values()))
        self.assertTrue(case['result']['gear_sign_jump'])

    def test_position_and_actual_rest_are_both_required_for_phase_change(self):
        for s_offset,speed in [(-.1,0.),(0.,.1),(0.,-1e-7)]:
            with self.subTest(s_offset=s_offset,speed=speed):
                agent=Agent('B',self.path['B'],SCENE)
                agent.s=agent.segment['length']+s_offset;agent.z=np.array(agent.segment['end']);agent.z[3]=speed
                before=agent.z.copy();agent.advance(False)
                self.assertEqual(agent.mode,'MOVE');np.testing.assert_array_equal(agent.z,before)
        agent=Agent('B',self.path['B'],SCENE)
        agent.s=agent.segment['length'];agent.z=np.array(agent.segment['end']);before=agent.z.copy()
        agent.advance(False);self.assertEqual(agent.mode,'STEER');np.testing.assert_array_equal(agent.z,before)


if __name__=='__main__':unittest.main()
