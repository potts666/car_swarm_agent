import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from merging_core import CFG, CASES, admission, integrate, matrices, prediction, receive, run_case, solve
from recording import Recording
from verify_bundle import audit_case


class Merging(unittest.TestCase):
    def test_exact_model_against_stepwise_integration(self):
        controls = np.linspace(-.1,.2,CFG.horizon)
        state = np.array([3.,.8])
        times = np.arange(1,CFG.horizon+1)*CFG.dt
        pmat,vmat = matrices(times)
        expected = []
        for acceleration in controls:
            state = integrate(state,acceleration,[CFG.dt])[0]
            expected.append(state.copy())
        np.testing.assert_allclose(np.column_stack((3+times*.8+pmat@controls,.8+vmat@controls)),expected,atol=1e-12)

    def test_prediction_time_coverage_and_tail(self):
        packet = prediction([0.,1.],np.zeros(CFG.horizon),[0.,0.,100.,6.],100.)
        end = 100+(CFG.horizon+1)*CFG.dt
        np.testing.assert_allclose(receive(packet,[100.5,end])[:,0],[.5,end-100])
        for times in ([99.9],[end+.1]):
            with self.assertRaises(ValueError):
                receive(packet,times)

    def test_virtual_front_changes_first_control(self):
        ticks = np.arange(1,CFG.horizon*CFG.substeps+1)*CFG.dt/CFG.substeps
        peer = np.column_stack((7.+ticks*.5,np.full(len(ticks),.5),np.zeros(len(ticks))))
        free,r0 = solve([0.,1.5],0.,np.zeros(CFG.horizon),100.,1.5)
        close,r1 = solve([0.,1.5],0.,np.zeros(CFG.horizon),100.,1.5,front=peer,following=True)
        self.assertTrue(r0['accepted'],r0)
        self.assertTrue(r1['accepted'],r1)
        self.assertLess(close[0],free[0]-.05)

    def test_admission_rejects_small_slot_and_future_closure(self):
        def packets(xs,vs):
            return {car:prediction([x,v],np.zeros(CFG.horizon),[0.,0.,100.,6.],100.)
                    for car,x,v in zip(('A','C','B'),xs,vs)}
        self.assertFalse(admission(packets([14.,7.,0.],[1.5]*3),100.)[0])
        self.assertTrue(admission(packets([20.,10.,0.],[1.5]*3),100.)[0])
        self.assertFalse(admission(packets([20.,10.,0.],[.5,2.,1.5]),100.)[0])

    def test_closed_loops_raw_admission_and_replay_endpoints(self):
        scene = json.loads((ROOT/'config/scenarios.json').read_text())
        with tempfile.TemporaryDirectory() as folder:
            for name in CASES:
                directory = Path(folder)/name
                summary = run_case(name,directory,scene)
                self.assertTrue(summary['expectation_met'],summary)
                self.assertTrue(audit_case(directory,video=False)['passed'])
                recording = Recording(directory)
                for car in ('A','B','C'):
                    final = recording.sample(car,recording.end+100)
                    np.testing.assert_allclose([final['x'],final['vx']],summary['final_states_x_v'][car])
                    self.assertTrue(recording.plan(car,recording.end))

    def test_prediction_horizon_must_cover_whole_merge(self):
        scene = copy.deepcopy(json.loads((ROOT/'config/scenarios.json').read_text()))
        scene['settings']['horizon'] = 4
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(ValueError):
                run_case('merge_cooperative',Path(folder)/'invalid',scene)


if __name__=='__main__':
    unittest.main()
