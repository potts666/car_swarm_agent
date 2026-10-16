import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from head_on_core import CFG, CASES, integrate, matrices, prediction, receive, run_case, solve
from recording import Recording
from verify_bundle import audit_case


class HeadOn(unittest.TestCase):
    def test_exact_model_against_stepwise_integration(self):
        controls = np.linspace(-.2,.3,CFG.horizon)
        state = np.array([3.,.8])
        times = np.arange(1,CFG.horizon+1)*CFG.dt
        pmat,vmat = matrices(times)
        expected = []
        for acceleration in controls:
            state = integrate(state,acceleration,[CFG.dt])[0]
            expected.append(state.copy())
        np.testing.assert_allclose(np.column_stack((3+times*.8+pmat@controls,.8+vmat@controls)),expected,atol=1e-12)

    def test_prediction_time_coverage(self):
        packet = prediction([0.,1.],np.zeros(CFG.horizon),[0.,0.,100.,6.],100.)
        end = 100+(CFG.horizon+1)*CFG.dt
        np.testing.assert_allclose(receive(packet,[100.5,end])[:,0],[.5,end-100])
        for times in ([99.9],[end+.1]):
            with self.assertRaises(ValueError):
                receive(packet,times)

    def test_neighbour_changes_first_control(self):
        peer = np.tile([6.,0.,0.],(CFG.horizon*CFG.substeps,1))
        empty,r0 = solve([0.,1.],0.,np.zeros(CFG.horizon),100.,18.)
        close,r1 = solve([0.,1.],0.,np.zeros(CFG.horizon),100.,18.,front=peer)
        self.assertTrue(r0['accepted'],r0)
        self.assertTrue(r1['accepted'],r1)
        self.assertLess(close[0],empty[0]-.05)

    def test_all_strategies_raw_motion_and_replay_endpoints(self):
        scene = json.loads((ROOT/'config/scenarios.json').read_text())
        with tempfile.TemporaryDirectory() as folder:
            for name in CASES:
                directory = Path(folder)/name
                summary = run_case(name,directory,scene)
                self.assertTrue(summary['expectation_met'],summary)
                self.assertTrue(audit_case(directory,video=False)['passed'])
                recording = Recording(directory)
                for car in ('A','B'):
                    final = recording.sample(car,recording.end+100)
                    np.testing.assert_allclose([final['x'],final['vx']],summary['final_states_x_v'][car])
                    self.assertTrue(recording.plan(car,recording.end))
                if name=='head_on_inline_stop':
                    self.assertEqual(summary['status'],'blocked')
                    self.assertAlmostEqual(recording.sample('B',recording.end)['y'],0.)


if __name__=='__main__':
    unittest.main()
