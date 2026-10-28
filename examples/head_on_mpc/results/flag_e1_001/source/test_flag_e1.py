import json
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from ros_flag import Channel
from flag_constraints import construct, accepts
from bicycle_core import bay_entry, rollout, return_plan

SCENE=json.loads((Path(__file__).resolve().parents[1]/'config/entry_bicycle.json').read_text())


class FlagTests(unittest.TestCase):
    def test_time_id_and_ready(self):
        c=Channel();c.request('B',2);c.request('A',2)
        self.assertEqual(c.grant(),'A')
        c.release('A',dict(passed=True,both_future_goals=True))
        self.assertIsNone(c.grant())
        c.ready.add('B');self.assertEqual(c.grant(),'B')
        self.assertEqual(c.grant(),'B')
        other=Channel();other.request('B',1);other.request('A',2)
        self.assertEqual(other.grant(),'B')

    def test_reject_invalid_release(self):
        c=Channel();c.request('A',0);c.grant()
        for car,checks in [('B',dict(passed=True,both_future_goals=True)),('A',dict(passed=False)),('A',dict(passed=True))]:
            with self.assertRaises(ValueError):c.release(car,checks)
        self.assertEqual(c.owner,'A')

    def test_wait_and_pass_constraints(self):
        z=np.array([30.,0.,np.pi,0.,0.]);entry=bay_entry(z,'B',SCENE)
        parked=rollout(z,entry,SCENE)[-1]
        waiting=construct('B',0,parked,[],'MUTEX_WAIT',SCENE)
        self.assertEqual(waiting['s_max_m'],0.)
        commands=return_plan(parked,'B',SCENE)
        predicted=rollout(parked,commands,SCENE)
        self.assertFalse(accepts(waiting,predicted,SCENE))
        allowed=construct('B',1,parked,commands,'RETURNING',SCENE)
        self.assertTrue(accepts(allowed,predicted,SCENE))
        self.assertGreater(allowed['s_max_m'],0.)
        with self.assertRaises(ValueError):construct('B',2,parked,[],'WAIT',SCENE)


if __name__=='__main__':unittest.main()
