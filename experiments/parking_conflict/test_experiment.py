import tempfile
import unittest
import json
from pathlib import Path
import numpy as np
import experiment as e

class ParkingExperiment(unittest.TestCase):
    def setUp(self):
        self.c=json.loads((e.ROOT/'scenario.json').read_text())

    def test_wait_witness_and_identical_baseline_limit_are_distinguished(self):
        a,b,_=e.primitives(self.c)
        with tempfile.TemporaryDirectory() as d:
            r,_=e.validate(self.c,[a,np.vstack((np.zeros((len(a),2)),b))],self.c['goals'],'wait',Path(d))
        self.assertTrue(r['physical_envelope_pass'],r)
        self.assertFalse(r['unchanged_baseline_envelope_pass'])
        self.assertFalse(r['continuous_time_certified'])
        self.assertGreater(r['completion_time_s'][1],r['completion_time_s'][0])

    def test_conflict_requires_full_body_not_point(self):
        a,b,_=e.primitives(self.c)
        with tempfile.TemporaryDirectory() as d:
            r,_=e.validate(self.c,[a,b],self.c['goals'],'conflict',Path(d))
        self.assertFalse(r['physical_envelope_pass'])
        self.assertGreater(r['violations']['pair_gap'],0)

    def test_bypass_failure_is_not_relabeled_as_success(self):
        a,_,b=e.primitives(self.c)
        with tempfile.TemporaryDirectory() as d:
            r,_=e.validate(self.c,[a,b],self.c['goals'],'bypass',Path(d))
        self.assertFalse(r['physical_envelope_pass'])
        self.assertGreater(r['violations']['1_static'],0)

    def test_heading_required_for_arrival(self):
        s=np.r_[self.c['goals'][0],0,0]
        self.assertTrue(e.arrived(s,self.c['goals'][0],self.c))
        s[2]=np.pi
        self.assertFalse(e.arrived(s,self.c['goals'][0],self.c))

if __name__=='__main__': unittest.main()
