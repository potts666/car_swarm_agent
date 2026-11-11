import gzip
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

HERE=Path(__file__).resolve().parent
RESULTS=HERE.parent
version=json.loads((RESULTS/'M2b_stage1_001/full_L_FIFO/r1/config.json').read_text())['source_version']
sys.path.insert(0,str(RESULTS/'M2b_stage1_001/sources'/version))
from analysis_io import digest,restored_plans,physical,CycleStream,run_paths,logical_motion
from metrics_core import aligned_difference,wrapped,stopping_distance,cumulative_progress


class AnalysisTests(unittest.TestCase):
    def test_common_absolute_time_has_311_points(self):
        old=np.zeros((321,5));old[:,0]=np.arange(321)*.02
        new=np.zeros((321,5));new[:,0]=.2+np.arange(321)*.02
        result,values=aligned_difference(old,new,.2,.02)
        self.assertEqual(len(values['position_m']),311)
        self.assertLess(result['maximum']['position_m'],1e-14)

    def test_yaw_wrap_does_not_create_two_pi_error(self):
        self.assertAlmostEqual(float(wrapped(np.pi-(-np.pi))),0.,places=12)
        self.assertAlmostEqual(float(wrapped(2*np.pi+.1)),.1)

    def test_compressed_reference_restore_and_missing_body(self):
        with tempfile.TemporaryDirectory() as t:
            path=Path(t)/'plans.jsonl.gz'
            first=dict(car='A1',context={'cycle':0},kind='final',states=[[0]*5,[1]*5,[2]*5],commands=[[0,0,'X']]*2,shifted_from=None)
            first['id']=digest(first)
            seed=dict(car='A1',context={'cycle':1},kind='seed',states=[[1]*5,[2]*5,[3]*5],commands=[[0,0,'X']]*2,shifted_from=first['id'])
            seed['id']=digest(seed);stored={**seed,'states':{'$shift_states':first['id'],'skip':1,'tail':[[3]*5]}}
            with gzip.open(path,'wt') as f:f.write(json.dumps(first)+'\n'+json.dumps(stored)+'\n')
            self.assertEqual(list(restored_plans(Path(t)/'plans.jsonl'))[1],seed)
            with gzip.open(path,'wt') as f:f.write(json.dumps(stored)+'\n')
            with self.assertRaises(ValueError):list(restored_plans(Path(t)/'plans.jsonl'))

    def test_missing_and_ambiguous_logs_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            p=Path(t)/'x'
            with self.assertRaises(FileNotFoundError):physical(p)
            p.write_text('');p.with_name('x.gz').write_bytes(b'')
            with self.assertRaises(ValueError):physical(p)

    def test_cycle_stream_order_and_progress_reset(self):
        stream=CycleStream(iter([{'cycle':0},{'cycle':1}]),lambda r:r['cycle'])
        self.assertEqual(stream.take(0),[{'cycle':0}])
        self.assertEqual(stream.take(1),[{'cycle':1}])
        route={'segments':[{'offset':0.,'length':10.},{'offset':10.,'length':5.}]}
        self.assertEqual(cumulative_progress({'index':0,'progress_m':10.},route),cumulative_progress({'index':1,'progress_m':0.},route))
        self.assertEqual(cumulative_progress({'index':2,'progress_m':0.},route),15.)
        bad=CycleStream(iter([{'cycle':0}]),lambda r:r['cycle'])
        with self.assertRaises(ValueError):bad.take(1)

    def test_discrete_stop_bound_and_separate_run_classes(self):
        self.assertAlmostEqual(stopping_distance(.1,.2,1.),.01)
        self.assertAlmostEqual(stopping_distance(1.,.2,1.),.5)
        paths=list(run_paths(Path('/fake')))
        self.assertEqual(sum(kind=='formal' for _,_,kind in paths),6)
        self.assertEqual(sum(kind=='artificial_rejection' for _,_,kind in paths),1)
        self.assertEqual(sum(kind=='bounded_prefix' for _,_,kind in paths),5)

    def test_repeats_group_on_motion_not_plan_id(self):
        with tempfile.TemporaryDirectory() as t:
            hashes=[]
            for name in ['first','repeat']:
                root=Path(t)/name;root.mkdir()
                row=dict(cycle=0,owner=None,permission_version=0,states={'A1':[[0]*5]},controls={'A1':[0,0,'X']},plans={'A1':name})
                with gzip.open(root/'actual.jsonl.gz','wt') as f:f.write(json.dumps(row)+'\n')
                hashes.append(logical_motion(root))
            self.assertEqual(hashes[0],hashes[1])


if __name__=='__main__':unittest.main()
