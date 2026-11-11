"""Cache invalidation, independent ACK checks, and non-overlapping timing."""
import copy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import m2b_protocol as protocol
from m2b_scene import build_scene,CARS
from m2a_logs import plan_id
from m2b_timing import CycleTiming,instrument


class StageOneTests(unittest.TestCase):
    def setUp(self):
        self.scene=build_scene()
        self.context=protocol.Channel(self.scene,'fifo-ready').context(0)
        self.reports={c:dict(car=c,z=self.scene['initial'][c],context=self.context) for c in CARS}
        self.plans={c:protocol.context_plan(self.reports[c],[(0.,0.,'TEST')]*32,
                        np.tile(self.reports[c]['z'],(321,1)),'final','test') for c in CARS}

    def test_hit_and_returned_state_cannot_poison_cache(self):
        c=CARS[0];cache={}
        with patch.object(protocol,'rollout',wraps=protocol.rollout) as replay:
            z=protocol.check_plan(self.plans[c],self.reports[c],self.context,self.scene,cache)
            z[0,0]+=10
            fresh=protocol.check_plan(self.plans[c],self.reports[c],self.context,self.scene,cache)
            self.assertEqual(replay.call_count,1)
            np.testing.assert_array_equal(fresh,self.plans[c]['states'])

    def test_content_change_stale_state_and_repair_without_prediction_rejected(self):
        c=CARS[0];cache={};p=self.plans[c];r=self.reports[c]
        protocol.check_plan(p,r,self.context,self.scene,cache)
        bad=copy.deepcopy(p);bad['states'][1][0]+=.01
        with self.assertRaises(ValueError):protocol.check_plan(bad,r,self.context,self.scene,cache)
        bad=copy.deepcopy(p);bad['commands'][0][0]=.01;bad['id']=plan_id(bad)
        with self.assertRaises(ValueError):protocol.check_plan(bad,r,self.context,self.scene,cache)
        context={**self.context,'permission_version':1}
        with self.assertRaises(ValueError):protocol.check_plan(p,r,context,self.scene,cache)
        changed=copy.deepcopy(r);changed['z']=list(changed['z']);changed['z'][0]+=.01
        with self.assertRaises(ValueError):protocol.check_plan(p,changed,self.context,self.scene,cache)

    def test_scene_and_validator_changes_miss(self):
        c=CARS[0];cache={};p=self.plans[c];r=self.reports[c]
        with patch.object(protocol,'rollout',wraps=protocol.rollout) as replay:
            protocol.check_plan(p,r,self.context,self.scene,cache)
            changed=copy.deepcopy(self.scene);changed['tolerance']/=2
            protocol.check_plan(p,r,self.context,changed,cache)
            with patch.object(protocol,'VALIDATION_VERSION','changed'):
                protocol.check_plan(p,r,self.context,changed,cache)
            self.assertEqual(replay.call_count,3)

    def test_cached_ack_still_checks_four_vehicle_geometry_and_missing_versions(self):
        cache={}
        for c in CARS:protocol.check_plan(self.plans[c],self.reports[c],self.context,self.scene,cache)
        with patch.object(protocol,'audit',wraps=protocol.audit) as audit:
            protocol.acknowledge(CARS[0],self.plans,self.reports,self.context,self.scene,cache)
            self.assertEqual(audit.call_count,1)
            self.assertEqual(set(audit.call_args.args[0]),set(CARS))
        with self.assertRaises(ValueError):protocol.acknowledge(CARS[0],{},self.reports,self.context,self.scene,cache)
        bad=copy.deepcopy(self.plans);bad[CARS[0]]['kind']='seed';bad[CARS[0]]['id']=plan_id(bad[CARS[0]])
        with self.assertRaises(ValueError):protocol.acknowledge(CARS[0],bad,self.reports,self.context,self.scene,cache)

    def test_exclusive_timings_partition_root_and_aliases_restore(self):
        clock=CycleTiming()
        with clock.span('root'):
            with clock.span('child'):clock.set_phase('next')
        self.assertAlmostEqual(sum(r['exclusive_s'] for r in clock.rows.values()),clock.rows['root']['inclusive_s'])
        self.assertAlmostEqual(sum(sum(r.values()) for r in clock.phases.values()),clock.rows['root']['inclusive_s'])
        before=protocol.rollout
        with instrument():self.assertIsNot(protocol.rollout,before)
        self.assertIs(protocol.rollout,before)

    def test_cache_does_not_hide_collision_or_released_reentry(self):
        cache={}
        for c in CARS:protocol.check_plan(self.plans[c],self.reports[c],self.context,self.scene,cache)
        plans=copy.deepcopy(self.plans);reports=copy.deepcopy(self.reports)
        c=CARS[1];reports[c]['z']=list(reports[CARS[0]]['z'])
        plans[c]=protocol.context_plan(reports[c],[(0.,0.,'TEST')]*32,np.tile(reports[c]['z'],(321,1)),'final','collision')
        with self.assertRaises(ValueError):protocol.acknowledge(CARS[0],plans,reports,self.context,self.scene,cache)
        context={**self.context,'released':[CARS[0]]}
        report={**self.reports[CARS[0]],'context':context}
        plan=protocol.context_plan(report,[(0.,0.,'TEST')]*32,np.tile(report['z'],(321,1)),'final','released_reentry')
        with self.assertRaises(ValueError):protocol.check_plan(plan,report,context,self.scene,cache)

    def test_candidate_subtraction_partitions_grey_phase(self):
        timer=CycleTiming()
        with timer.span('cycle'):
            with timer.span('candidate_compute'):
                with timer.span('integrate'):pass
            timer.set_phase('after_last_final')
        before=timer.phases['before_last_final'];candidate=timer.candidate_phases['before_last_final']
        self.assertTrue(all(before[k]>=v for k,v in candidate.items()))
        self.assertGreater(sum(candidate.values()),0.)


if __name__=='__main__':unittest.main()
