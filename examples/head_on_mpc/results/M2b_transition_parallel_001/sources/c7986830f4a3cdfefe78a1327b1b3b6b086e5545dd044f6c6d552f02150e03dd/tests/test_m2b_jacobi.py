import copy
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from m2b_jacobi import Actor
from m2b_scene import CARS,build_scene,build_routes
from m2b_protocol import Channel
from m2b_candidates import validate_start
from m2a_logs import plan_id


class JacobiTests(unittest.TestCase):
    def setUp(self):
        self.scene=build_scene();self.routes=build_routes(self.scene);self.context=Channel(self.scene,'fifo-ready').context(0)
        self.actors={c:Actor(c,self.routes[c],self.scene) for c in CARS}
        reports={c:a.snapshot(self.context) for c,a in self.actors.items()}
        for c,a in self.actors.items():a.prepare(self.context,{p:r for p,r in reports.items() if p!=c})
        self.seeds={c:a.start(1)['plan'] for c,a in self.actors.items()}

    def deliver(self,c,seeds=None):
        seeds=seeds or self.seeds
        return [dict(sender=p,receiver=c,kind='seed',payload=r) for p,r in seeds.items() if p!=c]

    def test_initial_steering_is_actual_queue(self):
        a=self.actors['B1'];p=self.seeds['B1']
        np.testing.assert_array_equal(np.array(p['commands'])[:len(a.work.steering),1].astype(float),[u[1] for u in a.work.steering])
        self.assertTrue(a.start_record['validation']['partial_phase_reference_passed'])
        self.assertFalse(a.start_record['validation']['full_ocp_checked'])

    def test_reject_does_not_consume_queue_or_committed_cache(self):
        a=self.actors['B1'];before=copy.deepcopy(a.agent.__dict__);warm=copy.deepcopy(a.optimizer.previous)
        with self.assertRaises(ValueError):a.solve()
        a.abort();np.testing.assert_array_equal(a.agent.z,before['z']);self.assertEqual(a.agent.steering,before['steering']);self.assertEqual(a.optimizer.previous,warm)

    def test_missing_duplicate_and_old_round(self):
        a=self.actors['A1'];a.deliver_inputs(self.deliver('A1')[:1])
        with self.assertRaises(ValueError):a.solve()
        with self.assertRaises(ValueError):a.deliver_inputs(self.deliver('A1')[:1])
        b=self.actors['A2'];bad=copy.deepcopy(self.seeds);bad['A1']['context']['coordination_round']=0;bad['A1']['id']=plan_id(bad['A1'])
        with self.assertRaises(ValueError):b.deliver_inputs(self.deliver('A2',bad))

    def test_mode_reference_and_repaired_content_binding(self):
        a=self.actors['B1'];p=copy.deepcopy(self.seeds['B1']);p['applicability']['mode']='MOVE';p['id']=plan_id(p)
        with self.assertRaises(ValueError):validate_start(p,a.work,a.parameters,a.context,a.identity,a.scene)
        p=copy.deepcopy(self.seeds['B1']);p['commands'][0][1]+=.01;p['id']=plan_id(p)
        with self.assertRaises(ValueError):a._check_public(p)

    def test_sealed_input_and_final_commit_versions(self):
        for c,a in self.actors.items():a.deliver_inputs(self.deliver(c))
        results={c:a.solve() for c,a in self.actors.items()};self.assertEqual(len({r['input_set_id'] for r in results.values()}),1)
        for c,a in self.actors.items():
            a.deliver_finals([dict(sender=p,receiver=c,kind='final',payload=r['plan']) for p,r in results.items() if p!=c]);a.ack()
            packet=dict(token=dict(context=a.context,plans=a.acceptance['plans']),effective={k:v for k,v in a.context.items() if k!='coordination_round'},activated_proposal_id=None)
            bad=copy.deepcopy(packet);bad['token']['plans'][c]='wrong'
            with self.assertRaises(ValueError):a.stage_commit(bad)
            before=a.agent.z.copy();queue=copy.deepcopy(a.agent.steering);a.stage_commit(packet)
            np.testing.assert_array_equal(a.agent.z,before);self.assertEqual(a.agent.steering,queue);a.abort();np.testing.assert_array_equal(a.agent.z,before)


if __name__=='__main__':unittest.main()
