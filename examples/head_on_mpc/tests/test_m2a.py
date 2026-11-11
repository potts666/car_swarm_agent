import copy
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from bicycle_core import rollout
from longitudinal_joint import Agent,routes
from m2a_protocol import Channel,Barrier,public_report,make_plan,check_plan,acknowledge,plan_id,cleared
from run_m2a import Endpoint
SCENE=json.loads((Path(__file__).resolve().parents[1]/'config/entry_bicycle.json').read_text())


class M2aTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.paths=routes(SCENE)
    def plan(self,report,kind='final'):
        commands=[(0.,0.,'TEST_HOLD')]*SCENE['N']
        return make_plan(report,commands,rollout(np.array(report['z']),commands,SCENE),kind,1,'test')
    def prepared(self):
        channel=Channel(SCENE,self.paths['B'][2]['end']);channel.request('A',0);channel.grant_initial(0);channel.request('B',1)
        a=Agent('A',self.paths['A'],SCENE);a.z[0]=30.
        b=Agent('B',self.paths['B'],SCENE);b.index=3;b.mode='WAIT';b.ready=True;b.z=np.array(self.paths['B'][2]['end']);b.z[4]=0.
        reports={c:public_report(x,channel.context(2)) for c,x in [('A',a),('B',b)]}
        return channel,reports
    def test_nonowner_and_old_authorization_rejected(self):
        channel,reports=self.prepared()
        for car,version in [('B',1),('A',0)]:
            with self.subTest(car=car,version=version),self.assertRaises(ValueError):channel.propose_release(car,version,reports,2)
        self.assertEqual(channel.owner,'A');self.assertEqual(channel.version,1)
    def test_predicted_clear_does_not_replace_actual_clear(self):
        channel,reports=self.prepared();reports['A']['z'][0]=0.;reports['A']['predicted_clear']=True
        with self.assertRaisesRegex(ValueError,'actual rotated body'):channel.propose_release('A',1,reports,2)
    def test_rotated_support_and_wait_label(self):
        channel,reports=self.prepared();reports['A']['z'][0]=27.6;reports['A']['z'][2]=np.pi
        self.assertGreater(reports['A']['z'][0]-SCENE['vehicle']['rear'],26.5)
        self.assertFalse(cleared('A',reports['A']['z'],SCENE))
        with self.assertRaises(ValueError):channel.propose_release('A',1,reports,2)
        channel,reports=self.prepared();reports['B']['z'][3]=.1
        with self.assertRaisesRegex(ValueError,'physically ready'):channel.propose_release('A',1,reports,2)
    def test_WAIT_label_at_wrong_position_is_not_ready(self):
        channel,reports=self.prepared();reports['B']['z'][0]+=1.
        with self.assertRaisesRegex(ValueError,'physically ready'):channel.propose_release('A',1,reports,2)
        self.assertEqual(channel.owner,'A')

    def test_valid_handoff_and_duplicate_release(self):
        channel,reports=self.prepared();ctx=channel.propose_release('A',1,reports,2)
        self.assertEqual(channel.owner,'A')
        for r in reports.values():r['context']=ctx
        plans={c:self.plan(r) for c,r in reports.items()};barrier=Barrier(ctx)
        for p in plans.values():barrier.publish(p)
        for c in ('A','B'):
            other='B' if c=='A' else 'A';barrier.accept(acknowledge(c,plans[c],plans[other],reports[c],reports[other],ctx,SCENE))
        channel.commit_release(barrier);self.assertEqual(channel.owner,'B');self.assertEqual(channel.version,2)
        with self.assertRaises(ValueError):channel.propose_release('A',1,reports,2)
    def test_missing_ack_cancel_keeps_old_authorization(self):
        channel,reports=self.prepared();ctx=channel.propose_release('A',1,reports,2)
        with self.assertRaisesRegex(ValueError,'missing compatible'):channel.commit_release(Barrier(ctx))
        channel.cancel(2,'missing acknowledgement')
        self.assertEqual(channel.owner,'A');self.assertEqual(channel.version,1);self.assertIsNone(channel.proposal)

    def test_stale_ack_and_repair_requires_new_prediction(self):
        channel,reports=self.prepared();ctx=channel.context(2);plans={c:self.plan(r) for c,r in reports.items()};barrier=Barrier(ctx)
        for p in plans.values():barrier.publish(p)
        ack=acknowledge('A',plans['A'],plans['B'],reports['A'],reports['B'],ctx,SCENE)
        bad=copy.deepcopy(ack);bad['context']['cycle']+=1
        with self.assertRaises(ValueError):barrier.accept(bad)
        modified=copy.deepcopy(plans['B']);modified['commands'][0][0]=.1
        with self.assertRaisesRegex(ValueError,'new ID'):check_plan(modified,reports['B'],ctx,SCENE)
        modified['id']=plan_id(modified)
        with self.assertRaisesRegex(ValueError,'inputs/prediction'):check_plan(modified,reports['B'],ctx,SCENE)
        updated=copy.deepcopy(plans['B']);updated['revision']=2;updated['id']=plan_id(updated);barrier.publish(updated)
        with self.assertRaisesRegex(ValueError,'stale acknowledgement'):barrier.accept(ack)
    def test_no_commit_before_conditional_authorization(self):
        channel,reports=self.prepared();ctx=channel.propose_release('A',1,reports,2)
        endpoint=Endpoint('B',self.paths['B'],SCENE);endpoint.begin(ctx);endpoint.plan=self.plan(endpoint.report)
        endpoint.peer_report=reports['A'];endpoint.peer_report['context']=ctx;endpoint.peer=self.plan(endpoint.peer_report)
        packet=dict(token=dict(context=ctx,plans={'A':endpoint.peer['id'],'B':endpoint.plan['id']}),effective=channel.context(2),activated_proposal_id=None)
        with self.assertRaisesRegex(ValueError,'effective authorization'):endpoint.check_commit(packet)
    def test_initialization_and_candidate_rejection(self):
        channel=Channel(SCENE,self.paths['B'][2]['end']);ctx=channel.context(0)
        a=Endpoint('A',self.paths['A'],SCENE);b=Endpoint('B',self.paths['B'],SCENE);a.agent.z[0]=-30.;b.agent.z[0]=-28.
        a.begin(ctx);b.begin(ctx);a.observe_report(b.report);b.observe_report(a.report)
        sa=a.seed();sb=b.seed();a.observe_plan(sb)
        with self.assertRaisesRegex(ValueError,'no compatible'):a.validate_seed_pair(sa)
        a.begin(ctx)
        with self.assertRaisesRegex(ValueError,'missing compatible'):a.local_candidate()
        a.observe_report(b.report);a.observe_plan(sb)
        with patch.object(a.optimizer,'predict',return_value=(None,None,dict(accepted=False,message='test local reject'))):
            with self.assertRaisesRegex(ValueError,'test local reject'):a.local_candidate()
        self.assertIsNone(a.optimizer.previous);self.assertEqual(a.last_commit,-1)
    def test_redundant_rows_have_valid_certificates_and_needed_rows_remain(self):
        channel=Channel(SCENE,self.paths['B'][2]['end']);ctx=channel.context(0)
        for peer_x,need_active in [(46.,False),(-25.,True)]:
            with self.subTest(peer_x=peer_x):
                a=Endpoint('A',self.paths['A'],SCENE);b=Endpoint('B',self.paths['B'],SCENE);b.agent.z[0]=peer_x
                a.begin(ctx);b.begin(ctx);a.observe_report(b.report);a.observe_plan(b.seed());a.local_candidate()
                rows=a.record['subproblems'][-1]['separating_rows']
                self.assertEqual(len(rows),32)
                self.assertEqual(any(not r['redundant_via_state_box'] for r in rows),need_active)
                for r in rows:
                    if r['redundant_via_state_box']:
                        self.assertTrue(r['state_box_high']<=r['bound'] if r['side']=='upper' else r['state_box_low']>=r['bound'])

    def test_local_qps_have_only_own_variables(self):
        channel=Channel(SCENE,self.paths['B'][2]['end']);ctx=channel.context(0);a=Endpoint('A',self.paths['A'],SCENE);b=Endpoint('B',self.paths['B'],SCENE)
        a.begin(ctx);b.begin(ctx);a.observe_report(b.report);a.observe_plan(b.seed())
        with patch('longitudinal_joint.solve',side_effect=AssertionError('joint called')),patch('linearized_joint.subproblem',side_effect=AssertionError('joint called')):
            plan=a.local_candidate()
        self.assertEqual(a.record['reference_record']['variables'],32)
        self.assertTrue(all(q['variables']==64 for q in a.record['subproblems']))
        self.assertEqual(set(a.scene['initial']),{'A'});self.assertEqual(set(a.scene['goals']),{'A'})
        self.assertNotIn('solution',plan);self.assertNotIn('reference_record',plan);self.assertFalse(hasattr(a,'agents'))


if __name__=='__main__':unittest.main()
