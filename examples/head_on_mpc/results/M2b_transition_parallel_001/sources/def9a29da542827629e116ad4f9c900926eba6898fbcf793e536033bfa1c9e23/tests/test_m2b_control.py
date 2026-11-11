from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from m2b_scene import *
from m2b_control import Optimizer
from m2b_protocol import public_report,context_plan,check_plan,Channel
from run_m2b import Endpoint

class ControlTests(unittest.TestCase):
 def setUp(self):self.s=build_scene();self.routes=build_routes(self.s)
 def test_joint_and_local_variables_all_pairs_and_cache(self):
  a={c:Agent(c,self.routes[c],self.s) for c in CARS};ctx=Channel(self.s,'fifo-ready').context(0)
  for x in a.values():x.advance(False,{})
  optimizer=Optimizer();u,z,r=optimizer.predict(a,{c:False for c in CARS},{},self.s)
  self.assertTrue(r['accepted']);self.assertEqual(r['reference_record']['variables'],128);self.assertEqual(r['subproblems'][0]['variables'],256)
  self.assertEqual(len({tuple(sorted((x['own'],x['peer']))) for x in r['subproblems'][0]['separating_rows']}),6)
  peers={c:context_plan(public_report(a[c],ctx),[(0.,0.,'TEST')]*32,np.tile(a[c].z,(321,1)),'seed','test') for c in CARS if c!='A1'}
  local=Optimizer()
  with patch('m2b_control.predict_joint',side_effect=AssertionError('joint forbidden')):
   u,z,r=local.predict({'A1':a['A1']},{'A1':False},peers,self.s)
  self.assertTrue(r['accepted']);self.assertEqual(r['reference_record']['variables'],32);self.assertEqual(r['subproblems'][0]['variables'],64)
  local.remember({'A1':a['A1']},r);self.assertEqual(set(local.previous),{'A1'})
  a['A1'].mode='STEER';u,z,r=local.predict({'A1':a['A1']},{'A1':False},peers,self.s);self.assertFalse(r['reference_record']['warm_start_per_car']['A1'])
 def test_follower_cannot_merge_or_advance_from_earlier_request(self):
  a=Agent('B2',self.routes['B2'],self.s);a.advance(False,{'B1':dict(queue_arrived=False,left_queue=False)})
  self.assertEqual(a.mode,'WAIT_MERGE');a.mode='QUEUE_WAIT';a.advance(False,{'B1':dict(queue_arrived=True,left_queue=False)});self.assertEqual(a.mode,'QUEUE_WAIT')
  a.advance(False,{'B1':dict(queue_arrived=True,left_queue=True)});self.assertNotEqual(a.mode,'QUEUE_WAIT')
 def test_corrupt_repair_and_moving_initialization_rejected(self):
  e=Endpoint('A1',self.routes['A1'],self.s);ctx=Channel(self.s,'fifo-ready').context(0);e.begin(ctx,{})
  p=e.seed();p['commands'][0][0]=.1
  with self.assertRaises(ValueError):check_plan(p,e.report,ctx,self.s)
  e.work.z[3]=.1
  with self.assertRaises(ValueError):e.seed()
 def test_frozen_stop_prefix_avoids_next_cycle_numeric_infeasibility(self):
  import json
  from bicycle_core import rollout
  f=json.loads((Path(__file__).parent/'fixtures/m2b_stop_prefix.json').read_text());a=Agent('A1',self.routes['A1'],self.s);a.mode='MOVE';a.s=f['progress'];a.z=np.array(f['state']);a.previous=f['previous_input'];a.stop_steps=f['stop_index'];ctx=Channel(self.s,'fifo-ready').context(42)
  peers={}
  for c,z in f['peer_states'].items():
   b=Agent(c,self.routes[c],self.s);b.z=np.array(z);u=[(0.,0.,'TEST')]*32;peers[c]=context_plan(public_report(b,ctx),u,rollout(b.z,u,self.s),'seed','frozen_state_probe')
  opt=Optimizer();opt.previous['A1']=dict(index=0,mode='MOVE',solution=f['previous_reference']);u,z,r=opt.predict({'A1':a},{'A1':False},peers,self.s)
  self.assertTrue(r['accepted']);acc=u['A1'][0][0];next_s=a.s+.2*a.z[3]+.02*acc;next_v=a.z[3]+.2*acc
  self.assertLessEqual(next_s+.1*max(0.,next_v),13.+1e-7)
  self.assertGreater(f['failed_progress']+.1*f['failed_state'][3]-13.,1e-7)

 def test_conditional_commit_cannot_execute_before_activation(self):
  e=Endpoint('A1',self.routes['A1'],self.s);ctx=Channel(self.s,'fifo-ready').context(0);ctx.update(conditional=True,owner='A1',permission_version=1,proposal_id='proposal');e.begin(ctx,{})
  e.plan=e.seed();e.peers={c:dict(id=c) for c in CARS if c!='A1'};pair={**{c:c for c in CARS if c!='A1'},'A1':e.plan['id']}
  with self.assertRaises(ValueError):e.check_commit(dict(token=dict(context=ctx,plans=pair),effective={**ctx,'owner':None,'permission_version':0},activated_proposal_id=None))

if __name__=='__main__':unittest.main()
