"""Supplementary negative checks; run after the online/audit driver finishes."""
import copy,json,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parent
VERSION=(ROOT/'current_source_version.txt').read_text().strip()
sys.path.insert(0,str(ROOT/'sources'/VERSION/'scripts'))
sys.path.append(str(ROOT/'supplemental_sources/scripts'))
from m2b_jacobi import Actor
from m2b_scene import build_scene,build_routes,CARS
from m2b_protocol import Channel,canonical
from m2b_round_wire import RoundWire
from m2b_candidates import validate_start
from m2a_logs import plan_id

class FaultChecks(unittest.TestCase):
 def setUp(self):
  self.s=build_scene();route=build_routes(self.s);self.ctx=Channel(self.s,'fifo-ready').context(0)
  self.actors={c:Actor(c,route[c],self.s) for c in CARS};reports={c:a.snapshot(self.ctx) for c,a in self.actors.items()}
  for c,a in self.actors.items():a.prepare(self.ctx,{p:r for p,r in reports.items() if p!=c})
  self.seeds={c:a.start(1)['plan'] for c,a in self.actors.items()}
 def test_wrong_parking_reference_rejected_with_valid_content_hash(self):
  a=self.actors['B1'];p=copy.deepcopy(self.seeds['B1']);p['applicability']['stop_index']+=1;p['id']=plan_id(p)
  with self.assertRaises(ValueError):validate_start(p,a.work,a.parameters,a.context,a.identity,a.scene)
 def test_final_before_own_compute_and_wrong_sender_rejected(self):
  a=self.actors['A1'];p=self.seeds['B1']
  with self.assertRaises(ValueError):a.deliver_finals([dict(sender='B1',receiver='A1',kind='final',payload=p)])
  with self.assertRaises(ValueError):a.deliver_inputs([dict(sender='A2',receiver='A1',kind='seed',payload=p)])
  self.assertEqual(a.last_cycle,-1)
 def test_input_copy_does_not_alias_sender(self):
  a=self.actors['A1'];p=copy.deepcopy(self.seeds['B1']);a.deliver_inputs([dict(sender='B1',receiver='A1',kind='seed',payload=p)])
  before=a.inputs['B1']['states'][0][0];p['states'][0][0]+=10;self.assertEqual(a.inputs['B1']['states'][0][0],before)
 def test_duplicate_misroute_and_stale_cycle_dds_envelope(self):
  w=RoundWire.__new__(RoundWire);w.cycle=0;w.seen=set();w.received={};w.log=[]
  class Msg:
   def __init__(self,p):self.data=canonical(p)
  e=dict(sender='A1',receiver='A2',kind='snapshot',cycle=0,sequence=1,payload={})
  w.receive('A2',Msg(e))
  with self.assertRaises(ValueError):w.receive('A2',Msg(e))
  with self.assertRaises(ValueError):w.receive('B1',Msg({**e,'sequence':2}))
  with self.assertRaises(ValueError):w.receive('A2',Msg({**e,'sequence':3,'cycle':-1}))
 def test_stale_permission_in_prepare_leaves_committed_state(self):
  a=self.actors['A1'];reports={c:b.snapshot(self.ctx) for c,b in self.actors.items() if c!='A1'}
  reports['B1']['context']['permission_version']=-1
  with self.assertRaises(ValueError):a.prepare(self.ctx,reports)
  self.assertEqual(a.agent.z.tolist(),self.s['initial']['A1']);self.assertEqual(a.optimizer.previous,{})

if __name__=='__main__':unittest.main()
