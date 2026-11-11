import copy
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from m2b_scene import build_scene,CARS
from m2b_protocol import Channel,Barrier,select_next,ready_report,cleared,context_plan

class ProtocolTests(unittest.TestCase):
 def test_rules(self):
  requests={'A1':5,'B1':6};self.assertEqual(select_next(['A1','B1'],requests,'fifo-ready',1),'A1');self.assertEqual(select_next(['A1','B1'],requests,'alternating-ready',1),'B1')
  for policy in ('fifo-ready','alternating-ready'):
   self.assertEqual(select_next(['A1'],requests,policy,1),'A1');self.assertIsNone(select_next([],requests,policy,1))
 def reports(self,ch,cycle=10):
  s=ch.scene;ctx=ch.context(cycle);r={}
  for c in CARS:
   h=s['queue_heads'][c[0]];r[c]=dict(car=c,z=[*h,0. if c[0]=='A' else 3.141592653589793,0.,0.],mode='READY',steering_complete=True,context=ctx)
  return r
 def test_no_overtake_preserve_request_and_cancel_history(self):
  ch=Channel(build_scene(),'alternating-ready');ch.request('A2',1);ch.request('A1',5);ch.request('A1',9);r=self.reports(ch);r['A1']['mode']='QUEUE_WAIT'
  self.assertNotIn('A2',ch.eligible(r));self.assertEqual(ch.requests['A1'],5)
  r['A1']['mode']='READY';p=ch.prepare(None,0,r,10);self.assertIsNone(ch.owner);ch.cancel(10,'test');self.assertIsNone(ch.last_direction);self.assertEqual(ch.version,0)
 def test_release_auth_body_and_atomic_ack(self):
  ch=Channel(build_scene(),'fifo-ready');ch.owner='A1';ch.version=1;ch.served.add('A1');r=self.reports(ch);r['A1']['z'][0]=30.
  for reporter,version in [('B1',1),('A1',0)]:
   with self.assertRaises(ValueError):ch.prepare(reporter,version,r,10)
  r['A1']['z'][0]=27.6;r['A1']['z'][2]=3.141592653589793
  self.assertFalse(cleared('A1',r['A1']['z'],ch.scene))
  with self.assertRaises(ValueError):ch.prepare('A1',1,r,10)
  r['A1']['z'][0]=30.;r['A1']['z'][2]=0.;ctx=ch.prepare('A1',1,r,10);self.assertIsNone(ctx['owner'])
  b=Barrier(ctx)
  for c in CARS:b.publish(dict(car=c,id=c,kind='final',context=ctx))
  with self.assertRaises(ValueError):ch.commit(b)
  for c in CARS:b.accept(dict(car=c,context=ctx,plans=b.plans.copy(),accepted=True))
  ch.commit(b);self.assertIsNone(ch.owner);self.assertEqual(ch.version,2)
  ch.request('B1',11);r=self.reports(ch,11);ctx=ch.prepare(None,2,r,11);self.assertEqual(ctx['owner'],'B1')
 def test_mixed_versions_invalidate_all_acks(self):
  ctx={'cycle':1};b=Barrier(ctx)
  for c in CARS:b.publish(dict(car=c,id=c,context=ctx,kind='final'))
  ack=dict(car='A1',context=ctx,plans=b.plans.copy(),accepted=True);b.accept(ack);b.publish(dict(car='B2',id='changed',context=ctx,kind='final'))
  self.assertFalse(b.acks)
  with self.assertRaises(ValueError):b.accept(ack)
  with self.assertRaises(ValueError):b.token()

if __name__=='__main__':unittest.main()
