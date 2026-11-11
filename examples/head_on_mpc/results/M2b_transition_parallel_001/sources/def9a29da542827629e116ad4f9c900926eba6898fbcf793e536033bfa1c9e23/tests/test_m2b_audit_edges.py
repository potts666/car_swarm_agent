"""A repaired command with a fresh hash still requires its matching nonlinear states."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from m2b_scene import build_scene,build_routes,Agent
from m2b_protocol import Channel,public_report,context_plan,check_plan
from m2a_logs import plan_id
from bicycle_core import rollout

class AuditEdgeTests(unittest.TestCase):
 def test_rehashed_input_repair_without_reprediction_rejected(self):
  s=build_scene();a=Agent('A1',build_routes(s)['A1'],s);ctx=Channel(s,'fifo-ready').context(0);r=public_report(a,ctx);u=[(0.,0.,'TEST')]*32;p=context_plan(r,u,rollout(a.z,u,s),'final','test')
  p['commands'][0][0]=-1e-7;p['id']=plan_id(p)
  with self.assertRaisesRegex(ValueError,'prediction'):check_plan(p,r,ctx,s)

if __name__=='__main__':unittest.main()
