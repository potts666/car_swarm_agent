import tempfile
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from m2a_logs import encode
from m2b_recording import export,compact,plain,lines
from m2b_scene import *
from m2b_protocol import Channel,public_report,context_plan,canonical

class RecordingTests(unittest.TestCase):
 def test_fourcar_restore_and_missing_content(self):
  s=build_scene();routes=build_routes(s);ctx=Channel(s,'fifo-ready').context(0);plans=[];messages=[]
  for c in CARS:
   a=Agent(c,routes[c],s);p=context_plan(public_report(a,ctx),[(0.,0.,'TEST')]*32,np.tile(a.z,(321,1)),'final','test');plans.append(p)
   messages.append(dict(event='send',kind='final',payload=p,application_bytes=len(canonical(p).encode())))
  with tempfile.TemporaryDirectory() as d:
   root=Path(d);export(root,dict(plans=plans,messages=messages),[]);self.assertEqual(encode(lines(root/'messages.jsonl')),encode(messages));(root/'plans.jsonl.gz').unlink()
   with self.assertRaises(FileNotFoundError):lines(root/'messages.jsonl')
 def test_infinite_diagnostic_bounds_are_explicit_not_invalid_json(self):
  import json
  value=plain(dict(lower=np.array([-np.inf,1e-7]),upper=float('inf')))
  self.assertEqual(value['lower'],['-Infinity',1e-7]);json.dumps(value,allow_nan=False)
 def test_shifted_seed_exact_recovery_and_corrupt_tail_rejected(self):
  import json,gzip
  from bicycle_core import rollout
  from m2a_logs import write_log
  from m2b_recording import stored_plans
  s=build_scene();route=build_routes(s);a=Agent('A1',route['A1'],s);ch=Channel(s,'fifo-ready');r=public_report(a,ch.context(0));u=[(.01,0.,'TEST')]*32;p=context_plan(r,u,rollout(a.z,u,s),'final','test');a.z=np.array(p['states'][10]);r=public_report(a,ch.context(1));u=p['commands'][1:]+[[0.,0.,'SHIFT_TAIL']];seed=context_plan(r,u,rollout(a.z,u,s),'seed','test',p['id']);stored,saved=stored_plans([p,seed]);self.assertGreater(saved,0);self.assertIsInstance(stored[1]['states'],dict)
  with tempfile.TemporaryDirectory() as d:
   root=Path(d);write_log(root/'plans.jsonl',stored);self.assertEqual(encode(lines(root/'plans.jsonl')),encode([p,seed]));stored[1]['states']['tail'][-1][0]+=1.;(root/'plans.jsonl.gz').write_bytes(gzip.compress(encode(stored)))
   with self.assertRaises(ValueError):lines(root/'plans.jsonl')

 def test_normal_diagnostics_pruned(self):
  self.assertEqual(compact(dict(_details={'H':[1]},message='solved',affine_violation=0.,separating_rows=[])),dict(message='solved',affine_violation=0.))

if __name__=='__main__':unittest.main()
