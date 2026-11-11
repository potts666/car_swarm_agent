import gzip
import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from m2a_logs import encode,lines,plan_id,write_bundle
from pack_m2a_logs import pack,verify


class LogsTests(unittest.TestCase):
    def sample(self):
        p=dict(car='A',commands=[[-1e-7,-0.0]],states=[[1e-12,3.141592653589793]])
        p['id']=plan_id(p)
        return dict(plans=[p],messages=[dict(event='send',kind='final',sequence=4,application_bytes=891,wall_ns=123,payload=p),dict(event='receive',sequence=4)],solves=[dict(residual=1.0000000000000001e-7)])
    def test_bundle_roundtrip_and_single_content(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);logs=self.sample();write_bundle(root,logs)
            for k,v in logs.items():self.assertEqual(encode(lines(root/(k+'.jsonl'))),encode(v))
            stored=gzip.decompress((root/'messages.jsonl.gz').read_bytes())
            self.assertNotIn(b'commands',stored);self.assertIn(b'$plan',stored)
            self.assertNotIn('$plan',logs['messages'][0]['payload'])
    def test_missing_or_corrupt_plan_rejected(self):
        for missing in (True,False):
            with self.subTest(missing=missing),tempfile.TemporaryDirectory() as d:
                root=Path(d);write_bundle(root,self.sample());p=root/'plans.jsonl.gz'
                if missing:p.unlink()
                else:p.write_bytes(gzip.compress(encode([dict(id='wrong',states=[])])))
                with self.assertRaises((ValueError,FileNotFoundError)):lines(root/'messages.jsonl')
    def test_wire_mismatch_and_ambiguous_files_rejected(self):
        logs=self.sample();logs['messages'][0]['payload']={**logs['plans'][0],'states':[]}
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            with self.assertRaises(ValueError):write_bundle(root,logs)
            write_bundle(root,self.sample());(root/'solves.jsonl').write_bytes(b'')
            with self.assertRaises(ValueError):lines(root/'solves.jsonl')
    def test_pack_preserves_original_bytes_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'old';source.mkdir();before={}
            for k,v in self.sample().items():
                p=source/(k+'.jsonl');p.write_bytes(encode(v));before[p.name]=p.read_bytes()
            (source/'actual.csv').write_bytes(b't,v\n0,-0.0000001\n')
            output=root/'compact';pack(source,output);verify(output)
            for k,v in before.items():self.assertEqual((source/k).read_bytes(),v)
            with self.assertRaises(FileExistsError):pack(source,output)
            (output/'solves.jsonl.gz').write_bytes(gzip.compress(b'{}\n'))
            with self.assertRaises(ValueError):verify(output)

if __name__=='__main__':unittest.main()
