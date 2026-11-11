"""Post-run archival logic, independent of the frozen control implementation."""
from pathlib import Path
import sys
import tempfile
import json
import gzip
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from m2a_logs import encode
from m2b_diagnostic_archive import archive,restore,logical_digest

class ArchiveTests(unittest.TestCase):
 def test_full_bytes_recover_and_missing_or_corrupt_object_rejected(self):
  with tempfile.TemporaryDirectory() as folder:
   root=Path(folder)/'study';root.mkdir();(root/'execution_status.json').write_text('{"completed":true}')
   for group in ('C-FIFO','L-FIFO','L-ALT'):
    for i in (1,2):
     d=root/group/f'r{i}';d.mkdir(parents=True);row={'cycle':i,'solve_wall_s':.001*i,'qp':{'_details':{'H':[[1e-7,-0.0]],'lower':['-Infinity']}}};(d/'diagnostics.jsonl.gz').write_bytes(gzip.compress(encode([row]),mtime=0))
   expected={str(p.relative_to(root)):logical_digest(p) for p in root.glob('*/r*/diagnostics.jsonl.gz')};archive(root)
   self.assertEqual(len(list((root/'diagnostic_objects').iterdir())),1);output=Path(folder)/'restored';restore(root,output)
   for name,digest in expected.items():self.assertEqual(logical_digest(output/name),digest)
   obj=next((root/'diagnostic_objects').iterdir());obj.write_bytes(gzip.compress(b'{}\n'))
   with self.assertRaises(ValueError):restore(root,Path(folder)/'bad')
   obj.unlink()
   with self.assertRaises(FileNotFoundError):restore(root,Path(folder)/'missing')

if __name__=='__main__':unittest.main()
