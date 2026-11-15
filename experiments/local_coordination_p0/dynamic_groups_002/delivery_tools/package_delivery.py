"""Standalone evidence archive with reproducible sources and an honest partial gate."""
import sys,tarfile,difflib
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from common import *

def manifest():
 files=[p for p in HERE.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc' and 'verification' not in p.relative_to(HERE).parts and p.name!='SHA256SUMS']
 (HERE/'SHA256SUMS').write_text(''.join(sha(p)+'  '+str(p.relative_to(HERE))+'\n' for p in sorted(files)))

def main():
 base=HERE.parent/'dynamic_groups_001';patch=[]
 for p in sorted(list(HERE.glob('*.py'))+list((HERE/'delivery_tools').glob('*.py'))):
  rel=p.relative_to(HERE);old=base/rel;a=old.read_text().splitlines(True) if old.exists() else [];patch.extend(difflib.unified_diff(a,p.read_text().splitlines(True),fromfile='a/'+str(rel) if old.exists() else '/dev/null',tofile='b/'+str(rel)))
 (HERE/'changes.patch').write_text(''.join(patch));write(HERE/'delivery_manifest.json',dict(experiment_status=read(HERE/'GATE.json')['status'],experiment_passed=False,calls=len(read(HERE/'solver_calls.json')),call_limit=16,ledger_sha256=sha(HERE/'solver_calls.json'),source_versions=['executed_source_versions.zero_v1.json','executed_source_versions.C_v1.json','executed_source_versions.L_v3.json'],partial_gate_preserved=True,verification_entry='verify_delivery.py',full_replay_by_actual_version=True,code={str(p.relative_to(HERE)):sha(p) for p in HERE.rglob('*.py') if '__pycache__' not in p.parts}))
 manifest();archive=HERE.parent/'dynamic_groups_002_evidence.tar.gz'
 with tarfile.open(archive,'w:gz') as tar:
  for p in sorted(HERE.rglob('*')):
   if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc':tar.add(p,arcname=HERE.name+'/'+str(p.relative_to(HERE)),recursive=False)
 receipt=dict(archive=str(archive),sha256=sha(archive),bytes=archive.stat().st_size,experiment_status='partial',native_calls=12,limit=16);write(HERE.parent/'dynamic_groups_002_evidence.archive.json',receipt);print(json.dumps(receipt,indent=2))
if __name__=='__main__':main()
