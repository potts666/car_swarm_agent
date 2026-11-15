"""Compact portable evidence archive, excluding caches and the archive itself."""
import difflib,hashlib,json,os,sys,tarfile
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(ROOT))
from common import *

def manifest():
    files=[p for p in ROOT.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc' and p.name!='SHA256SUMS' and 'verification' not in p.relative_to(ROOT).parts and not p.name.endswith('.tar.gz')]
    (ROOT/'SHA256SUMS').write_text(''.join(sha(p)+'  '+str(p.relative_to(ROOT))+'\n' for p in sorted(files)))

def main():
    # Historical source already content-checked before this phase; no external object paths remain.
    source=ROOT.parent/'region_reservation_003_reverse';parts=[]
    for p in sorted(ROOT.glob('*.py')):
        old=source/p.name;a=old.read_text().splitlines(True) if old.exists() else [];b=p.read_text().splitlines(True);parts.extend(difflib.unified_diff(a,b,fromfile='a/'+p.name if old.exists() else '/dev/null',tofile='b/'+p.name))
    (ROOT/'changes.patch').write_text(''.join(parts))
    inventory=read(ROOT/'source_inventory.json');write(ROOT/'delivery_manifest.json',dict(passed=read(ROOT/'GATE.json')['passed'],original_source_commit=inventory['source_commit'],historical_inventory='source_inventory.json',call_ledger_sha256=sha(ROOT/'solver_calls.json'),solver_source_versions=['executed_source_versions.v1.json','executed_source_versions.solver_v2.json'],code={str(p.relative_to(ROOT)):sha(p) for p in ROOT.rglob('*.py') if '__pycache__' not in p.parts},recheck_entry='verify_delivery.py',full_replay_flag='--full-replay',new_calls_allowed=0))
    manifest()
    archive=ROOT.parent/'dynamic_groups_001_evidence.tar.gz'
    with tarfile.open(archive,'w:gz') as tar:
        for p in sorted(ROOT.rglob('*')):
            if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc':tar.add(p,arcname=ROOT.name+'/'+str(p.relative_to(ROOT)),recursive=False)
    write(ROOT.parent/'dynamic_groups_001_evidence.archive.json',dict(archive=str(archive),sha256=sha(archive),bytes=archive.stat().st_size,root=ROOT.name))
    print(json.dumps(dict(archive=str(archive),sha256=sha(archive),bytes=archive.stat().st_size)),flush=True)
if __name__=='__main__':main()
