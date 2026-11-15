"""Rebuild a temporary checkout of the exact C-v1 code for zero-call full replay."""
import sys,tempfile,shutil,subprocess,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent

def replay():
    target=Path(tempfile.mkdtemp(prefix='dg002_C_v1_'))
    for name in ('inputs','sources','objects'):
        shutil.copytree(ROOT/name,target/name)
    shutil.copytree(ROOT/'results/C-new',target/'results/C-new');shutil.copy2(ROOT/'domain_checks.json',target/'domain_checks.json');shutil.copy2(ROOT/'rational_inputs.json',target/'rational_inputs.json');shutil.copy2(ROOT/'results/pre_registration.C_v1.json',target/'pre_registration.json')
    manifest=json.loads((ROOT/'executed_source_versions.C_v1.json').read_text())
    import hashlib
    for name,digest in manifest.items():
        original=ROOT/'source_archive'/digest/name
        assert hashlib.sha256(original.read_bytes()).hexdigest()==digest
        shutil.copy2(original,target/name)
    result=subprocess.run([sys.executable,'accept_case.py','--arm','C-new'],cwd=target,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
    assert result.returncode==0,result.stderr
    accepted=json.loads((target/'results/C-new/independent_acceptance.json').read_text());assert accepted['passed'];return dict(passed=True,new_calls=0,restored_source=str(target),result=accepted)
if __name__=='__main__':print(json.dumps(replay(),indent=2))
