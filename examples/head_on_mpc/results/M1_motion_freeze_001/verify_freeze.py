"""Verify the frozen M1 package and referenced original evidence, without executing control."""
import hashlib
import json
import subprocess
from pathlib import Path

OUT=Path(__file__).resolve().parent
REPO=OUT.parents[3]

def read(p):return json.loads(p.read_text())
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def version(data):return hashlib.sha256(json.dumps(data,sort_keys=True,separators=(',',':')).encode()).hexdigest()

manifest=read(OUT/'source_manifest.json')
for item in manifest['runtime_files']:assert digest(OUT/'source'/item['name'])==item['sha256'],item['name']
for item in manifest['regression_evidence']['version_locations']:
    location=item['immutable_source']
    if location.startswith('git '):
        raw=subprocess.check_output(['git','show',location[4:]],cwd=REPO)
        assert hashlib.sha256(raw).hexdigest()==item['sha256'],item['name']
    else:assert digest(Path(location))==item['sha256'],item['name']
for item in manifest['raw_evidence']:assert digest(REPO/item['path'])==item['sha256'],item['path']
for name,sha in read(OUT/'artifact_hashes.json').items():assert digest(OUT/name)==sha,name
config=read(OUT/'frozen_config.json');assert version(config['scene'])==manifest['versions']['scene']
assert version({k:v for k,v in config.items() if k not in ('source_version','config_version','acceptance_version')})==manifest['versions']['config']
assert config['config_version']==read(OUT/'selection.json')['config_version']==manifest['versions']['config']
assert read(OUT/'config/entry_bicycle.json')==config['scene']
assert version({x['name']:x['sha256'] for x in manifest['runtime_files']})==manifest['versions']['source']
assert version({n:digest(OUT/'source'/n) for n in manifest['acceptance_files']})==manifest['versions']['acceptance']
assert read(OUT/'selection.json')['new_experiments_executed'] is False
assert read(OUT/'selection.json')['M1_complete']
assert read(OUT/'packaging_check.json')['passed']

# Check seed provenance against committed reference logs without building or solving any QP.
seed_checks=0
for run in ('repaired_r1','repaired_r2'):
 d=OUT.parent/'M0_closure_001/closed_loop'/run
 cfg=read(d/'config.json');assert cfg['scene']==config['scene'] and cfg['reference_warm_start'] is True
 solves=[json.loads(s) for s in (d/'solves.jsonl').read_text().splitlines()]
 constraints={(r['cycle'],r['car']):r for r in [json.loads(s) for s in (d/'constraints.jsonl').read_text().splitlines()]}
 for cycle,record in enumerate(solves):
  reference=record['reference_record']
  if not reference['warm_start_used']:continue
  assert cycle>0 and solves[cycle-1]['accepted']
  expected=[]
  for i,car in enumerate(('A','B')):
   row=constraints[cycle,car];previous=constraints[cycle-1,car];p=reference['parameters'][car];N=cfg['scene']['N'];h=cfg['scene']['h'];vehicle=cfg['scene']['vehicle']
   if (row['segment'],row['phase'])==(previous['segment'],previous['phase']):
    seed=solves[cycle-1]['reference_record']['solution'][i*N+1:(i+1)*N]+[0.]
   else:seed=[0.]*N
   for k,value in enumerate(seed):
    if p['cap']==0.:lo=hi=0.
    elif p['stop_at_end'] and p['stop_index']==1:lo=hi=-p['v0']/h if k==0 else 0.
    elif k>=p['stop_index']:lo=hi=0.
    elif p['direction']==1:lo,hi=vehicle['a_min'],vehicle['a_max']
    else:lo,hi=-vehicle['a_max'],-vehicle['a_min']
    expected.append(min(hi,max(lo,value)))
  assert max(abs(x-y) for x,y in zip(expected,reference['warm_start_solution']))<1e-12,(run,cycle)
  seed_checks+=1
print(json.dumps(dict(artifact_integrity_passed=True,original_evidence_hashes_passed=True,committed_reference_seed_checks=seed_checks,
 new_experiments_executed=False,new_integrations_executed=False,
 maintenance_still_matches=all(Path(x['maintenance_path']).exists() and digest(Path(x['maintenance_path']))==x['sha256'] for x in manifest['runtime_files']))))
