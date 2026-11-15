#!/usr/bin/env python3
"""Final evidence/source checks after all online control and independent audits finish."""
import argparse
import hashlib
import json
from pathlib import Path


def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def read(p):return json.loads(p.read_text())
def write(p,x):p.write_text(json.dumps(x,indent=2,ensure_ascii=False)+'\n')

def check(root):
 scripts=Path(__file__).resolve().parent;freeze=read(root/'formal_freeze.json');status=read(root/'execution_status.json')
 assert status['completed'],'formal execution incomplete'
 for name,digest in freeze['source_sha256'].items():assert sha(scripts/name)==digest,name
 assert sha(scripts.parent/'config/m2b_fourcar.json')==freeze['scene_sha256']
 old=read(root.parent/'M1_motion_freeze_001/source_manifest.json')
 for name in ('bicycle_core.py','longitudinal_joint.py','linearized_joint.py','verify_entry_bicycle.py'):
  expected=next(x['sha256'] for x in old['runtime_files'] if x['name']==name);assert sha(scripts/name)==expected,name
 results={};versions=set();motion_configs=[]
 for group in ('C-FIFO','L-FIFO','L-ALT'):
  env=read(root/group/'environment.json');version=env['source_version'];versions.add(version)
  snapshot=root/'sources'/version;hashes=read(snapshot/'manifest.json')
  for name,digest in hashes.items():assert sha(snapshot/name)==digest==freeze['source_sha256'][name],(group,name)
  for i in (1,2):
   d=root/group/f'r{i}';cfg=read(d/'config.json');scene=dict(cfg['scene']);assert scene.pop('source_version')==version;assert scene==freeze['scene'];motion_configs.append(scene)
   summary=read(d/'summary.json');v=read(d/'independent_validation.json')
   assert v['actual_prefix_acceptance'] and v['physical']['passed'] and len(v['physical']['pair_gaps_m'])==6,(group,i)
   assert v['protocol_passed'] and v['all_exit_timing_passed'] and v['original_sign_check_passed'] and not v['violations']
   assert summary['timed_cycles']>=summary['committed_cycles'] and cfg['online_future_calls']==0
   assert (d/'diagnostics.jsonl.gz').exists() and (d/'recording_stats.json').exists(),(group,i,'missing detailed/export evidence')
   if group.startswith('L'):
    assert cfg['online_joint_forbidden'] and v['local_information_boundary_passed'] and v['actual_full_acceptance'],(group,i)
    for c,n in v['dimensions'].items():assert n['reference']<=32 and n['bicycle']<=64,(group,c)
   results[f'{group}/r{i}']=dict(all_completed=summary['all_completed'],actual_full_acceptance=v['actual_full_acceptance'],completion_s=summary['completion_sim_s'],min_gap_m=v['physical']['minimum_gap_m'],original_sign_passed=True,six_pair_passed=True,protocol_passed=True,all_exit_timing_passed=True,local_information_boundary=v['local_information_boundary_passed'],source_version=version)
 assert len(versions)==1 and len(results)==6,'formal versions differ'
 assert all(x==motion_configs[0] for x in motion_configs),'motion settings changed'
 tests=(root/'tests_frozen.log').read_text();assert 'Ran 30 tests' in tests and tests.rstrip().endswith('OK')
 delivered=(root/'tests_delivery.log').read_text();assert 'Ran 32 tests' in delivered and delivered.rstrip().endswith('OK')
 assert read(root/'archival_input_integrity.json')['acceptance_inputs_unchanged']
 assert all(v['passed'] for v in read(root/'seed_replay_validation.json').values())
 for item in read(root/'diagnostic_archive_manifest.json')['files'].values():assert item['original_logical_sha256']==item['restored_logical_sha256']
 ref=read(root/'plan_reference_validation.json');assert ref['original_uncompressed_sha256']==ref['restored_uncompressed_sha256']
 data=dict(six_runs_executed_and_independently_audited=True,all_six_full_acceptance=all(v['actual_full_acceptance'] for v in results.values()),frozen_sources_match=True,same_motion_config_all_arms=True,legacy_physics_and_M1_controller_unchanged=True,tests_passed=32,pre_formal_tests_passed=30,new_M2b_motion_protocol_tests=15,post_archival_edge_tests=2,related_M2a_tests_rerun=15,old_M1_34_tests_not_rerun=True,plan_reference_restores_original_bytes=True,runs=results,online_central_future_calls=0,hard_realtime_claimed=False,continuous_safety_claimed=False,recursive_feasibility_claimed=False)
 write(root/'verification.json',data)
 files={str(p.relative_to(root)):sha(p) for p in sorted(root.rglob('*')) if p.is_file() and '__pycache__' not in p.parts and p.name!='artifact_manifest.json'}
 write(root/'artifact_manifest.json',files);print(json.dumps(data),flush=True);return data

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();check(a.run)
