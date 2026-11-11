"""Read-only evidence checks; writes only this package's verification summary."""
import hashlib
import json
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parent
BASE=ROOT.parents[1]
SCRIPTS=BASE/'scripts'

def read(p):return json.loads(p.read_text())
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def main():
    manifest=read(ROOT/'artifact_manifest.json')
    for name,digest in manifest.items():assert sha(ROOT/name)==digest,name
    frozen=read(ROOT/'design/source_manifest.json')
    for name,digest in frozen.items():assert sha(ROOT/'design'/name)==digest,name
    core=['m2a_local.py','m2a_protocol.py','run_m2a.py','verify_m2a.py','audit_m2a_future.py']
    for name in core:assert sha(SCRIPTS/name)==frozen[name],name
    old=ROOT.parent/'M1_motion_freeze_001'
    subprocess.run(['python3',str(old/'verify_freeze.py')],check=True,capture_output=True,text=True)
    runtime=read(old/'source_manifest.json')['runtime_files']
    for item in runtime:assert sha(SCRIPTS/item['name'])==item['sha256'],item['name']
    runs={}
    config=read(ROOT/'frozen_M1_config.json')
    for name in ('C-new_r1','C-new_r2','L-new_r1','L-new_r2'):
        p=ROOT/'formal'/name;snapshot=p.with_name(name+'_source')
        provenance=read(snapshot/'provenance.json')
        for file,digest in provenance['source_sha256'].items():assert sha(snapshot/file)==digest,(name,file)
        for file in core:assert provenance['source_sha256'][file]==frozen[file],(name,file)
        for item in runtime:assert provenance['source_sha256'][item['name']]==item['sha256'],(name,item['name'])
        cfg=read(p/'config.json');assert cfg['scene']==config['scene'] and cfg['max_coordination_rounds']==1
        summary=read(p/'summary.json');v=read(p/'independent_validation.json');offline=read(p/'centralized_future_diagnostic.json')
        for key in ('actual_full_acceptance','physical_and_original_sign_check_passed','release_and_plan_version_contract_passed','timing_complete'):assert v[key],(name,key)
        assert summary['both_completed'] and summary['online_full_future_calls']==0
        if name.startswith('L-'):
            assert summary['online_joint_optimizer_forbidden'] and v['local_information_boundary_passed']
            for dimensions in v['local_decision_dimensions'].values():assert dimensions['reference_max']<=32 and dimensions['bicycle_max']<=64
        assert offline['executed'] and offline['online_records_unchanged']
        for file,digest in offline['input_hashes'].items():assert sha(p/file)==digest,(name,file)
        runs[name]=dict(actual_full_acceptance=True,physical_and_sign=True,new_contract_and_versions=True,timing_complete=True,
            local_information_boundary=v['local_information_boundary_passed'],completion_s=summary['T_complete_sim_s'],
            online_future_calls=0,offline_central_diagnostic_passed=all(c['check']['passed'] for c in offline['checkpoints']))
    repeat=read(ROOT/'repeatability.json')
    for arm in ('C-new','L-new'):
        assert repeat[arm]['actual_bytes_equal']
        assert sha(ROOT/'formal'/f'{arm}_r1/actual.csv')==sha(ROOT/'formal'/f'{arm}_r2/actual.csv')==repeat[arm]['sha256']
    tests=(ROOT/'analysis_source/test_m2a.log').read_text();assert 'Ran 11 tests' in tests and tests.rstrip().endswith('OK')
    assert sha(BASE/'tests/test_m2a.py')==sha(ROOT/'analysis_source/test_m2a.py')
    for name in ('report_m2a.py',):assert sha(SCRIPTS/name)==sha(ROOT/'analysis_source'/name)
    debug={}
    for version in (1,2,3):
        p=ROOT/f'debug_v{version}/L-new';v=read(p/'independent_validation.json')
        assert not v['actual_full_acceptance'] and v['actual_prefix_acceptance']
        debug[f'v{version}']=dict(full_acceptance=False,prefix_acceptance=True,failure=read(p/'summary.json')['failure'])
    result=dict(artifact_integrity_passed=True,hashed_artifacts=len(manifest),formal_source_frozen_and_equal=True,
        original_M1_maintenance_and_evidence_unchanged=True,runs=runs,deterministic_actual_repeatability=True,
        new_protocol_and_local_tests=11,new_tests_passed=True,M1_regressions_reused_not_rerun=34,debug_failures_retained=debug,
        M2b_entry_criteria_met=True,M2b_implemented=False,continuous_safety_proven=False,recursive_feasibility_proven=False,hard_realtime_proven=False)
    (ROOT/'verification.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(dict(artifact_integrity_passed=True,formal_runs_passed=len(runs),new_tests_passed=11,M2b_entry_criteria_met=True)))

if __name__=='__main__':main()
