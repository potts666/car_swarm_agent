"""M1 artifact audit and snapshot construction; no controller or trajectory execution."""
import ast
import csv
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import numpy as np
import scipy
import osqp

OUT=Path(__file__).resolve().parent
PROJECT=OUT.parents[1]
REPO=PROJECT.parents[1]
SCRIPTS=PROJECT/'scripts'
CLOSURE=PROJECT/'results/M0_closure_001'
LOOP=CLOSURE/'closed_loop'
B1=PROJECT/'results/bicycle_e3_002'
T1=PROJECT/'results/B1&B2-T1'


def read(p):return json.loads(p.read_text())
def lines(p):return [json.loads(line) for line in p.read_text().splitlines()]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(name,data):(OUT/name).write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
def version(data):return hashlib.sha256(json.dumps(data,sort_keys=True,separators=(',',':')).encode()).hexdigest()
def relative(p):return str(p.relative_to(REPO))
def hash_map(base,mapping):
    for name,digest in mapping.items():assert sha(base/name)==digest,(base,name)


assert read(CLOSURE/'closure.json')['closed_under_original_M0_contract']
hash_map(CLOSURE,read(CLOSURE/'verification.json')['hashes'])
for name,digest in read(CLOSURE/'verification.json')['evidence_origin_sha256'].items():assert sha(REPO/name)==digest,name
historical=read(LOOP/'provenance.json');hash_map(LOOP/'source',historical['source_sha256'])
hash_map(B1/'source',read(B1/'provenance.json')['source_sha256'])
t1=read(T1/'provenance.json');hash_map(T1,t1['hashes']);hash_map(B1,t1['original_hashes'])
for dirname in ('frozen','local_paired'):
    assert (CLOSURE/dirname).is_dir()
frozen=read(CLOSURE/'frozen/provenance.json');hash_map(REPO/Path(frozen['source']),frozen['original_hashes']);hash_map(CLOSURE/'frozen',frozen['frozen_sha256'])
assert len(read(CLOSURE/'hand_cases.json'))==11
assert all(c['expected_passed']==c['result']['passed'] and c['expected_sign_jump']==c['result']['gear_sign_jump'] for c in read(CLOSURE/'hand_cases.json'))
assert all(r['accepted'] and r['original_candidate_reproduced'] for r in read(CLOSURE/'local_paired/results.json'))

runtime=set()
def imports(name):
    if name in runtime:return
    p=SCRIPTS/(name+'.py')
    if not p.exists():return
    runtime.add(name)
    for node in ast.walk(ast.parse(p.read_text())):
        if isinstance(node,ast.ImportFrom) and node.module:imports(node.module.split('.')[0])
        if isinstance(node,ast.Import):
            for item in node.names:imports(item.name.split('.')[0])
imports('run_m0')
for name in runtime:assert sha(SCRIPTS/(name+'.py'))==historical['source_sha256'][name+'.py'],name
scene=read(PROJECT/'config/entry_bicycle.json')
assert scene==read(LOOP/'scene.json')==read(PROJECT/'results/M0_001/scene.json')==read(B1/'scene.json')
actual=dict(python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,osqp=osqp.__version__,
 platform=platform.platform(),cpu_model=next(s.split(':',1)[1].strip() for s in Path('/proc/cpuinfo').read_text().splitlines() if s.startswith('model name')),
 threads={k:os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')})
assert all(actual[k]==historical[k] for k in actual)
assert os.environ.get('ROS_DISTRO')=='jazzy'

source=OUT/'source';source.mkdir()
for name in sorted(runtime):shutil.copy2(SCRIPTS/(name+'.py'),source/(name+'.py'))
(OUT/'config').mkdir();shutil.copy2(PROJECT/'config/entry_bicycle.json',OUT/'config/entry_bicycle.json')
for name in ('bicycle_joint_qp.md','longitudinal_joint_qp.md','requirements-qp.txt'):shutil.copy2(PROJECT/name,OUT/name)
test_hashes={};test_locations=[]
modules=read(CLOSURE/'verification.json')['regression_tests']['modules']
for module in modules:
    name=module+'.py';current=PROJECT/'tests'/name
    if name=='test_stopping_acceptance.py':
        expected=sha(CLOSURE/'analysis_source'/name);location=str(CLOSURE/'analysis_source'/name)
    elif (LOOP/'source'/name).exists():
        expected=historical['source_sha256'][name];location=str(LOOP/'source'/name)
    else:
        raw=subprocess.check_output(['git','show',historical['git_head']+':'+relative(current)],cwd=REPO)
        expected=hashlib.sha256(raw).hexdigest();location='git '+historical['git_head']+':'+relative(current)
    assert sha(current)==expected,name
    test_hashes[name]=expected;test_locations.append(dict(name=name,sha256=expected,immutable_source=location))

runtime_hashes={name+'.py':sha(source/(name+'.py')) for name in sorted(runtime)}
acceptance_names=['bicycle_core.py','verify_entry_bicycle.py','verify_m0.py','run_joint_e2.py']
versions=dict(source=version(runtime_hashes),config=version(scene),acceptance=version({n:runtime_hashes[n] for n in acceptance_names}))
import linearized_joint as bj
import longitudinal_joint as lj
config=dict(label='B2-C/M1-v1',source_version=versions['source'],config_version=versions['config'],acceptance_version=versions['acceptance'],scene=scene,
 constructor='BicycleController(scene, paths, updates=3, reference_warm_start=True)',
 reference=dict(weights=lj.WEIGHTS,feasibility_band=bj.BicycleController.reference_feasibility_band,eps_abs=1e-8,eps_rel=1e-8,max_iter=20000,polishing=True,adaptive_rho_interval=50,
 warm_start='shift last committed reference primal per car; phase/index invalidate; clamp to current input bounds; no dual reuse'),
 bicycle=dict(state_weights=bj.STATE_WEIGHT.tolist(),input_weights=bj.INPUT_WEIGHT.tolist(),input_change_weight=.02,terminal_state_weight_multiplier=10,
 state_trust=bj.STATE_TRUST.tolist(),input_trust=bj.INPUT_TRUST.tolist(),error_limits=bj.ERROR_LIMIT,numerical_band=bj.NUMERICAL_BAND,
 eps_abs=1e-7,eps_rel=1e-7,max_iter=10000,polishing=True,adaptive_rho_interval=50,maximum_updates=bj.MAX_UPDATES,deadline_s=bj.DEADLINE_S,
 derivative='one 0.2 s RK4; defect aligns ten-substep reference; not exact composed-map derivative',stop_input_repair='MOVE and stop_at_end; if direction*(v+h*a)<0, use a=-v/h before original complete candidate checks',
 terminal_box=dict(x_m=.005,y_m=.005,heading_rad=.003),terminal_steering='segment delta equality'),
 physical_acceptance_tolerance=scene['tolerance'],sign_product_rejection_m2_s2=-scene['tolerance']**2,
 phase_transition=dict(arrival_tolerance_m=1e-6,near_rest_m_s=1e-6,rest_for_transition_m_s=1e-12),
 planning_budget_s=bj.DEADLINE_S,control_period_s=scene['h'],future_witness_budget_s=bj.BicycleController.future_deadline_s,
 fixed_permission_order=['A','B'],state_frozen_during_computation=True,ideal_cycle_synchronous_DDS=True,
 input_repair_is_not_exact_QP_optimum=True,hard_realtime_guarantee=False)
versions['scene']=version(scene)
versions['config']=version({k:v for k,v in config.items() if k not in ('source_version','config_version','acceptance_version')})
config['config_version']=versions['config']
write('frozen_config.json',config)

run_metrics={};checks={};evidence=[]
for arm in ('repaired','unrepaired'):
 for rep in (1,2):
    name=f'{arm}_r{rep}';d=LOOP/name
    summary=read(d/'summary.json');m=read(d/'metrics_e3.json');v=read(d/'independent_validation.json');ros=read(d/'ros_validation.json');f=read(d/'future_execution_consistency.json')
    planning=read(d/'planning.json');solves=lines(d/'solves.jsonl');predictions=lines(d/'predictions.jsonl');events=lines(d/'events.jsonl');future=lines(d/'future_diagnostics.jsonl')
    rows=list(csv.DictReader((d/'actual.csv').open()));N=scene['N'];h=scene['h'];ds=scene['sample_s'];committed=sum(p['committed'] for p in planning)
    assert len(solves)==len(predictions)==committed
    assert len(planning)==committed+1 and planning[-1]['outcome']=='completed'
    assert all(s['cycle']==i and s['accepted'] for i,s in enumerate(solves))
    assert all(abs(p['total_cycle_wall_s']-(p['end_monotonic_s']-p['start_monotonic_s']))<1e-9 for p in planning)
    actual_by_car={c:[r for r in rows if r['car']==c] for c in ('A','B')}
    for c in actual_by_car:assert len(actual_by_car[c])==1+committed*round(h/ds)
    for prediction in predictions:
     stamp=prediction['time_s'];solve=solves[round((stamp-scene['epoch_s'])/h)]
     for i,c in enumerate(('A','B')):
      logged=np.array(prediction['commands'][c],dtype=object)[:,:2].astype(float)
      np.testing.assert_allclose(logged,np.array(solve['solution'])[i*2*N:(i+1)*2*N].reshape(N,2),atol=1e-12,rtol=0)
      start=round((stamp-scene['epoch_s'])/ds)
      applied=actual_by_car[c][start+1:start+1+round(h/ds)]
      np.testing.assert_allclose([[float(r['a']),float(r['omega'])] for r in applied],np.tile(logged[0],(round(h/ds),1)),atol=1e-12,rtol=0)
    effort=sum(ds*(float(r['a'])**2+float(r['omega'])**2) for c in actual_by_car for r in actual_by_car[c][1:])
    assert abs(effort-m['normalized_execution_effort'])<1e-9
    t=np.array([p['total_cycle_wall_s'] for p in planning]);timing=dict(count=len(t),p50_ms=float(np.percentile(t,50)*1000),p95_ms=float(np.percentile(t,95)*1000),p99_ms=float(np.percentile(t,99)*1000),max_ms=float(max(t)*1000),over_200ms=int(np.sum(t>h)))
    assert timing['count']==m['timed_cycles'] and abs(timing['max_ms']-m['cycle_max_ms'])<1e-7
    main_reference=sum(s['reference_numeric_qp_calls'] for s in solves);main_bicycle=sum(s['bicycle_numeric_qp_calls'] for s in solves)
    fs=[s for w in future for s in w['solves']]
    future_reference=sum(s['reference_numeric_qp_calls'] for s in fs);future_bicycle=sum(s['bicycle_numeric_qp_calls'] for s in fs)
    assert (main_reference,main_bicycle,future_reference,future_bicycle)==tuple(m[k] for k in ('main_reference_numeric_QPs','main_bicycle_numeric_QPs','future_reference_numeric_QPs','future_bicycle_numeric_QPs'))
    repairs=[dict(cycle=s['cycle'],update=i,**a) for s in solves for i,q in enumerate(s['subproblems'],1) for a in q.get('stop_input_adjustments',[])]
    status_hist={status:sum(s['reference_record']['message']==status for s in solves) for status in sorted({s['reference_record']['message'] for s in solves})}
    check=dict(artifact_integrity_passed=True,both_goals_reached=bool(summary['both_completed'] and all(v['independently_replayed_goals'].values())),
     original_full_acceptance_passed=bool(v['passed'] and v['original_log_validation']['passed']),protocol_passed=ros['passed'],
     timing_complete=v['all_cycle_exits_timed'],observed_deadline_misses=dict(control_period_200ms=timing['over_200ms'],prediction_budget_500ms=m['deadline_rejections']),
     future_witness_and_execution_consistency_passed=f['passed'],independent_validation_reused=True,new_integration_executed=False)
    assert check['both_goals_reached'] and check['protocol_passed'] and check['timing_complete']
    assert check['original_full_acceptance_passed']==(arm=='repaired')
    if arm=='repaired':assert len(repairs)==6 and not v['original_log_validation']['component_equations']['B']['gear_sign_jump']
    checks[name]=check
    grouped_timing=dict(
     decision_communication_and_prepare_including_future_s=sum(p['decision_communication_wall_s'] for p in planning),
     future_witness_subset_s=sum(p['future_check_wall_s'] for p in planning),
     reference_build_setup_solve_check_s=sum(p['reference_build_solve_check_s'] or 0. for p in planning),
     reference_OSQP_solve_only_s=sum(s['reference_record']['solver_wall_s'] for s in solves),
     reference_OSQP_setup_s=sum(s['reference_record']['setup_wall_s'] for s in solves),
     bicycle_subproblem_OSQP_setup_and_solve_subset_s=sum(q['solver_wall_s'] for s in solves for q in s['subproblems']),
     prediction_including_reference_bicycle_and_internal_replay_s=sum(p['prediction_wall_s'] or 0. for p in planning),
     outer_acceptance_buffered_record_and_commit_s=sum(p['acceptance_wall_s'] for p in planning),
     total_cycle_s=float(t.sum()),total_policy_including_bulk_export_s=summary['total_policy_wall_s'],
     unavailable_separate=['bicycle assembly alone','reference rollout alone','internal nonlinear replay alone','buffered logging alone','commit alone'])
    per_car= v['original_log_validation']['metrics']['per_car']
    protocol={}
    for c in ('A','B'):
     def event(kind):return next((e['time_s'] for e in events if e['kind']==kind and e.get('car')==c),None)
     protocol[c]=dict(request_s=event('request'),grant_s=event('grant'),release_s=event('release'),completion_s=event('car_completed'),
      request_to_grant_s=event('grant')-event('request'),stationary_per_stage_s=per_car[c]['stationary_per_stage_s'])
     speeds=np.array([float(r['v']) for r in actual_by_car[c]]);stopped=np.abs(speeds)<=scene['terminal']['speed_m_s']
     protocol[c]['stopped_to_moving_transitions']=int(np.sum(stopped[:-1]&~stopped[1:]))
     last=actual_by_car[c][-1];goal=scene['goals'][c]
     endpoint=dict(position_m=float(np.linalg.norm([float(last['x'])-goal[0],float(last['y'])-goal[1]])),heading_rad=abs(float(last['psi'])-goal[2]),speed_m_s=abs(float(last['v'])),steering_rad=abs(float(last['delta'])))
     for k,val in endpoint.items():assert abs(val-m['endpoint_errors'][c][k])<1e-9
    run_metrics[name]=dict(evidence_source=relative(d),new_experiment=False,completion_s=m['completion_sim_s'],per_car_completion_s=summary['completion_per_car_s'],
     endpoint_errors=m['endpoint_errors'],minimum_gap_m=m['minimum_sampled_gap_m'],normalized_control_cost=effort,protocol_and_stops=protocol,
     repair_count=len(repairs),repairs=repairs,main_reference_qps=main_reference,main_bicycle_qps=main_bicycle,future_reference_qps=future_reference,future_bicycle_qps=future_bicycle,
     future_update_histogram={str(k):sum(s['qp_updates']==k for s in fs) for k in range(4)},
     future_repair_count=sum(len(q.get('stop_input_adjustments',[])) for s in fs for q in s['subproblems']),
     future_rejections=sum(not s['accepted'] for s in fs),future_deadline_rejections=sum(s.get('deadline_exceeded',False) for s in fs),
     update_histogram=m['update_count_histogram'],sequential_update_cycles=m['sequential_update_cycles'],reference_status_histogram=status_hist,
     reference_rejections=m['reference_rejections'],bicycle_rejections=m['rejected_subproblems'],prediction_deadline_rejections=m['deadline_rejections'],timing=timing,grouped_timing=grouped_timing,acceptance=check)
    for p in sorted(d.iterdir()):
     if p.is_file():evidence.append(dict(path=relative(p),sha256=sha(p),bytes=p.stat().st_size,role=f'raw_and_checks_{name}'))

for arm in ('repaired','unrepaired'):
 assert (LOOP/f'{arm}_r1/actual.csv').read_bytes()==(LOOP/f'{arm}_r2/actual.csv').read_bytes()
assert (LOOP/'unrepaired_r1/actual.csv').read_bytes()==(PROJECT/'results/M0_001/B2_sequence_warm_r1/actual.csv').read_bytes()
b1summary=read(B1/'B1/summary.json');b1validation=read(B1/'B1/validation.json');assert b1summary['both_completed'] and b1validation['passed'] and read(B1/'B1/ros_validation.json')['passed']
rows=list(csv.DictReader((B1/'B1/actual.csv').open()));effort=sum(scene['sample_s']*(float(r['a'])**2+float(r['omega'])**2) for c in ('A','B') for r in [x for x in rows if x['car']==c][1:])
b1timing=read(B1/'B1/planning.json');times=np.array([p['total_cycle_wall_s'] for p in b1timing])
run_metrics['B1_historical']=dict(evidence_source=relative(B1/'B1'),new_experiment=False,comparison_role='historical diagnostic only; no speedup/ranking',
 both_goals_reached=True,original_full_acceptance_passed=True,protocol_passed=True,timing_complete=False,
 completion_s=b1summary['T_complete_sim_s'],normalized_control_cost=effort,minimum_gap_m=b1validation['trajectory']['minimum_gap_m'],
 timed_cycles=len(times),cycle_p50_ms=float(np.percentile(times,50)*1000),cycle_p95_ms=float(np.percentile(times,95)*1000),cycle_p99_ms=float(np.percentile(times,99)*1000),cycle_max_ms=float(max(times)*1000),
 over_200ms=int(np.sum(times>scene['h'])),timing_limit='historical recorder omits final completion exit; CPU/thread provenance incomplete; not matched to current all-exit timing',
 source_provenance=relative(B1/'provenance.json'))
for folder in (B1/'B1',):
 for p in sorted(folder.iterdir()):
  if p.is_file():evidence.append(dict(path=relative(p),sha256=sha(p),bytes=p.stat().st_size,role='B1 historical diagnostic'))

core_locations={}
for name in ('linearized_joint','longitudinal_joint','ros_flag','run_joint_e2','verify_m0','verify_entry_bicycle'):
 tree=ast.parse((source/(name+'.py')).read_text())
 core_locations[name+'.py']={getattr(n,'name'):n.lineno for n in ast.walk(tree) if isinstance(n,(ast.ClassDef,ast.FunctionDef))}
analysis_changes=[n for n,d in historical['source_sha256'].items() if (SCRIPTS/n).exists() and sha(SCRIPTS/n)!=d]
manifest=dict(selected_implementation='B2-C/M1-v1',versions=versions,maintenance_owner=str(SCRIPTS),
 authoritative_frozen_runtime=str(source),frozen_snapshot_is_evidence_not_maintenance_directory=True,
 runtime_files=[dict(name=n,maintenance_path=str(SCRIPTS/n),source_run_snapshot=str(LOOP/'source'/n),frozen_path=str(source/n),sha256=d,behavior_equivalent=True) for n,d in runtime_hashes.items()],
 acceptance_files=acceptance_names,core_locations=core_locations,
 analysis_only_changes_since_run=analysis_changes,analysis_changes_are_outside_selected_import_closure=True,
 regression_evidence=dict(source=str(CLOSURE/'verification.json'),count=34,rerun=False,test_sha256=test_hashes,version_locations=test_locations,test_bodies_copied=False),
 B1_source_hashes_checked=True,T1_resolved_path=str(T1),T1_requested_alias='B1&B2-T1-03 not present; same-origin archive located by recorded SHA256',raw_evidence=evidence)
write('source_manifest.json',manifest);write('metrics.json',run_metrics)
write('repeatability.json',dict(new_experiments_executed=False,evidence_reused_from=relative(LOOP),
 repaired_actual_bytes_equal=True,repaired_actual_sha256=sha(LOOP/'repaired_r1/actual.csv'),unrepaired_actual_bytes_equal=True,
 unrepaired_matches_M0_001=True,interpretation='two deterministic repeats in one scenario/environment; not success probability or tail distribution estimate'))
write('provenance.json',dict(new_experiments_executed=False,new_closed_loop_runs=0,new_integrations_executed=False,new_regression_executed=False,
 evidence_environment=historical,observed_current_environment=actual,declared_environment_matches=True,ROS_DISTRO=os.environ.get('ROS_DISTRO'),
 historical_ROS_patch_build_not_recorded=True,task_path='/home/sunflower/.codex/attachments/bfb57f1c-af26-4409-b117-1f7e3989fd33/已粘贴的文本.txt',
 task_sha256=sha(Path('/home/sunflower/.codex/attachments/bfb57f1c-af26-4409-b117-1f7e3989fd33/已粘贴的文本.txt')),
 git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
 freeze_command='source /opt/ros/jazzy/setup.bash && OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/t1_qp_deps:'+str(SCRIPTS)+':$PYTHONPATH python3 '+str(OUT/'build_manifest.py'),
 original_files_modified=False,geometry_verifier_shared=True))
write('verification.json',dict(artifact_integrity_passed=True,maintenance_runtime_equivalent=True,configuration_equivalent=True,declared_environment_equivalent=True,
 original_full_acceptance_passed=all(checks[n]['original_full_acceptance_passed'] for n in ('repaired_r1','repaired_r2')),
 both_goals_reached=True,protocol_passed=True,timing_complete=True,
 observed_deadline_misses={n:checks[n]['observed_deadline_misses'] for n in ('repaired_r1','repaired_r2')},
 independent_full_acceptance_reused=True,new_independent_reintegration=False,packaging_smoke_check_passed=None,
 regression_evidence_reused=dict(count=34,tests_version_checked=True,rerun=False),per_run=checks,
 historical_B1=dict(artifact_integrity_passed=True,both_goals_reached=True,original_full_acceptance_passed=True,protocol_passed=True,timing_complete=False,current_environment_fair_performance_comparison=False)))
with (OUT/'comparison.csv').open('w',newline='') as f:
 fields=['implementation','run','evidence_source','new_experiment','goals_reached','original_full_acceptance','protocol_passed','timing_complete','completion_s','control_cost','minimum_gap_m','repairs','main_reference_qps','main_bicycle_qps','future_reference_qps','future_bicycle_qps','timed_cycle_count','p50_ms','p95_ms','p99_ms','max_ms','over_200ms','timing_comparability']
 writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
 for name,m in run_metrics.items():
  if name=='B1_historical':
   writer.writerow(dict(implementation='B1',run=name,evidence_source=m['evidence_source'],new_experiment=False,goals_reached=True,original_full_acceptance=True,protocol_passed=True,timing_complete=False,completion_s=m['completion_s'],control_cost=m['normalized_control_cost'],minimum_gap_m=m['minimum_gap_m'],timed_cycle_count=m['timed_cycles'],p50_ms=m['cycle_p50_ms'],p95_ms=m['cycle_p95_ms'],p99_ms=m['cycle_p99_ms'],max_ms=m['cycle_max_ms'],over_200ms=m['over_200ms'],timing_comparability='historical diagnostic only; no relative ranking'))
  else:
   a=m['acceptance'];t=m['timing'];writer.writerow(dict(implementation='B2-C' if name.startswith('repaired') else 'B2-unrepaired historic failure comparator',run=name,evidence_source=m['evidence_source'],new_experiment=False,goals_reached=a['both_goals_reached'],original_full_acceptance=a['original_full_acceptance_passed'],protocol_passed=a['protocol_passed'],timing_complete=a['timing_complete'],completion_s=m['completion_s'],control_cost=m['normalized_control_cost'],minimum_gap_m=m['minimum_gap_m'],repairs=m['repair_count'],main_reference_qps=m['main_reference_qps'],main_bicycle_qps=m['main_bicycle_qps'],future_reference_qps=m['future_reference_qps'],future_bicycle_qps=m['future_bicycle_qps'],timed_cycle_count=t['count'],p50_ms=t['p50_ms'],p95_ms=t['p95_ms'],p99_ms=t['p99_ms'],max_ms=t['max_ms'],over_200ms=t['over_200ms'],timing_comparability='same closure experiment; B1 is not matched'))
print(json.dumps(dict(artifact_integrity_passed=True,runtime_files=len(runtime),new_experiments_executed=False,source_version=versions['source'],config_version=versions['config'],acceptance_version=versions['acceptance'])))
