"""Summarize final B evidence without running an optimizer or altering results."""
from pathlib import Path
import sys,csv,math
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common import *

root=HERE/'results/comparison_002';status=read(root/'status.json');assert status['passed'] and len(status['stages'])==21
assert status['code_sha256']=={f.name:sha(f) for f in HERE.glob('*.py')}
rows=[];details={};gate_cases={}
for name in ('curve','straight_acceleration'):
    details[name]={};shared=set();geometry=set()
    for method in ('B-ZG','B-ZL','B-JL'):
        run=root/(name+'_'+method);s=read(run/'summary.json');acceptance=read(run/'independent_acceptance.json');assert s['passed'] and acceptance['passed']
        event=s['solver_events'][0];shared.add(event['anchor_id']);attempt=event['attempts'][-1]
        result=get(HERE/'objects',attempt['result_id']);anchor=get(HERE/'objects',event['anchor_id']);cert=get(HERE/'objects',attempt['derivative_id'])
        base=[tuple(map(F,u)) for u in anchor['plan']['remaining'][:32]];energy=H*sum(sum(x*x for x in u) for u in base)
        geo=[r for r in result['rows'] if r['kind']=='geometry'];geometry.add(content_id(geo))
        with gzip.open(run/'execution.jsonl.gz','rt') as f:trace=[json.loads(x) for x in f]
        executed=H*sum(sum(F(x)**2 for x in t['actual']['executed_input']) for t in trace[anchor['phase']:anchor['phase']+32])
        raw=result['raw_solution'];scale=F(result['numerical_variable_scale']);rawdelta=[float(scale)*float(x) for x in raw[:64]]
        rawenergy=float(H)*sum((float(x)+d)**2 for x,d in zip((x for u in base for x in u),rawdelta)) if all(map(math.isfinite,rawdelta)) else None
        norm=float(norm_upper(result['accepted_delta'])) if result['accepted'] else 0.
        if result['accepted']:assert executed==F(result['accepted_prefix_energy'])
        else:assert executed==energy
        diag=read(run/'A_direction_diagnostic.json');accepted_attempt=next((x for x in result.get('attempts',[]) if x.get('passed')),None)
        row=dict(interface=name,method=method,status=result['status'],native_called=result['solver_called'],accepted=result['accepted'],accepted_alpha=result.get('accepted_alpha',''),anchor_energy=float(energy),raw_candidate_energy=rawenergy,rational_accepted_energy=float(executed) if result['accepted'] else None,actual_prefix_energy=float(executed),energy_reduction_percent=100*float((energy-executed)/energy),actual_delta_norm=norm,A_direction_alpha=float(F(diag['alpha_lower'])),A_limiting_row=diag['limiting_rows'][0]['name'] if F(diag['alpha_lower'])<1 else 'none_at_alpha_1',variables=result['variables'],constraints=result['constraint_rows'],rho=result['physical_trust_radius'],numerical_scale=result['numerical_variable_scale'],primal_residual=result['primal_residual'],dual_residual=result['dual_residual'],objective_gap=result['absolute_objective_gap'],iterations=result['iterations'],K_executed=acceptance['actual_K_executions'],old_numeric_K_fallbacks=acceptance['damaged_K_proposals_rejected_with_actual_old_K'],derivative_build_s=cert['build_wall_s'],assembly_s=result['assembly_wall_s'],solver_s=result['solver_wall_s'],repair_s=sum(t.get('repair_wall_s',0) for t in result.get('attempts',[])),nonlinear_validation_s=sum(t.get('validation_wall_s',0) for t in result.get('attempts',[])),commit_s=trace[anchor['phase']]['commit_wall_s'],road_lower=acceptance['minimum_road_lower_m'],pair_lower=acceptance['minimum_pair_lower_m'],rejection_layer=result.get('failure_layer',''))
        rows.append(row);details[name][method]=dict(metrics=row,result_id=attempt['result_id'],anchor_id=event['anchor_id'],derivative_id=attempt['derivative_id'],conic_decomposition=accepted_attempt['cone'] if accepted_attempt else None,acceptance=acceptance)
        gate_cases[name+'_'+method]=dict(executed_safely=acceptance['passed'],solver_candidate_accepted=result['accepted'],numeric_nonzero_executed=result['accepted'] and bool(result['changed']),status=result['status'])
    assert len(shared)==len(geometry)==1,'unmatched anchor or geometry'
with (HERE/'comparison.csv').open('w',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
write(HERE/'measurements.json',dict(comparison=details,deterministic_development_only=True,matched_anchors_and_geometry=True))
shift={}
for name in ('curve','straight_acceleration'):
    s=read(root/(name+'_B-JL-shift')/'summary.json');a=read(root/(name+'_B-JL-shift')/'independent_acceptance.json');assert s['passed'] and a['passed']
    shift[name]=dict(recertified_after_actual_first_step=len(a['shift_recertifications'])==1,first_JL_candidate_accepted=s['solver_events'][0]['accepted'],second_JL_candidate_accepted=s['solver_events'][1]['accepted'],all_statuses=[[x['status'] for x in e['attempts']] for e in s['solver_events']],first_step_was_fallback=not s['solver_events'][0]['accepted'])
write(HERE/'gate_observations.json',dict(cases=gate_cases,shift=shift))
print('Six paired rows summarized; matched anchors/geometry verified; numeric accepted',sum(r['accepted'] for r in rows),'/6')
for row in rows:print(row['interface'],row['method'],row['status'],row['energy_reduction_percent'],row['A_direction_alpha'])
