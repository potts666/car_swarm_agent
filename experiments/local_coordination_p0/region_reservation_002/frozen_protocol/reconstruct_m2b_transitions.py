#!/usr/bin/env python3
"""Restore committed private steering queues from actual execution, never current final."""
import copy
import csv
import gzip
import json
from pathlib import Path
import time
import numpy as np
from m2b_scene import Agent,CARS
from m2b_control import commit_agent
from m2b_candidates import contract,phase_residuals,build_start
from m2b_identity import Identity
from m2b_protocol import public_report
from m2a_logs import plan_id,write_log
from m2b_recording import plain
from run_entry_bicycle import write


def stream(path):
    with gzip.open(path,'rt') as f:
        for row in f:yield json.loads(row)


def reconstruct(root):
    baseline=root.parent/'M2b_stage1_001/full_L_FIFO';run=baseline/'r1';scene=json.loads((run/'config.json').read_text())['scene'];routes=json.loads((baseline/'routes.json').read_text())
    actual={r['cycle']:r for r in stream(run/'actual.jsonl.gz')};messages={};solves={r['cycle']:r for r in stream(run/'solves.jsonl.gz')}
    for m in stream(run/'messages.jsonl.gz'):
        if m.get('event')=='send' and m.get('kind') in ('snapshot','state'):
            messages.setdefault(m['cycle'],[]).append(m)
    with (root.parent/'M2b_theory_metrics_analysis_001/seed_failure_events.csv').open() as f:
        observations=[r for r in csv.DictReader(f) if r['metrics_source']=='L-FIFO/r1']
    selected={(int(r['cycle']),r['car']) for r in observations};agents={c:Agent(c,routes[c],scene) for c in CARS};last={};previous_contract={};cases=[];matrix=[];current=[];last_final={}
    plans=stream(run/'plans.jsonl.gz')
    def process(group):
        if not group:return
        cycle=group[0]['context']['cycle'];ms=messages[cycle];ctx=group[0]['context'];committed=actual.get(cycle);work={};params={};contracts={}
        snapshots={m['sender']:m['payload'] for m in ms if m['kind']=='snapshot'}
        active={m['sender']:m['payload'] for m in ms if m['kind']=='state' and not m['payload']['context']['conditional']}
        for c in CARS:
            w=copy.deepcopy(agents[c]);w.advance(snapshots[c]['context']['owner']==c,{p:r for p,r in snapshots.items() if p!=c})
            if ctx['conditional']:
                w=copy.deepcopy(agents[c]);w.advance(ctx['owner']==c,{p:r for p,r in active.items() if p!=c})
            expected=next(p['report'] for p in group if p['car']==c)
            if public_report(w,ctx)!=expected:raise ValueError(f'independent queue restoration disagrees with public report {c}/{cycle}')
            params[c]=w.parameters(ctx['owner']==c);contracts[c]=contract(w,params[c],ctx['owner']==c);work[c]=w
            ref=solves[cycle]['local'][c]['reference_record']['parameters'][c]
            if params[c]!=ref:raise ValueError(f'restored stopping activation differs {c}/{cycle}')
        seeds={p['car']:p for p in group if p['kind']=='seed'}
        for c in CARS:
            old=previous_contract.get(c);new=contracts[c];changed=old is None or any(old[k]!=new[k] for k in ('mode','index','stop_active','authorized'))
            if (cycle,c) not in selected and not changed:continue
            w=work[c];residuals=phase_residuals(w,seeds[c]['commands'],params[c],scene);round_ctx={**ctx,'coordination_round':1};report=public_report(w,round_ctx)
            detail=dict(car=c,cycle=cycle,actual_state=w.z.tolist(),mode=w.mode,index=w.index,progress=w.s,steering_queue=w.steering,parameters=params[c],old_contract=old,new_contract=new,context=ctx,previous_committed_plan=last.get(c),old_public_candidates=seeds,old_residuals=residuals,full_ocp_checked=False,source_run=str(run))
            try:
                p,r=build_start(w,params[c],report,round_ctx,Identity(),scene,last.get(c),old,{})
                detail.update(new_candidate=p,new_validation=r);outcome='passed_declared_partial_scope';reason=r['source']
            except Exception as e:detail['new_failure']=type(e).__name__+': '+str(e);outcome='rejected_preserved';reason=str(e)
            cases.append(plain(detail));matrix.append(dict(car=c,cycle=cycle,mode=w.mode,index=w.index,observed_phase_failure=any(int(o['cycle'])==cycle and o['car']==c and o['kind']=='partial_phase_failures' for o in observations),observed_terminal_failure=any(int(o['cycle'])==cycle and o['car']==c and o['kind']=='partial_terminal_failures' for o in observations),old_partial_rows_passed=all(v<=scene['tolerance'] for v in residuals.values()),old_max_residual=max(residuals.values()),new_outcome=outcome,new_source_or_failure=reason,full_ocp_checked=False))
        if committed:
            for c in CARS:
                # Only the executed input and actual prefix update the restored private queue.
                commit_agent(work[c],[committed['controls'][c]],committed['states'][c]);agents[c]=work[c];previous_contract[c]=contracts[c]
                last[c]=next(p for p in group if p['id']==committed['plans'][c])
    for p in plans:
        if current and p['context']['cycle']!=current[0]['context']['cycle']:process(current);current=[]
        if isinstance(p['states'],dict):
            body=p['states'];source=last_final[p['car']]
            if source['id']!=body['$shift_states']:raise ValueError('bad historical state reference')
            p['states']=source['states'][body['skip']:]+body['tail']
        if p['id']!=plan_id(p):raise ValueError('historical content hash mismatch')
        if p['kind']=='final':last_final[p['car']]=p
        current.append(p)
    process(current)
    out=root/'reconstruction';out.mkdir(exist_ok=False);write_log(out/'cases.jsonl',cases)
    with (out/'case_matrix.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(matrix[0]));w.writeheader();w.writerows(matrix)
    counts={kind:sum(r['kind']==kind for r in observations) for kind in {r['kind'] for r in observations}}
    write(out/'summary.json',dict(observation_counts=counts,unique_observed_cases=len(selected),reconstructed_cases=len(cases),restored_cycles=len(actual),private_queue_source='Agent rules + previous actual submitted controls; current final is never used to infer queue',new_rejected=sum(r['new_outcome']=='rejected_preserved' for r in matrix),missing_debug_v2='source_version and diagnostic matrix absent; full OCP not reconstructible'))
    print(json.dumps(json.loads((out/'summary.json').read_text())),flush=True)


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();reconstruct(a.root)
