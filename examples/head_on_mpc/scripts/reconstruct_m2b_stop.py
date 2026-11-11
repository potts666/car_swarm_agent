#!/usr/bin/env python3
"""Natural debug_v1 stopping counterexample, restored from original public/executed data."""
import copy
import gzip
import json
from pathlib import Path
import numpy as np
from scipy.sparse import csc_matrix
import osqp
from m2b_scene import Agent,CARS
from m2b_protocol import public_report
from m2b_control import commit_agent
from m2b_identity import Identity
from m2b_candidates import contract,build_start
from m2b_recording import lines,plain
from m2b_diagnostic_archive import expand,object_loader
from run_entry_bicycle import write


def run(root):
    old=root.parent/'M2b_fourcar_001';directory=old/'debug_v1/r1';cfg=json.loads((directory/'config.json').read_text());s=cfg['scene'];routes=json.loads((directory.parent/'routes.json').read_text())
    actual={r['cycle']:r for r in lines(directory/'actual.jsonl')};plans=lines(directory/'plans.jsonl');messages=lines(directory/'messages.jsonl',plans)
    grouped={};ms={}
    for p in plans:grouped.setdefault(p['context']['cycle'],[]).append(p)
    for m in messages:
        if m['event']=='send' and m['kind'] in ('snapshot','state'):ms.setdefault(m['cycle'],[]).append(m)
    agents={c:Agent(c,routes[c],s) for c in CARS};last={};prior={};snapshot=None
    for cycle in range(44):
        group=grouped[cycle];ctx=group[0]['context'];snapshots={m['sender']:m['payload'] for m in ms[cycle] if m['kind']=='snapshot'};active={m['sender']:m['payload'] for m in ms[cycle] if m['kind']=='state' and not m['payload']['context']['conditional']}
        for c in CARS:
            w=copy.deepcopy(agents[c]);w.advance(snapshots[c]['context']['owner']==c,{p:r for p,r in snapshots.items() if p!=c})
            if ctx['conditional']:w=copy.deepcopy(agents[c]);w.advance(ctx['owner']==c,{p:r for p,r in active.items() if p!=c})
            parameters=w.parameters(ctx['owner']==c)
            if cycle==43 and c=='A1':
                newctx={**ctx,'coordination_round':1};report=public_report(w,newctx);p,r=build_start(w,parameters,report,newctx,Identity(),s,last.get(c),prior.get(c),{})
                minimum=.5*s['h']*abs(float(w.z[3]));remaining=w.segment['length']-w.s
                snapshot=plain(dict(cycle=cycle,car=c,actual_state=w.z,progress_m=w.s,segment=w.segment,steering_queue=w.steering,parameters=parameters,context=ctx,previous_committed_plan=last.get(c),new_candidate=p,new_candidate_validation=r,necessary_one_step_nonreverse_distance_m=minimum,remaining_distance_m=remaining,necessary_stop_slack_m=remaining-minimum,interpretation='partial candidate checks can pass within tolerance; this is not a certificate for the stricter full bicycle prefix/terminal intersection'))
            if cycle in actual:
                a=actual[cycle];prior[c]=contract(w,parameters,ctx['owner']==c);commit_agent(w,[a['controls'][c]],a['states'][c]);agents[c]=w;last[c]=next(p for p in group if p['id']==a['plans'][c])
    diag=expand(next(r for r in lines(directory/'diagnostics.jsonl') if r['cycle']==43),object_loader(old));q=diag['local']['A1'];solved=[]
    for layer,detail in [('reference',q['reference_record']['_details']),('bicycle',q['subproblems'][0]['_details'])]:
        H=np.array(detail['H']);solver=osqp.OSQP();solver.setup(P=csc_matrix(H),q=np.array(detail['q']),A=csc_matrix(detail['A']),l=np.array(detail['lower']),u=np.array(detail['upper']),eps_abs=1e-8,eps_rel=1e-8,max_iter=20000,verbose=False,polishing=True)
        result=solver.solve(raise_error=False);solved.append(dict(layer=layer,status=result.info.status,iterations=result.info.iter,note='independent raw-unit diagnostic solve of frozen assembled matrix, not online result or equivalent timing benchmark'))
    snapshot['frozen_matrix_diagnostic']=solved;snapshot['original_failure']=dict(reference=q['reference_record']['message'],bicycle=q['subproblems'][0]['message']);snapshot['debug_v2_missing']='source version and QP matrices absent; no full reconstruction claimed'
    out=root/'reconstruction';write(out/'debug_v1_stop.json',snapshot)
    with gzip.open(out/'debug_v1_frozen_diagnostic.json.gz','wt') as f:json.dump(diag,f,separators=(',',':'))
    print(json.dumps(dict(necessary_stop_slack_m=snapshot['necessary_stop_slack_m'],frozen_matrix_diagnostic=solved)),flush=True)


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();run(a.root)
