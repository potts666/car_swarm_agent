#!/usr/bin/env python3
"""Offline independent dynamics and event/version reconstruction for M2a-release-v1."""
import argparse
import csv
import json
from pathlib import Path
import time
import numpy as np
from bicycle_core import CARS,SIGN,corners,clipped_area,audit,completed
from verify_entry_bicycle import replay,check_series
from m2a_protocol import canonical,plan_id
from run_entry_bicycle import write


def lines(p):return [json.loads(line) for line in p.read_text().splitlines()]


def verify(directory):
    begun=time.perf_counter();cfg=json.loads((directory/'config.json').read_text());scene=cfg['scene'];summary=json.loads((directory/'summary.json').read_text())
    records=lines(directory/'solves.jsonl');plans=lines(directory/'plans.jsonl');agreements=lines(directory/'agreements.jsonl');events=lines(directory/'events.jsonl');messages=lines(directory/'messages.jsonl')
    timing=json.loads((directory/'planning.json').read_text());rows=list(csv.DictReader((directory/'actual.csv').open()));n=round(scene['h']/scene['sample_s']);tol=scene['tolerance']
    actual={};reintegrated={};equations={};errors={}
    for c in CARS:
        rr=[r for r in rows if r['car']==c];actual[c]=rr;z=np.array([[float(r[k]) for k in ('x','y','psi','v','delta')] for r in rr])
        inputs=[[float(r['a']),float(r['omega'])] for r in rr[1:]]
        reintegrated[c]=replay(np.array(scene['initial'][c],float),inputs,scene['sample_s'],scene['vehicle'])
        equations[c]=check_series(rr,scene) if len(rr)>1 else dict(passed=True,gear_sign_jump=False,residuals={})
        errors[c]=float(np.max(abs(reintegrated[c]-z)))
    geometry=audit(reintegrated,scene);goals={c:completed(reintegrated[c][-1],scene['goals'][c],scene) for c in CARS}
    plan_map={p['id']:p for p in plans};violations=[]
    def require(condition,message):
        if not condition:violations.append(message)
    for p in plans:
        require(p['id']==plan_id(p),'plan content/ID mismatch')
        require(set(p)=={'car','context','report','commands','states','kind','revision','origin','shifted_from','id'},'private information in public plan')
    for q in plans:
        if q['kind']!='seed':continue
        cycle=q['context']['cycle'];car=q['car']
        if q['shifted_from'] is None:
            require(cycle==0 and abs(q['report']['z'][3])<=1e-12,'invalid static initialization')
        else:
            previous=plan_map.get(q['shifted_from'])
            require(previous is not None and previous['car']==car and previous['kind']=='final' and previous['context']['cycle']==cycle-1,'stale seed source')
            if previous:
                require(np.max(abs(np.array(previous['states'])[n]-q['report']['z']))<=1e-12,'shifted initial-state mismatch')
                expected=previous['commands'][1:]+[[0.,0.,'SHIFT_TAIL']]
                require(q['commands']==expected,'seed not shifted/padded committed input')
                require(any(a['cycle']==cycle-1 and a['plans'][car]==previous['id'] for a in agreements),'seed from uncommitted plan')
    sends=[m for m in messages if m['event']=='send'];receives=[m for m in messages if m['event']=='receive']
    for m in sends:
        envelope={k:m[k] for k in ('sender','receiver','kind','cycle','sequence','payload')}
        require(len(canonical(envelope).encode())==m['application_bytes'],'application byte mismatch')
        require(sum(r['sequence']==m['sequence'] and r['receiver']==m['receiver'] for r in receives)==1,'delivery not exactly once')
        if m['kind'] in ('final','seed'):require(m['payload']['id'] in plan_map and m['payload']==plan_map[m['payload']['id']],'publication/wire mismatch')
    owner=None;version=0;released=set();pending={};committed_cycles=set();full_prediction_error=0.;minimum_prediction_gap=None
    protection=[-scene['admission_protection_abs_x'],scene['admission_protection_abs_x'],-scene['W']/2,scene['W']/2]
    def full_clear(c,z):return float(np.min(SIGN[c]*corners([z],scene['vehicle'])[0,:,0]))>=scene['release_plane_abs_x']+scene['release_margin_m']-tol
    def state(c,cycle):return reintegrated[c][min(cycle*n,len(reintegrated[c])-1)]
    agreement_by_cycle={a['cycle']:a for a in agreements}
    for p in timing:
        cycle=p['cycle'];ev=[e for e in events if e['cycle']==cycle];ms=[m for m in sends if m['cycle']==cycle]
        for e in ev:
            if e['kind']=='request':
                require(e['car'] not in pending and e['car'] not in released,'duplicate/released request');pending[e['car']]=cycle
                require(SIGN[e['car']]*state(e['car'],cycle)[0]>=-scene['request_rear_axle_abs_x']-tol,'premature request')
            if e['kind']=='grant':
                require(owner is None and not released,'unexpected initial grant')
                require(bool(pending) and e['car']==min(pending,key=lambda c:(pending[c],c)),'request ordering mismatch')
                owner=e['car'];pending.pop(owner,None);version+=1;require(e['version']==version,'initial authorization version mismatch')
            if e['kind']=='handoff':
                old=e['old_owner'];require(old==owner and old not in released,'nonowner/repeated handoff')
                require(full_clear(old,state(old,cycle)),'actual rotated body not clear')
                report=[m for m in ms if m['kind']=='release_report' and m['sender']==old]
                require(len(report)==1 and report[0]['payload']['authorization_version']==version,'stale/missing release report')
                prepared=[x for x in ev if x['kind']=='release_prepared'];require(len(prepared)==1,'missing prepared transaction')
                if old=='A':
                    ready=[m['payload'] for m in ms if m['kind']=='state' and m['receiver']=='channel' and m['sender']=='B' and m['payload']['context']['permission_version']==version]
                    require(len(ready)==1,'missing active-version readiness evidence')
                    if ready:
                        r=ready[0];z=state('B',cycle)
                        require(r['mode']=='WAIT' and r['index']==3 and r['ready'] and r['steering_complete'] and abs(z[3])<=1e-12 and abs(z[4])<1e-7,'moving/unready next owner')
                        require(np.max(abs(z-np.array(r['z'])))<=tol,'ready state not actual snapshot')
                        require(clipped_area(corners([z],scene['vehicle'])[0],protection)<=tol,'ready vehicle inside protection')
                require(e['owner'] in pending if e['owner'] is not None else not pending,'next owner not pending')
                owner=e['owner'];version+=1;require(e['version']==version,'handoff version mismatch');released.add(old)
                if owner is not None:pending.pop(owner,None)
        if not p['committed']:
            require(cycle not in agreement_by_cycle,'agreement without actual commit');continue
        require(cycle in agreement_by_cycle,'commit without version agreement')
        if cycle not in agreement_by_cycle:continue
        a=agreement_by_cycle[cycle];committed_cycles.add(cycle);packet=a['packet'];ctx=packet['token']['context'];pair=a['plans']
        require(packet['effective']['owner']==owner and packet['effective']['permission_version']==version,'executed permission mismatch')
        require(ctx['cycle']==cycle and ctx['permission_version']==version and ctx['owner']==owner,'stale plan permission/state cycle')
        require(abs(ctx['state_time_s']-(scene['epoch_s']+cycle*scene['h']))<1e-9,'state timestamp mismatch')
        if ctx['conditional']:
            require(packet['activated_proposal_id']==ctx['proposal_id'],'conditional execution before activation')
            require(any(e['kind']=='handoff' and e['proposal_id']==ctx['proposal_id'] and e['plans']==pair for e in ev),'no handoff for acknowledged pair')
        for c in CARS:
            ack=a['acks'][c];require(ack['plans']==pair and ack['context']==ctx and ack['accepted'],'stale or wrong plan acknowledgement')
            wire_ack=[m for m in ms if m['kind']=='ack' and m['sender']==c and m['payload']==ack]
            wire_commit=[m for m in ms if m['kind']=='commit' and m['receiver']==c and m['payload']==packet]
            require(len(wire_ack)==len(wire_commit)==1,'ack/commit wire identity missing')
            if wire_ack and wire_commit:require(wire_ack[0]['sequence']<wire_commit[0]['sequence'],'execution before acknowledgment')
        predicted={}
        for c in CARS:
            q=plan_map[pair[c]];require(q['context']==ctx and q['kind']=='final','mixed/nonfinal commit')
            require(np.max(abs(np.array(q['report']['z'])-state(c,cycle)))<=tol,'plan not from actual state')
            require(q['report']['context']==ctx,'phase report version mismatch')
            commands=np.array(q['commands'],dtype=object)[:,:2].astype(float)
            require(commands.shape==(scene['N'],2) and np.isfinite(commands).all(),'bad public final inputs')
            require(commands[:,0].min()>=scene['vehicle']['a_min']-tol and commands[:,0].max()<=scene['vehicle']['a_max']+tol and np.max(abs(commands[:,1]))<=scene['vehicle']['omega_max']+tol,'predicted actuator bound')
            predicted[c]=replay(state(c,cycle),np.repeat(commands,n,axis=0),scene['sample_s'],scene['vehicle'])
            full_prediction_error=max(full_prediction_error,float(np.max(abs(predicted[c]-np.array(q['states'])))))
            for i,r in enumerate(actual[c][cycle*n+1:(cycle+1)*n+1],1):
                require(abs(float(r['a'])-commands[0,0])<=1e-12 and abs(float(r['omega'])-commands[0,1])<=1e-12,'actual input not final acknowledged version')
                require(r['owner']==(owner or ''),'actual interval owner mismatch')
            body=corners(predicted[c],scene['vehicle'])
            if c!=owner:require(all(clipped_area(b,protection)<=tol for b in body),'unauthorized prediction')
            if c in released:require(all(full_clear(c,z) for z in predicted[c]),'released prediction reentry')
        check=audit(predicted,scene);require(check['passed'],'final pair physically incompatible')
        if check['minimum_gap_m'] is not None:minimum_prediction_gap=check['minimum_gap_m'] if minimum_prediction_gap is None else min(minimum_prediction_gap,check['minimum_gap_m'])
    require(full_prediction_error<=tol,'independent final prediction replay mismatch')
    counts=all(len(actual[c])==1+len(committed_cycles)*n for c in CARS)
    timed=all(p['end_monotonic_s']>=p['start_monotonic_s'] and abs(p['total_cycle_wall_s']-(p['end_monotonic_s']-p['start_monotonic_s']))<1e-9 for p in timing)
    require(counts,'committed sample count');require(timed and len(timing)==len(records),'all-exit timing missing')
    local_boundary=True;dimensions={c:dict(reference_max=0,bicycle_max=0) for c in CARS}
    if cfg['arm']=='L-new':
        require(summary['online_joint_optimizer_forbidden'],'joint-call guard absent')
        for record in records:
            for c,local in (record['local'] or {}).items():
                if local is None:continue
                peer=plan_map.get(local['peer_plan_id'])
                require(peer is not None and peer['context']==record['context'],'stale optimizer peer message')
                if peer:require(peer['kind']==('seed' if c=='A' else 'final'),'undeclared candidate exchange order')
                dimensions[c]['reference_max']=max(dimensions[c]['reference_max'],local['reference_record']['variables'])
                dimensions[c]['bicycle_max']=max([dimensions[c]['bicycle_max']]+[s['variables'] for s in local['subproblems']])
                local_boundary &= local['reference_record']['variables']<=scene['N'] and all(s['variables']<=2*scene['N'] for s in local['subproblems'])
        local_boundary &= all(p['origin']=='local' for p in plans if p['kind']=='final')
        require(local_boundary,'local dimension/origin boundary failed')
    else:local_boundary=None
    physical=bool(geometry['passed'] and all(q['passed'] for q in equations.values()) and max(errors.values())<=tol)
    prefix=bool(physical and not violations)
    result=dict(actual_full_acceptance=bool(prefix and summary['both_completed'] and all(goals.values())),actual_prefix_acceptance=prefix,
        both_goals_reached=goals,physical_and_original_sign_check_passed=physical,release_and_plan_version_contract_passed=not violations,
        protocol_violations=violations,component_equations=equations,accumulated_replay_error=errors,trajectory=geometry,
        maximum_final_prediction_replay_error=full_prediction_error,minimum_final_prediction_gap_m=minimum_prediction_gap,
        timing_complete=bool(timed and len(timing)==len(records)),local_information_boundary_passed=local_boundary,local_decision_dimensions=dimensions,
        independent_audit_wall_s=time.perf_counter()-begun,geometry_verifier_shared=True,contract='M2a-release-v1',old_full_future_contract_not_claimed=True,
        centralized_future_diagnostic='separate offline file, not part of actual acceptance')
    write(directory/'independent_validation.json',result)
    print(directory.name,json.dumps({k:result[k] for k in ('actual_full_acceptance','actual_prefix_acceptance','physical_and_original_sign_check_passed','release_and_plan_version_contract_passed','local_information_boundary_passed')}),flush=True)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True);args=parser.parse_args();verify(args.run)
