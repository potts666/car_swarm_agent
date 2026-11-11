#!/usr/bin/env python3
"""Independent scalar replay, six-pair geometry and four-version protocol reconstruction."""
import argparse
from collections import Counter,defaultdict
import itertools
import json
import math
from pathlib import Path
import time
import numpy as np
from bicycle_core import corners,clipped_area,completed
from verify_entry_bicycle import replay
from m2a_logs import lines,plan_id
from m2a_protocol import canonical
from m2b_scene import CARS,ORDER,SIGMA,PREDECESSOR,audit
from linearized_joint import ERROR_LIMIT
from run_entry_bicycle import write


def verify_run(directory,output=None):
    started=time.perf_counter();cfg=json.loads((directory/'config.json').read_text());s=cfg['scene'];summary=json.loads((directory/'summary.json').read_text());N=s['N'];h=s['h'];ds=s['sample_s'];n=round(h/ds);tol=s['tolerance']
    routes=json.loads((directory.parent/'routes.json').read_text());head_seen=set()
    actual=lines(directory/'actual.jsonl');plans=lines(directory/'plans.jsonl');messages=lines(directory/'messages.jsonl');events=lines(directory/'events.jsonl');solves=lines(directory/'solves.jsonl');timing=lines(directory/'planning.jsonl');agreements=lines(directory/'agreements.jsonl')
    violations=[]
    def need(ok,message):
        if not ok and message not in violations:violations.append(message)
    by_id={p['id']:p for p in plans};need(len(by_id)==len(plans),'duplicate plan ID');by_cycle={a['cycle']:a for a in agreements};executed={a['cycle']:a for a in actual}
    states={c:[np.array(s['initial'][c],float)] for c in CARS};controls={c:[] for c in CARS};component_error={c:0. for c in CARS};accumulated={c:0. for c in CARS};at={c:{0:np.array(s['initial'][c],float)} for c in CARS}
    for index,a in enumerate(actual):
        need(a['cycle']==index,'actual cycles not contiguous');need(set(a['states'])==set(CARS),'missing actual vehicle')
        for c in CARS:
            z=np.asarray(a['states'][c]);u=np.array(a['controls'][c][:2],float);need(z.shape==(n+1,5),'wrong actual sampling');need(np.max(abs(z[0]-states[c][-1]))<=tol,'actual state reset')
            predicted=replay(states[c][-1],np.tile(u,(n,1)),ds,s['vehicle']);component_error[c]=max(component_error[c],float(np.max(abs(predicted-z))))
            for q,r in zip(z[:-1],z[1:]):component_error[c]=max(component_error[c],float(np.max(abs(replay(q,[u],ds,s['vehicle'])[-1]-r))))
            states[c].extend(z[1:]);controls[c].extend([u]*n);at[c][index+1]=z[-1]
            need(s['vehicle']['a_min']-tol<=u[0]<=s['vehicle']['a_max']+tol and abs(u[1])<=s['vehicle']['omega_max']+tol,'actual actuator limit')
    for c in CARS:
        states[c]=np.asarray(states[c]);acc=replay(np.array(s['initial'][c],float),controls[c],ds,s['vehicle']);accumulated[c]=float(np.max(abs(acc-states[c])))
        need(component_error[c]<=tol and accumulated[c]<=tol,'component/accumulated dynamics failed')
        need(not np.any(states[c][:-1,3]*states[c][1:,3]<-tol**2),'original speed sign crossing')
    physical=audit(states,s);need(physical['passed'],'actual physical geometry/bounds failed');need(len(physical['pair_gaps_m'])==6,'missing actual pair')
    goals={c:completed(states[c][-1],s['goals'][c],s) for c in CARS}
    receives=Counter((m['sequence'],m['receiver']) for m in messages if m['event']=='receive');sends=[m for m in messages if m['event']=='send'];need(len({m['sequence'] for m in sends})==len(sends),'duplicate send sequence')
    grouped=defaultdict(list)
    for m in sends:
        grouped[m['cycle']].append(m);envelope={k:m[k] for k in ('sender','receiver','kind','cycle','sequence','payload')}
        need(len(canonical(envelope).encode())==m['application_bytes'],'restored application byte mismatch');need(receives[m['sequence'],m['receiver']]==1,'delivery not exactly once')
        if m['kind'] in ('seed','final'):need(m['payload']['id'] in by_id and m['payload']==by_id[m['payload']['id']],'message/plan content mismatch')
    need(len(receives)==len(sends),'orphan receive')
    prediction_error=0.;prediction_gap=None;final_states={};public_fields={'car','context','report','commands','states','kind','origin','shifted_from','id'}
    for p in plans:
        c=p['car'];cycle=p['context']['cycle'];need(c in CARS and p['id']==plan_id(p),'invalid plan ID');need(set(p)==public_fields,'private/unknown public plan fields');need(p['report']['context']==p['context'],'plan/state context mismatch')
        need(cycle in at[c] and np.max(abs(np.array(p['states'][0])-at[c].get(cycle,np.full(5,np.inf))))<=tol,'prediction not at actual state')
        need(len(p['commands'])==N and np.asarray(p['states']).shape==(N*n+1,5),'plan horizon mismatch')
        if p['kind']=='seed':
            previous=by_id.get(p['shifted_from'])
            if previous is None:need(cycle==0 and p['shifted_from'] is None and abs(p['report']['z'][3])<=1e-12,'invalid initialization')
            else:
                need(previous['car']==c and previous['kind']=='final' and previous['context']['cycle']==cycle-1,'invalid shifted source')
                need(p['commands']==previous['commands'][1:]+[[0.,0.,'SHIFT_TAIL']],'seed not shifted/padded')
                need(cycle-1 in by_cycle and by_cycle[cycle-1]['plans'].get(c)==previous['id'],'seed from uncommitted plan')
                need(np.max(abs(np.array(previous['states'][n])-p['report']['z']))<=1e-12,'shift actual state mismatch')
        elif p['kind']=='final':
            u=np.repeat(np.array([v[:2] for v in p['commands']],float),n,axis=0);z=replay(np.array(p['report']['z'],float),u,ds,s['vehicle']);prediction_error=max(prediction_error,float(np.max(abs(z-np.array(p['states'])))))
            need(u[:,0].max()<=1+tol and u[:,0].min()>=-1-tol and abs(u[:,1]).max()<=1+tol,'predicted actuator limit')
            need(audit({c:z},s)['passed'],'predicted own physics failed');final_states[p['id']]=z
        else:need(False,'unknown plan kind')
    need(prediction_error<=tol,'independent prediction replay error')
    owner=None;version=0;served=set();released=set();requests={};last_direction=None;handoffs=[];ready_first={};entry_first={};clear_first={};idle=Counter();waits={};local_boundary=True;dimensions={c:dict(reference=0,bicycle=0) for c in CARS};last_reference={}
    event_by_cycle=defaultdict(list)
    for e in events:event_by_cycle[e['cycle']].append(e)
    for row in timing:
        cycle=row['cycle'];ms=grouped[cycle];ev=event_by_cycle[cycle];actual_now={c:at[c].get(cycle,states[c][-1]) for c in CARS}
        active_reports={m['sender']:m['payload'] for m in ms if m['kind']=='state' and m['receiver']=='channel' and m['payload']['context']['permission_version']==version and not m['payload']['context']['conditional']}
        for c,z in actual_now.items():
            head=np.array(s['queue_heads'][c[0]])
            if np.linalg.norm(z[:2]-head)<=.005 and abs(z[3])<=1e-12:head_seen.add(c)
        for m in ms:
            if m['kind'] in ('state','snapshot'):
                r=m['payload'];need(r['car']==m['sender'] and r['context']['cycle']==cycle and np.max(abs(np.array(r['z'])-actual_now[r['car']]))<=tol,'state message provenance failed')
                c=r['car'];need(set(r)=={'car','z','mode','index','progress_m','queue_arrived','left_queue','steering_complete','context'},'private/unknown state message field')
                if r['queue_arrived']:need(c in head_seen,'queue-arrival report without actual head stop')
                if r['left_queue']:need(c in head_seen and float(np.min(SIGMA[c]*corners([actual_now[c]],s['vehicle'])[0,:,0]))>=-16-tol,'false queue-clear report')
                pred=PREDECESSOR[c]
                if pred and r['mode']=='MOVE':
                    segment=routes[c]['segments'][min(r['index'],len(routes[c]['segments'])-1)]
                    if segment['stage']=='APPROACH_MERGE':need(pred in head_seen,'follower starts merge before leader ready')
                    if segment['stage']=='QUEUE_ADVANCE':need(float(np.min(SIGMA[pred]*corners([actual_now[pred]],s['vehicle'])[0,:,0]))>=-16-tol,'follower advances before actual queue clearance')
        for e in ev:
            if e['kind']=='request':
                c=e['car'];need(c not in requests and SIGMA[c]*actual_now[c][0]>=-34-1e-8,'invalid/repeated request');requests[c]=cycle
        def ready(c,r):
            z=actual_now[c];head=np.array(s['queue_heads'][c[0]]);yaw=math.atan2(math.sin(z[2]-(0 if SIGMA[c]==1 else math.pi)),math.cos(z[2]-(0 if SIGMA[c]==1 else math.pi)))
            return r.get('mode')=='READY' and r.get('steering_complete') and abs(z[3])<=1e-12 and abs(z[4])<=1e-7 and np.linalg.norm(z[:2]-head)<=.005 and abs(yaw)<=.003 and clipped_area(corners([z],s['vehicle'])[0],[-18,18,-1.4,1.4])<=tol
        eligible=[]
        for side in ('A','B'):
            head=next((c for c in CARS if c[0]==side and c not in served),None)
            if head in requests and head in active_reports and ready(head,active_reports[head]):eligible.append(head);ready_first.setdefault(head,cycle*h)
        for e in ev:
            if e['kind']=='handoff':
                need(e['old_owner']==owner and e['version']==version+1,'invalid owner/version handoff')
                if owner:need(float(np.min(SIGMA[owner]*corners([actual_now[owner]],s['vehicle'])[0,:,0]))>=26.5-tol,'actual rotated body not clear at release');released.add(owner)
                choices=eligible[:]
                if cfg['policy']=='alternating-ready' and last_direction is not None:
                    opposite=[c for c in choices if SIGMA[c]!=last_direction]
                    if opposite:choices=opposite
                expected=min(choices,key=lambda c:(requests[c],CARS.index(c))) if choices else None
                need(e['owner']==expected and set(e['eligible'])==set(eligible),'policy/queue eligibility mismatch');need(owner is not None or expected is not None,'spurious idle handoff')
                owner=e['owner'];version=e['version'];handoffs.append(e)
                if owner:
                    served.add(owner);last_direction=SIGMA[owner];waits[owner]=dict(request_to_grant_s=cycle*h-requests[owner]*h,ready_to_grant_s=cycle*h-ready_first[owner],grant_s=cycle*h)
                need(cycle in by_cycle and by_cycle[cycle]['plans']==e['plans'],'handoff lacks final acknowledgements')
        if not owner and len(served)<4:
            idle['no_candidate' if not eligible and not any(c not in served for c in requests) else 'candidates_not_ready' if not eligible else 'control_or_handoff']+=h
        for c,z in actual_now.items():
            if float(np.min(SIGMA[c]*corners([z],s['vehicle'])[0,:,0]))>=26.5-tol:clear_first.setdefault(c,cycle*h)
        if row['committed']:
            need(cycle in executed and cycle in by_cycle,'committed input/ack missing')
            if cycle not in executed or cycle not in by_cycle:continue
            a=executed[cycle];agreement=by_cycle[cycle];ctx=agreement['packet']['token']['context'];pair=agreement['plans'];effective=agreement['packet']['effective'];need(set(pair)==set(CARS),'missing member of final combination')
            need(a['owner']==owner==effective['owner'] and a['permission_version']==version==effective['permission_version'],'actual effective permission mismatch');need(pair==a['plans']==agreement['packet']['token']['plans'],'mixed executed versions');need(ctx['cycle']==cycle and ctx['owner']==owner and ctx['permission_version']==version,'stale commit context')
            if ctx['conditional']:need(agreement['packet']['activated_proposal_id']==ctx['proposal_id'],'conditional input executed before activation')
            joint={}
            for c in CARS:
                p=by_id.get(pair[c]);need(p is not None and p['kind']=='final' and p['context']==ctx,'stale final member')
                if p is None:continue
                joint[c]=np.array(p['states']);need(a['controls'][c]==p['commands'][0] and np.max(abs(np.array(a['states'][c])-joint[c][:n+1]))<=1e-12,'actual input does not match final prefix')
                ack=agreement['acks'].get(c);need(ack is not None and ack['plans']==pair and ack['context']==ctx and ack['accepted'],'missing/mixed/expired acknowledgement')
                need(any(m['kind']=='ack' and m['sender']==c and m['receiver']=='channel' and m['payload']==ack for m in ms),'ack not delivered')
                need(any(m['kind']=='commit' and m['sender']=='channel' and m['receiver']==c and m['payload']==agreement['packet'] for m in ms),'commit not delivered')
                need(sum(m['kind']=='final' and m['sender']==c and m['payload']['id']==p['id'] for m in ms)==3,'final not exchanged to all three peers')
                body=corners(joint[c],s['vehicle'])
                if owner!=c:need(all(clipped_area(b,[-18,18,-1.4,1.4])<=tol for b in body),'unauthorized predicted protection entry')
                if c in released:need(float(np.min(SIGMA[c]*body[:,:,0]))>=26.5-tol,'released future reentry')
                actual_body=corners(np.array(a['states'][c]),s['vehicle'])
                occupied=[j for j,b in enumerate(actual_body) if clipped_area(b,[-18,18,-1.4,1.4])>tol]
                if occupied:need(owner==c,'actual unauthorized protection occupancy');entry_first.setdefault(c,cycle*h+occupied[0]*ds)
            check=audit(joint,s);need(check['passed'] and len(check['pair_gaps_m'])==6,'six-pair final compatibility failed');prediction_gap=min(prediction_gap,check['minimum_gap_m']) if prediction_gap is not None else check['minimum_gap_m']
        else:need(cycle not in executed,'input executed on rejected/completion exit')
        need(row['outcome'] in ('failed','committed','completed') and row['end_monotonic_s']>=row['start_monotonic_s'] and abs(row['total_cycle_wall_s']-row['end_monotonic_s']+row['start_monotonic_s'])<1e-8,'exit timing invalid')
        need(row['application_bytes']==sum(m['application_bytes'] for m in ms) and row['send_messages']==len(ms),'cycle communication accounting mismatch')
    for r in solves:
        entries=r['local'] if cfg['arm']=='local' else {'joint':r['central']}
        for c,q in entries.items():
            if q is None:continue
            ref=q['reference_record'];need(ref['variables']<=(32 if cfg['arm']=='local' else 128),'reference variable limit')
            if r['cycle'] in executed:
                need(q['accepted'] and not q['deadline_exceeded'] and q['solve_wall_s']<=.5 and ref['accepted'],'rejected/budget-exceeded candidate executed')
                need(all(e[k]<=limit for e in q['mapping_errors'].values() for k,limit in ERROR_LIMIT.items()),'executed model error limit failed')
                need(q['subproblems'] and q['subproblems'][-1]['accepted'] and q['subproblems'][-1]['affine_violation']<=tol,'executed affine check failed')
            need(all(x['variables']<=(64 if cfg['arm']=='local' else 256) for x in q['subproblems']),'bicycle variable limit')
            if cfg['arm']=='local':
                dimensions[c]['reference']=max(dimensions[c]['reference'],ref['variables']);dimensions[c]['bicycle']=max([dimensions[c]['bicycle']]+[x['variables'] for x in q['subproblems']]);local_boundary &= set(ref['parameters'])=={c} and set(q['peer_plan_ids'])==set(CARS)-{c}
                for other,pid in q['peer_plan_ids'].items():
                    p=by_id.get(pid);need(p is not None and p['context']==r['context'],'stale/missing optimization peer');kind='final' if ORDER.index(other)<ORDER.index(c) else 'seed';need(p is not None and p['kind']==kind,'sequential exchange used undeclared version')
                need(ref['warm_start_per_car'].keys()=={c},'foreign reference cache')
    if cfg['arm']=='local':
        local_boundary &= cfg['online_joint_forbidden'] and all(p['origin']=='local' for p in plans if p['kind']=='final');need(local_boundary,'local information/dimension boundary failed')
    else:local_boundary=None
    need(cfg['online_future_calls']==0,'online central future used');need(len(timing)==len(solves)==summary['timed_cycles'],'missing failed/completion timing');need(len(actual)==len(agreements)==summary['committed_cycles'],'missing actual/commit record')
    full=bool(not violations and all(goals.values()) and summary['all_completed'])
    result=dict(actual_full_acceptance=full,actual_prefix_acceptance=not violations,violations=violations,goals=goals,physical=physical,component_replay_error=component_error,accumulated_replay_error=accumulated,original_sign_check_passed='original speed sign crossing' not in violations,prediction_replay_error=prediction_error,minimum_prediction_gap_m=prediction_gap,protocol_passed=not any(any(word in e for word in ('owner','permission','authorization','queue','policy','ack','commit','version','request','protection','handoff','reentry')) for e in violations),all_exit_timing_passed=len(timing)==len(solves),local_information_boundary_passed=local_boundary,dimensions=dimensions,handoffs=handoffs,waiting=waits,first_ready_s=ready_first,first_protection_entry_s=entry_first,first_actual_tail_clear_s=clear_first,idle_time_s=dict(idle),independent_audit_wall_s=time.perf_counter()-started,geometry_shared_with_M1=True,central_future_not_run=True)
    write(output or directory/'independent_validation.json',result);print(directory,full,'prefix',not violations,'violations',violations[:4],flush=True);return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path);a=p.parse_args()
    if (a.run/'config.json').exists():verify_run(a.run,a.output)
    else:
        for cfg in sorted(a.run.glob('*/r*/config.json')):verify_run(cfg.parent)

if __name__=='__main__':main()
