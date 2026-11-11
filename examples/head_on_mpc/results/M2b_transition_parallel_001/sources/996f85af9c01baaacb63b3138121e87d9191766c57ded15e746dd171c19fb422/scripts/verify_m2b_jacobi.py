"""Additional offline sealed-round and independently restored phase checks."""
import copy
from collections import defaultdict
import numpy as np
from m2b_identity import Identity
from m2b_scene import Agent,CARS
from m2b_protocol import public_report,canonical
from m2b_jacobi import sha,base
from m2b_candidates import contract,compatible_reference
from m2b_control import commit_agent
from m2b_recording import lines


def extra_verify(directory,cfg,routes,plans,messages,actual,solves,agreements):
    identity=Identity(cfg['identity_mapping']);cars=identity.cars;s=copy.deepcopy(cfg['scene']);violations=[]
    def need(ok,msg):
        if not ok and msg not in violations:violations.append(msg)
    for key in ('initial','goals'):s[key]={identity.roles[c]:z for c,z in s[key].items()}
    agents={c:Agent(identity.roles[c],routes[c],s) for c in cars};previous_contract={};last={};by_cycle=defaultdict(list);by_id={p['id']:p for p in plans}
    for p in plans:by_cycle[p['context']['cycle']].append(p)
    sends=defaultdict(list)
    for m in messages:
        if m['event']=='send':sends[m['cycle']].append(m)
    actual_by={a['cycle']:a for a in actual};max_fixed=0.;max_progress=0.;candidate_count=0
    for cycle,group in sorted(by_cycle.items()):
        ms=sends[cycle];seeds={p['car']:p for p in group if p['kind']=='seed'}
        if not seeds:continue
        need(set(seeds)==set(cars),'incomplete start candidates');ctx=base(next(iter(seeds.values()))['context'])
        snapshots={m['sender']:m['payload'] for m in ms if m['kind']=='snapshot'}
        active={m['sender']:m['payload'] for m in ms if m['kind']=='state' and not m['payload']['context']['conditional'] and 'coordination_round' not in m['payload']['context']}
        work={};contracts={}
        for c,p in seeds.items():
            role=identity.roles[c];w=copy.deepcopy(agents[c]);peer={identity.roles[k]:identity.report_to_role(r) for k,r in snapshots.items() if k!=c}
            w.advance(snapshots[c]['context']['owner']==c,peer)
            if ctx['conditional']:
                w=copy.deepcopy(agents[c]);w.advance(ctx['owner']==c,{identity.roles[k]:identity.report_to_role(r) for k,r in active.items() if k!=c})
            par=w.parameters(ctx['owner']==c);expected=contract(w,par,ctx['owner']==c);contracts[c]=expected;work[c]=w
            reported=public_report(w,identity.context(p['context'],False));reported.update(car=c,context=p['context'])
            need(reported==p['report'],'independent phase/queue restoration failed');need(p['applicability']==expected,'candidate applicability identity failed')
            if w.index>agents[c].index:
                endpoint=agents[c].segment['end'];yaw=np.arctan2(np.sin(w.z[2]-endpoint[2]),np.cos(w.z[2]-endpoint[2]))
                need(np.linalg.norm(w.z[:2]-endpoint[:2])<=.005 and abs(yaw)<=.003,'original terminal pose transition guard failed')
            u=np.array([q[:2] for q in p['commands']]);candidate_count+=1
            if w.mode!='MOVE':
                q=w.steering[:s['N']]+[(0.,0.,w.mode)]*max(0,s['N']-len(w.steering));delta=float(np.max(abs(u-np.array([v[:2] for v in q]))));max_fixed=max(max_fixed,delta);need(delta<=s['tolerance'],'candidate violates independently restored fixed queue')
            else:
                velocity=float(w.z[3]);position=float(w.s);positions=[position];velocities=[velocity]
                for a,_ in u:
                    position+=s['h']*velocity+.5*s['h']**2*a;velocity+=s['h']*a;positions.append(position);velocities.append(velocity)
                K=par['stop_index'];err=max(0.,-min(velocities),max(velocities)-par['cap'],-min(positions),max(positions)-par['upper'],abs(velocities[K]))
                if par['stop_at_end']:err=max(err,abs(positions[K]-par['upper']))
                max_progress=max(max_progress,float(err));need(err<=s['tolerance'],'candidate partial reference rows failed')
            if p['origin']=='committed_shift':need(compatible_reference(previous_contract.get(c),expected),'shift reused across incompatible phase/reference')
        if cycle in actual_by:
            a=actual_by[cycle]
            for c,w in work.items():commit_agent(w,[a['controls'][c]],a['states'][c]);agents[c]=w;previous_contract[c]=contracts[c];last[c]=a['plans'][c]
    rounds=lines(directory/'rounds.jsonl');overlap=[];pids=set()
    for r in rounds:
        expected=sha(dict(context=r['context'],plans=r['input_ids']));need(expected==r['input_set_id'],'sealed input-set hash mismatch');need(set(r['input_ids'])==set(cars),'sealed input set missing member')
        spans=[]
        for c,v in r['workers'].items():
            pids.add(v['pid']);spans.append((v['compute_start_ns'],v['compute_end_ns']))
            need(v['input_set_id']==expected,'worker sealed manifest mismatch')
            own=by_id.get(r['input_ids'][c]);need(own is not None,'sealed own candidate missing')
            for other,pid in r['input_ids'].items():
                if other==c:continue
                need(any(m['sender']==other and m['receiver']==c and m['kind'] in ('seed','final') and m['payload']['id']==pid for m in sends[r['cycle']]),'sealed input lacks DDS provenance')
        overlap.append(max(0.,(min(b for a,b in spans)-max(a for a,b in spans))/1e9))
    for row in solves:
        cohort=row.get('local') or {}
        for c,q in cohort.items():
            ctx=q['round_context'];manifest=q['round_input_ids'];expected=sha(dict(context=ctx,plans=manifest));need(q['input_set_id']==expected,'local optimizer manifest mismatch')
            need(q['peer_plan_ids']=={k:v for k,v in manifest.items() if k!=c},'optimizer reads foreign/unsealed peer')
            for p in by_cycle[row['cycle']]:
                if p['kind']=='final' and p['car']==c and p['context']==ctx:need(p['input_set_id']==expected,'final not bound to optimizer input set')
    for a in agreements:
        token=a['packet']['token'];need(all(ack.get('input_set_id')==next(iter(a['acks'].values())).get('input_set_id') for ack in a['acks'].values()),'ACK input-set mismatch')
        need('coordination_round' in token['context'],'missing commit round')
    need(len(pids)==4 or not rounds,'not four persistent computation workers')
    return dict(violations=violations,candidate_count=candidate_count,max_fixed_input_error=max_fixed,max_partial_progress_speed_error=max_progress,full_ocp_candidate_certificate=False,worker_pids=sorted(pids),four_way_overlap_cycles=sum(v>0 for v in overlap),max_four_way_overlap_s=max(overlap,default=0.),round_count=len(rounds),independent_queue_restoration=True)
