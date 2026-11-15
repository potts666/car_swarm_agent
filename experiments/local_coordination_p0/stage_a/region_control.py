"""Fixed-duration Stage A proposals; full interval acceptance, no derivative solver."""
import copy
from fractions import Fraction as F
import json
from pathlib import Path
import sys

from region_certificates import (CARS,H,content_id,controls,pose_rows,in_box,in_flow,reset_input)
from pilot_control import execute
from moving_objects import pair_partition


def seal(plan):
    plan={k:v for k,v in plan.items() if k!='id'}
    return {**plan,'id':content_id(plan)}


def groups(phase,actor):
    others=[c for c in CARS if c!=actor]
    return ([[actor,others[0]],[others[1]],[others[2]]] if phase%64==0 else
            [[actor,others[0],others[1]],[others[2]]] if phase%64==1 else [[actor],[others[0],others[1]],[others[2]]])


def receiver(cert,phase):
    end=min(phase+32,cert['ticks'])
    return dict(component='flow' if end<cert['ticks'] else 'coarse_stopped',phase=end,
                domain_object=cert['_object_id'],deadline=end-phase)


def initial_plan(data,state):
    cert=data.cert;c=cert['car']
    return seal(dict(branch='locked_inherited',object=cert['_object_id'],phase=0,
        source_history=state['history'],source_pose_Q=pose_rows(state['poses'][c]),
        v0=state['v'][c],delta0=state['delta'][c],remaining=controls(cert),
        receiver=receiver(cert,0),pending_reset=None,ancestry=[],parent=None,
        witness=data.witness_id,witness_version=data.witness['version'],
        suffix_ticks=data.suffix_ticks,budget=cert['ticks']+data.suffix_ticks))


def window(plan):return [tuple(map(F,u)) for u in plan['remaining'][:32]]+[(F(0),F(0))]*max(0,32-len(plan['remaining']))


def validate_header(data,state,plan,phase):
    cert=data.cert;c=cert['car'];r=plan['receiver']
    if seal(plan)['id']!=plan['id']:raise ValueError('plan content identity')
    if plan['object']!=cert['_object_id']:raise ValueError('wrong motion object')
    if plan['phase']!=phase or plan['source_history']!=state['history']:raise ValueError('stale source phase/history')
    if plan['source_pose_Q']!=pose_rows(state['poses'][c]) or F(plan['v0'])!=state['v'][c] or F(plan['delta0'])!=state['delta'][c]:raise ValueError('actual source state seam')
    if len(plan['remaining'])!=cert['ticks']-phase:raise ValueError('changed duration')
    if plan['witness']!=data.witness_id or plan['witness_version']!=data.witness['version'] or plan['suffix_ticks']!=data.suffix_ticks:raise ValueError('changed completion witness')
    if plan['budget']!=len(plan['remaining'])+data.suffix_ticks:raise ValueError('completion budget mismatch')
    if r['phase']!=min(phase+32,cert['ticks']):raise ValueError('wrong receiver phase')
    if r['deadline']!=min(32,cert['ticks']-phase):raise ValueError('wrong receiver deadline')
    if r['domain_object']!=cert['_object_id']:raise ValueError('wrong receiver object')
    if r['component'] not in ('box','flow','coarse_stopped'):raise ValueError('unknown receiving domain')
    return True


def validate_candidate(model,data,state,old,new,phase):
    validate_header(data,state,old,phase);validate_header(data,state,new,phase)
    if new['id']==old['id']:
        if new!=old:raise ValueError('same identity changed')
        return dict(kind='locked_inherited',zero_increment_feasible=True)
    cert=data.cert;c=cert['car'];r=new['receiver'];m=r['deadline']
    if new['branch']!='box_candidate' or r['component']!='box' or m!=32 or new['parent']!=old['id']:raise ValueError('new candidate lacks current inherited parent')
    row=data.row(r['phase']);d=row['domain'];reset=new['pending_reset']
    if r.get('row_sha256')!=content_id(row) or r.get('domain')!=d:raise ValueError('wrong or widened receiving domain')
    if reset is None or reset['phase']!=r['phase'] or reset['next_phase']!=r['phase']+1 or reset['row_sha256']!=content_id(row):raise ValueError('wrong reset phase/domain')
    u=[tuple(map(F,x)) for x in new['remaining']];base=[tuple(map(F,x)) for x in old['remaining']]
    if u[m+1:]!=base[m+1:]:raise ValueError('changed actual old suffix')
    if sum(a*a+w*w for a,w in u[:m])>sum(a*a+w*w for a,w in base[:m]):raise ValueError('prefix energy increased')
    if new['budget']>old['budget']:raise ValueError('replacement increases old budget')
    staged=copy.deepcopy(state);road=pair=100*model.Q
    for j,command in enumerate(u[:m+1]):
        if j==m:
            if not in_box(model,staged,c,row):raise ValueError('terminal outside receiving box')
            expected=reset_input(staged['v'][c],staged['delta'][c],row)
            if tuple(map(F,reset['input']))!=expected or command!=expected:raise ValueError('wrong state-dependent K')
        record=execute(model,staged,c,command,data.cfg,cert['source']>0)
        road=min(road,record['minimum_road_lower_Q']);pair=min(pair,record['minimum_pair_lower_Q'])
        if j+1==m:
            terminal=dict(pose_Q=pose_rows(staged['poses'][c]),v=str(staged['v'][c]),delta=str(staged['delta'][c]))
    if not in_flow(model,staged,cert,r['phase']+1):raise ValueError('reset did not enter inherited flow')
    return dict(kind='box_prefix_and_actual_K',terminal=terminal,reset_result=dict(pose_Q=pose_rows(staged['poses'][c]),v=str(staged['v'][c]),delta=str(staged['delta'][c])),
        prefix_ticks=m,completion_ticks=new['budget'],suffix_witness=data.witness_id,
        minimum_road_lower_Q=road,minimum_pair_lower_Q=pair,zero_increment_feasible=True)


def construct(model,data,state,old,phase):
    """Change one prefix input; physical duration and the actual suffix are retained."""
    k=phase+32;row=data.row(k);d=row['domain'];base=[tuple(map(F,u)) for u in old['remaining']]
    cap=min(F(d['speed_radius'])/H,F(d['steering_radius'])/H)/4
    for attempt in range(24):
        epsilon=cap/2**attempt
        index=31 if any(base[31]) else max(range(32),key=lambda j:sum(x*x for x in base[j]))
        a,w=base[index]
        da=-epsilon if a>0 else epsilon;dw=-epsilon if w>0 else epsilon
        new=copy.deepcopy(old);u=base.copy();u[index]=(a+da,w+dw)
        v=state['v'][data.cert['car']]+H*sum(x[0] for x in u[:32]);delta=state['delta'][data.cert['car']]+H*sum(x[1] for x in u[:32])
        u[32]=reset_input(v,delta,row)
        new.update(branch='box_candidate',remaining=u,parent=old['id'],
            receiver={**receiver(data.cert,phase), 'component':'box','row_sha256':content_id(row),'domain':d},
            pending_reset=dict(phase=k,next_phase=k+1,row_sha256=content_id(row),input=u[32]),
            ancestry=old['ancestry']+[dict(parent=old['id'],interface=content_id(row),phase=k)])
        new=seal(new)
        try:proof=validate_candidate(model,data,state,old,new,phase)
        except ValueError:continue
        if v==F(d['center_v']) or delta==F(d['center_delta']):continue
        return new,{**proof,'construction_attempts':attempt+1,'changed_input_index':index,'input_delta':[str(da),str(dw)],'optimizer_called':False,'objective':'unchanged prefix input energy'}
    raise ValueError('no constructed candidate passed within 24 bounded attempts')


def commit(model,data,state,old,proposal,phase,cycle):
    proof=validate_candidate(model,data,state,old,proposal,phase)
    c=data.cert['car'];partition=groups(phase,c)
    ctx=dict(cycle=cycle,history=state['history'],state_time_s=str(cycle*H),owner=c,
        permission_version=data.witness['version'],joint_source=data.edge['source'],joint_target=data.edge['target'],groups=partition,pair_contracts=pair_partition(partition),
        receiver=proposal['receiver'],pending_reset=proposal['pending_reset'],witness=proposal['witness'],
        witness_version=proposal['witness_version'],budget=proposal['budget'],plan_id=proposal['id'])
    ctx=json.loads(json.dumps(ctx,default=str))
    sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'examples/head_on_mpc/scripts'))
    from m2b_protocol import Barrier
    barrier=Barrier(ctx);ids={};messages=[]
    def deliver(sender,target,kind,payload):
        envelope=dict(sender=sender,target=target,kind=kind,payload=payload);raw=json.dumps(envelope,default=str,sort_keys=True)
        decoded=json.loads(raw);messages.append(dict(bytes=len(raw.encode()),sha256=content_id(decoded)))
        return decoded['payload']
    for car in CARS:
        snapshot=dict(car=car,history=state['history'],context=ctx)
        for target in [x for x in CARS if x!=car]+['coordinator']:deliver(car,target,'snapshot',snapshot)
    for car in CARS:
        public=dict(car=car,context=ctx,controls=window(proposal) if car==c else [(F(0),F(0))]*32,
            certificate=dict(plan_id=proposal['id'],witness=proposal['witness'],kind=proof['kind']))
        ids[car]=content_id(public);public.update(kind='final',id=ids[car])
        for target in [x for x in CARS if x!=car]+['coordinator']:
            received=deliver(car,target,'final',public)
            if target=='coordinator':barrier.publish(received)
    for car in CARS:
        validate_header(data,state,proposal,phase)
        barrier.accept(deliver(car,'coordinator','ack',dict(car=car,context=ctx,plans=ids,accepted=True)))
    token=barrier.token()
    for car in CARS:deliver('coordinator',car,'commit',token)
    return dict(context=ctx,plans=token['plans'],application_messages=len(messages),application_bytes=sum(x['bytes'] for x in messages),message_digest=content_id(messages),proof=proof)


def shift(data,committed,state):
    """Drop the actual first input; append the saved K instead of a reference tail."""
    new=copy.deepcopy(committed);c=data.cert['car'];phase=committed['phase']+1
    tail=tuple(map(F,committed['remaining'][32])) if len(committed['remaining'])>32 else (F(0),F(0))
    reset=committed['pending_reset'];is_K=reset is not None and reset['phase']==committed['phase']+32
    new.update(branch='locked_derived_suffix' if committed['ancestry'] else 'locked_inherited',phase=phase,parent=committed['id'],
        remaining=committed['remaining'][1:],budget=committed['budget']-1,source_history=state['history'],
        source_pose_Q=pose_rows(state['poses'][c]),v0=state['v'][c],delta0=state['delta'][c],receiver=receiver(data.cert,phase))
    if reset is not None and reset['phase']<phase:new['pending_reset']=None
    new=seal(new)
    assert window(new)==window(committed)[1:]+[tail]
    return new,dict(appended_input=list(map(str,tail)),kind='saved_state_dependent_K' if is_K else 'actual_old_suffix_or_goal_hold',parent=committed['id'],new_id=new['id'])
