"""Small certified-reference controller used only by the development pilot.

Each active car owns its 32 ZOH inputs. A rejected optimization executes the
committed branch. All three inactive cars keep their certified hold inputs.
"""
from fractions import Fraction as F
import hashlib
import importlib.util
import json
import math
import copy
from pathlib import Path

from moving_objects import CARS,content_id,pair_partition
from socp_anchor import anchor_problem,derivative_bounds

H=F(1,5);N=32
PI_LO=F('3.14159265358979323846264338327950288');PI_HI=PI_LO+F(1,10**35)


def load_model(rebuild):
    work=(Path(rebuild)/'work').resolve()
    spec=importlib.util.spec_from_file_location('pilot_interval_model',work/'M2b_首段自由度与递归可行性/route_zoh_certificate.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    assert m.H==H and m.L==F(27,10)
    return m


def raw_pose(m,rows):return tuple(m.I.raw(lo,hi) for lo,hi in rows)
def pose_rows(pose):return [[x.l,x.u] for x in pose]
def inflate(m,x,r):return x+m.I.raw(-m.I(r).u,m.I(r).u)


def initial_state(model,cfg):
    poses={}
    for c in CARS:
        z=cfg['initial'][c];psi=model.I(F(z[2]))
        if c[0]=='B':psi-=model.I.raw(model.I(PI_LO).l,model.I(PI_HI).u)
        poses[c]=(model.I(F(z[0])),model.I(F(z[1])),psi)
    return dict(poses=poses,v={c:F(0) for c in CARS},delta={c:F(0) for c in CARS},
                history=content_id(dict(initial=cfg['initial'],model='bicycle-ZOH-.2-v1')))


def point_domain(cfg,c,stage):
    if stage==0:return tuple(map(F,cfg['initial'][c][:2]))
    if stage==3:return tuple(map(F,cfg['goals'][c][:2]))
    x=F(cfg['release_plane_abs_x'])+F(cfg['release_margin_m'])+F(cfg['vehicle']['rear'])+F(cfg['s_min'])
    ys=sorted(set(F(z[1]) for z in cfg['goals'].values()),reverse=True)
    return (x if c[0]=='A' else -x),ys[stage-1]


def state_in_node(state,cfg,label,radii):
    for i,c in enumerate(CARS):
        center=point_domain(cfg,c,int(label[i]));p,t=map(F,radii[int(label[i])])
        pose=state['poses'][c]
        if sum((pose[j]-center[j]).abs_upper() for j in (0,1))>p or pose[2].abs_upper()>t:
            return False
        if state['v'][c] or state['delta'][c]:return False
    return True


def object_by_id(root,sha):
    p=Path(root)/'objects'/(sha+'.json')
    if hashlib.sha256(p.read_bytes()).hexdigest()!=sha:raise ValueError('candidate content hash changed')
    return {**json.loads(p.read_text()),'_object_id':sha}


def root_margin(cert,source):
    Q=cert['scale'];actor=cert['car']
    values=[F(cert['road_lower_Q'],Q)]
    values += [F(cert['peer_gap_lower_Q'][c+':'+source[i]],Q)-F(1,2)
               for i,c in enumerate(CARS) if c!=actor]
    if cert['released_lower_Q'] is not None:values.append(F(cert['released_lower_Q'],Q))
    # Half the pair surplus is reserved for the fixed separating contracts.
    margin=min(values)/2
    if margin<0:raise ValueError('candidate is incompatible with joint source')
    return margin


def advance_pose(model,pose,v,delta,u,sign):
    a,w=u;v1=v+H*a;d1=delta+H*w
    if not 0<=v<=2 or not 0<=v1<=2 or max(abs(delta),abs(d1))>F(1,2) or max(abs(a),abs(w))>1:
        raise ValueError('actual input/state bound violation')
    return model.propagate(pose,v,v1,delta,d1,sign,4),v1,d1


def reference_control(cert,phase):
    return tuple(map(F,cert['inputs'][phase])) if phase<cert['ticks'] else (F(0),F(0))


def initial_plan(model,state,cert,edge,stopped=False,progress=None):
    c=cert['car'];u=[reference_control(cert,j) for j in range(N)]
    poses=[state['poses'][c]];v=state['v'][c];d=state['delta'][c]
    for command in u:
        if v==d==0 and command==(0,0):poses.append(poses[-1])
        else:
            nodes,v,d=advance_pose(model,poses[-1],v,d,command,1 if c[0]=='A' else -1)
            poses.append(nodes[-1])
    return dict(controls=u,poses=poses,margin=root_margin(cert,edge['source']),
                id=content_id(dict(object=edge['object'],phase=0,controls=u)),
                history=state['history'],phase=0,v0=state['v'][c],delta0=state['delta'][c],
                object=edge['object'],joint_source=edge['source'],joint_target=edge['target'],require_physical_stop=stopped,
                progress=copy.deepcopy(progress))


def receiver_room(model,plan,cert,phase,cfg):
    end_phase=min(phase+N,cert['ticks']);free=end_phase-phase
    point=plan['poses'][free]
    if end_phase==cert['ticks']:
        center=point_domain(cfg,cert['car'],cert['target'])
        error=[point[j]-center[j] for j in (0,1)];theta=point[2]
        P,T=map(F,cert['receiver_radius']);distance=F(0)
    else:
        nominal=raw_pose(model,cert['pose_nodes_Q'][end_phase])
        origin=raw_pose(model,cert['pose_nodes_Q'][0])
        theta=point[2]-nominal[2]
        if theta.abs_upper()>1:return F(0),F(0),'inherited_image_only'
        si,co=model.sincos(theta);dx,dy=nominal[0]-origin[0],nominal[1]-origin[1]
        error=[point[0]-nominal[0]-(co-1)*dx+si*dy,
               point[1]-nominal[1]-si*dx-(co-1)*dy]
        P,T=map(F,cert['source_radius']);distance=dx.abs_upper()+dy.abs_upper()
    mp=P-sum(e.abs_upper() for e in error);mt=T-theta.abs_upper()
    # The inherited certificate still proves membership if an interval test is
    # inconclusive. A zero-radius box retains that exact anchor without widening.
    if min(mp,mt)<=0:return F(0),F(0),'inherited_image_only'
    return mp/4,min(mt/4,mp/(4*(distance+1))),'certified_positive_box'


def upper_norm(values):
    square=sum(x*x for x in values);q=10**24
    n=math.isqrt(square.numerator*q*q//square.denominator)
    if F(n,q)**2<square:n+=1
    return F(n,q)


def admit_delta(model,state,plan,cert,phase,delta,rp,rt,scale):
    delta=[tuple(map(F,row)) for row in delta]
    free=min(N,cert['ticks']-phase);c=cert['car']
    stops=[j['cut']-phase for j in cert['joins']
           if plan['require_physical_stop'] and 0<j['cut']-phase<=free]
    if any(any(row) for row in delta[free:]):raise ValueError('changed certified tail')
    t=upper_norm([x for row in delta for x in row]);s=t*t;b=derivative_bounds()
    if t>scale or b['Gc']*t+b['Hc']*s/2>plan['margin'] or b['Gp']*t+b['Hp']*s/2>rp or b['P1']*t+b['P2']*s/2>rt:
        raise ValueError('rational repaired proposal exceeds certified cone bound')
    commands=[tuple(a+d for a,d in zip(u,du)) for u,du in zip(plan['controls'],delta)]
    v=state['v'][c];d=state['delta'][c]
    for j,(a,w) in enumerate(commands):
        v+=H*a;d+=H*w
        if not 0<=v<=2 or abs(d)>F(1,2) or max(abs(a),abs(w))>1:raise ValueError('repaired proposal physical bounds')
        if j+1 in stops and v!=0:raise ValueError('physical stop missed')
    if sum(delta[j][0] for j in range(free)) or sum(delta[j][1] for j in range(free)):
        raise ValueError('receiver velocity/steering equality missed')
    if sum(a*a+w*w for a,w in commands)>sum(a*a+w*w for a,w in plan['controls']):
        raise ValueError('rational repair did not improve secondary input energy')
    poses=[plan['poses'][0]]
    for j in range(1,N+1):
        if j>free:poses.append(poses[free]);continue
        b_j=derivative_bounds(time_steps=j)
        ep=b_j['Gp']*t+b_j['Hp']*s/2;et=b_j['P1']*t+b_j['P2']*s/2
        old=plan['poses'][j];poses.append((inflate(model,old[0],ep),inflate(model,old[1],ep),inflate(model,old[2],et)))
    return {**plan,'controls':commands,'poses':poses,'margin':plan['margin']-b['Gc']*t-b['Hc']*s/2,
        'id':content_id(dict(parent=plan['id'],delta=delta)),
        'proof':dict(parent=plan['id'],delta=delta,rp=rp,rpsi=rt,scale=scale,norm=t)}


def optimize(model,state,plan,cert,phase,cfg,stopped):
    free=min(N,cert['ticks']-phase);c=cert['car']
    if stopped!=plan['require_physical_stop']:raise ValueError('changed stop rule within a committed branch')
    stops=[j['cut']-phase for j in cert['joins'] if stopped and 0<j['cut']-phase<=free]
    rp,rt,proof=receiver_room(model,plan,cert,phase,cfg)
    record=dict(accepted=False,receiver_box=proof,rp=str(rp),rpsi=str(rt),free_steps=free,
                stopped_indices=stops,coefficient_version='global-theorem-bounds-v1')
    try:
        solved=anchor_problem(plan['controls'],state['v'][c],state['delta'][c],plan['margin'],rp,rt,free,stops)
        record.update(solved)
        if not solved['solver_called']:
            record.update(accepted=True,changed=False);return plan,record
        scale=F(solved['variable_scale'])
        raw=solved['scaled_numerical_solution']
        delta=[[F(round(float(scale)*raw[2*j+k]*10**12),10**12) if j<free else F(0)
                for k in (0,1)] for j in range(N)]
        previous=0
        for end in sorted(set(stops+[free])):
            delta[end-1][0]=-sum(delta[j][0] for j in range(previous,end-1));previous=end
        delta[free-1][1]=-sum(delta[j][1] for j in range(free-1))
        updated=admit_delta(model,state,plan,cert,phase,delta,rp,rt,scale)
        t=updated['proof']['norm']
        record.update(accepted=True,changed=t>0,delta_norm_upper=str(t),geometry_reserve=str(updated['margin']))
        return updated,record
    except Exception as error:
        if hasattr(error,'report'):record.update(error.report)
        layer='candidate_numeric' if hasattr(error,'report') else 'new_candidate' if isinstance(error,ValueError) else 'optimizer_program_error'
        if any(word in str(error) for word in ('supplied reference','anchor initial-state','anchor misses')):layer='inherited_anchor'
        record.update(accepted=False,changed=False,rejection=str(error),rejection_type=type(error).__name__,fallback='execute committed input')
        record['failure_layer']=layer
        return plan,record


def shift_plan(model,plan,cert,phase,v,d):
    tail=reference_control(cert,phase+N)
    end_v=v+H*sum(u[0] for u in plan['controls'])
    end_d=d+H*sum(u[1] for u in plan['controls'])
    poses=plan['poses'][1:]
    if end_v==end_d==0 and tail==(0,0):poses.append(poses[-1])
    else:poses.append(advance_pose(model,poses[-1],end_v,end_d,tail,1 if cert['car'][0]=='A' else -1)[0][-1])
    commands=plan['controls'][1:]+[tail]
    return {**plan,'controls':commands,'poses':poses,'margin':plan['margin'],
        'id':content_id(dict(parent=plan['id'],operation='shift',tail=tail,phase=phase+1)),
        'history':content_id(dict(parent=plan['history'],actor=cert['car'],input=plan['controls'][0])),
        'phase':phase+1,'v0':v+H*plan['controls'][0][0],'delta0':d+H*plan['controls'][0][1]}


def execute(model,state,actor,command,cfg,released):
    """Continuous interval acceptance of the actual first input, before commit."""
    nodes,v,d=advance_pose(model,state['poses'][actor],state['v'][actor],state['delta'][actor],command,1 if actor[0]=='A' else -1)
    roads=[[model.I(F(x)).l for x in box] for box in cfg['road_rectangles']]
    chord=model.I(15*(H/4)**2/8).u;min_road=100*model.Q;min_pair=100*model.Q
    for left,right in zip(nodes,nodes[1:]):
        bodies={c:model.corners(left,1 if c[0]=='A' else -1)+model.corners(right,1 if c[0]=='A' else -1)
                if c==actor else model.corners(state['poses'][c],1 if c[0]=='A' else -1) for c in CARS}
        boxes={c:model.axis_box(body) for c,body in bodies.items()}
        for c in CARS:
            road=model.road(boxes[c],roads)-(chord if c==actor else 0);min_road=min(min_road,road)
            if road<0:raise ValueError('actual continuous road enclosure failed')
            if c!=actor and max(boxes[c][0]-18*model.Q,-18*model.Q-boxes[c][1])<0:
                raise ValueError('actual permission enclosure failed')
        if released and model.rear(boxes[actor],1 if actor[0]=='A' else -1)-chord<int(F(53,2)*model.Q):
            raise ValueError('released car reentered release boundary')
        for i,a in enumerate(CARS):
            for b in CARS[i+1:]:
                gap=model.gap(bodies[a],bodies[b],boxes[a],boxes[b],model.normed_axes(left))-(chord if actor in (a,b) else 0)
                min_pair=min(min_pair,gap)
                if gap<model.Q//2:raise ValueError('actual continuous six-pair enclosure failed')
    state['poses'][actor]=nodes[-1];state['v'][actor]=v;state['delta'][actor]=d
    state['history']=content_id(dict(parent=state['history'],actor=actor,input=command))
    return dict(minimum_road_lower_Q=min_road,minimum_pair_lower_Q=min_pair,
                executed_input=list(map(str,command)),actual_pose_Q=pose_rows(nodes[-1]),v=str(v),delta=str(d))


def commit_token(plan,state,cert,phase,cycle,version,groups,old_plan=None,cfg=None,model=None):
    """Reuse the existing four-way barrier, now binding the new domain/deadline."""
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'examples/head_on_mpc/scripts'))
    from m2b_protocol import Barrier
    actor=cert['car']
    if plan['object']!=cert['_object_id']:raise ValueError('wrong receiving-domain certificate')
    if old_plan is not None:
        if plan['progress']!=old_plan['progress']:raise ValueError('local proposal changes the continuation policy version')
        if plan['require_physical_stop']!=old_plan['require_physical_stop']:
            raise ValueError('proposal changes the matched stop rule')
        if plan['id']==old_plan['id']:
            if plan['controls']!=old_plan['controls'] or plan['margin']!=old_plan['margin'] or [pose_rows(p) for p in plan['poses']]!=[pose_rows(p) for p in old_plan['poses']]:
                raise ValueError('unchanged plan ID has changed inputs or receiver prediction')
        else:
            proof=plan.get('proof',{})
            if proof.get('parent')!=old_plan['id']:raise ValueError('proposal lacks committed parent')
            if model is None or cfg is None:raise ValueError('proposal needs interval proof validation')
            validate_update(model,state,old_plan,plan,cert,phase,cfg)
    if plan['history']!=state['history'] or plan['phase']!=phase:
        raise ValueError('stale actual-state lineage or phase')
    if plan['v0']!=state['v'][actor] or plan['delta0']!=state['delta'][actor]:
        raise ValueError('full-state velocity/steering seam mismatch')
    end=min(phase+N,cert['ticks'])
    target=(tuple(map(F,(cert['knots'][end][0],cert['knots'][end][2])))
            if end<cert['ticks'] else (F(0),F(0)))
    actual=(plan['v0']+H*sum(u[0] for u in plan['controls']),
            plan['delta0']+H*sum(u[1] for u in plan['controls']))
    if target!=actual:raise ValueError('moving receiver velocity/steering mismatch')
    context=dict(cycle=cycle,state_time_s=str(100+cycle*H),permission_version=version,
        owner=actor,groups=groups,pair_contracts=pair_partition(groups),
        source_history=state['history'],object=plan['object'],receiver_phase=end,
        joint_source=plan['joint_source'],joint_target=plan['joint_target'],
        physical_stop_required=plan['require_physical_stop'],
        remaining_deadline=min(N,cert['ticks']-phase),continuation_index=end,
        certificate_version='P0-input-cone-tube-v1')
    progress=plan.get('progress')
    if progress is not None:
        target=plan['joint_target'];value=progress['U'][target]
        if cert['ticks']+value>progress['source_allowance']:raise ValueError('connection exceeds accepted physical budget')
        context.update(policy_version=progress['version'],policy_sha256=content_id(dict(policy=progress['policy'],U=progress['U'])),
            receiver_time_value=cert['ticks']-end+value,physical_budget=cert['ticks']-phase+value)
        assert context['remaining_deadline']+context['receiver_time_value']==context['physical_budget']
    context=json.loads(json.dumps(context,default=str))
    barrier=Barrier(context);ids={};messages=[]
    def deliver(sender,receiver,kind,payload):
        envelope=dict(sender=sender,receiver=receiver,kind=kind,cycle=cycle,payload=payload)
        raw=json.dumps(envelope,default=str,sort_keys=True,separators=(',',':'))
        decoded=json.loads(raw)
        assert decoded['cycle']==cycle and decoded['sender']==sender and decoded['receiver']==receiver
        messages.append((len(raw.encode()),hashlib.sha256(raw.encode()).hexdigest()))
        return decoded['payload']
    if phase==0 and progress is not None:
        for c in CARS:deliver('coordinator',c,'continuation_policy',progress)
    # Explicit reliable ideal messages; these byte counts are application JSON,
    # not DDS traffic. All snapshots refer to the same frozen physical history.
    for c in CARS:
        snapshot=dict(car=c,history=state['history'],pose_Q=pose_rows(state['poses'][c]),
                      v=str(state['v'][c]),delta=str(state['delta'][c]),context=context)
        for receiver in [x for x in CARS if x!=c]+['coordinator']:deliver(c,receiver,'snapshot',snapshot)
    for c in CARS:
        controls=plan['controls'] if c==actor else [(F(0),F(0))]*N
        certificate=dict(root_object=plan['object'],source_history=state['history'],input_plan=plan['id'],geometry_reserve=str(plan['margin']),
            proof=plan['proof'] if old_plan is not None and plan['id']!=old_plan['id'] else {'kind':'inherited_certified_branch'}) if c==actor else \
            dict(kind='stationary_hold',stage=plan['joint_source'][CARS.index(c)],source_history=state['history'])
        public=dict(car=c,context=context,controls=controls,certificate=certificate)
        ids[c]=content_id(public)
        payload=dict(**public,kind='final',id=ids[c])
        for receiver in [x for x in CARS if x!=c]+['coordinator']:
            received=deliver(c,receiver,'final',payload)
            if content_id({k:received[k] for k in ('car','context','controls','certificate')})!=ids[c]:raise ValueError('message plan content mismatch')
            if receiver=='coordinator':barrier.publish(received)
    for c in CARS:
        if old_plan is not None and plan['id']!=old_plan['id']:validate_update(model,state,old_plan,plan,cert,phase,cfg)
        barrier.accept(deliver(c,'coordinator','ack',dict(car=c,context=context,plans=ids,accepted=True)))
    token=barrier.token()
    for c in CARS:deliver('coordinator',c,'commit',token)
    return {**token,'application_messages':len(messages),'application_bytes':sum(n for n,_ in messages),
            'message_digest':content_id(messages),'communication_scope':'reliable in-process JSON, no network/DDS headers'}


def validate_update(model,state,old,new,cert,phase,cfg):
    if new['id']==old['id']:return new
    proof=new['proof'];rp,rt,_=receiver_room(model,old,cert,phase,cfg);b=derivative_bounds()
    scale=min(rp/(b['Gp']+1),rt/(b['P1']+1),old['margin']/(b['Gc']+1),F(1,100))/2
    if proof['scale']!=scale:raise ValueError('wrong proposal trust scale')
    delta=[tuple(a-b for a,b in zip(n,o)) for n,o in zip(new['controls'],old['controls'])]
    checked=admit_delta(model,state,old,cert,phase,delta,rp,rt,scale)
    if checked['id']!=new['id'] or checked['margin']!=new['margin'] or [pose_rows(p) for p in checked['poses']]!=[pose_rows(p) for p in new['poses']]:
        raise ValueError('changed receiver prediction or unbound actual inputs')
    return new
