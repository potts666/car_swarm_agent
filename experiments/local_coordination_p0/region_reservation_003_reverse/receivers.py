"""Margin allocation and exact feedback into a saved certified suffix."""
from common import *
from rules import contracts_for,assert_step
from physics import step,geometry,predicted_nodes,body
from certify_domains import check_domain


def domain_description(car,kind):
    if kind.startswith('generated:'):return get(HERE/'objects',kind.split(':')[1])['domain']
    d=read(HERE/'domain_checks.json')['domains'][kind]
    xy=WAIT[car] if kind in ('W_in','W_hold') else (direction(car)*F(d.get('center_local_x',-22 if kind=='G_in' else 34 if kind=='G_out' else 46)),0 if kind in ('G_in','G_in_shifted','G_out') else WAIT[car][1])
    return dict(center_pose_Q=[[F(x).numerator,F(x).denominator] for x in (*xy,0)],position_l1_radius=d['position_l1'],yaw_radius=d['yaw'],center_v=str((F(d['v'][0])+F(d['v'][1]))/2),speed_radius=str((F(d['v'][1])-F(d['v'][0]))/2),center_delta=str((F(d['delta'][0])+F(d['delta'][1]))/2),steering_radius=str((F(d['delta'][1])-F(d['delta'][0]))/2))

def check_generated(state,car,domain):
    center=list(map(lambda x:F(*x),domain['center_pose_Q']));p=state['poses'][car]
    ep=sum((p[j]-center[j]).abs_upper() for j in (0,1));et=(p[2]-center[2]).abs_upper()
    assert ep<=F(domain['position_l1_radius']) and et<=F(domain['yaw_radius']),'generated receiver pose/yaw'
    for key,radius,ref in [('v','speed_radius','center_v'),('delta','steering_radius','center_delta')]:assert abs(state[key][car]-F(domain[ref]))<=F(domain[radius]),'generated receiver '+key
    return dict(car=car,position_l1=str(ep),yaw=str(et),v=str(state['v'][car]),delta=str(state['delta'][car]))

def feedback(state,actor,certificate):
    d=certificate['domain'];u=tuple(map(F,certificate['reference_K']))
    return (u[0]-(state['v'][actor]-F(d['center_v']))/H,u[1]-(state['delta'][actor]-F(d['center_delta']))/H)

def synthesize(state,actor):
    from derivatives import builder
    w=get(HERE/'objects',state['witness']);k=state['cycle'];n=k+32
    later=sorted((int(t),kind) for t,ev in w['events'].items() for c,kind in ev if c==actor and int(t)>n+1)
    if not later:raise ValueError('missing_continuation: no retained receiver')
    target,kind=later[0];target_domain=domain_description(actor,kind);boxes=contracts_for(state,actor,target-k+1);s=copy.deepcopy(state);road=pair=F(100);resource=F(100);Lrem=F(0);theta_max=F(0)
    for t in range(k,target):
        u={c:tuple(map(F,w['controls'][c][t])) for c in CARS};assert_step(s['v'][actor],s['delta'][actor],u[actor],boxes[t-k])
        if t==n:
            nominal=state_record(s)
            old_pose=s['poses'][actor];source_ep=sum(F(x.u-x.l,2*m.Q) for x in old_pose[:2]);source_et=F(old_pose[2].u-old_pose[2].l,2*m.Q)
            reference_center=[F(x.l+x.u,2*m.Q) for x in old_pose]
            s['poses'][actor]=tuple(m.I(x) for x in reference_center)
        if t>=n:
            nodes=predicted_nodes(s,u,4);g=geometry(nodes,w['owners'][t],4);road=min(road,g['road']);pair=min(pair,g['pair']-SAFE)
            theta_max=max(theta_max,max(p[2].abs_upper() for p in nodes[actor]))
            if w['owners'][t]!=actor:
                for pose in nodes[actor]:
                    corners=body(pose,actor);xs=[p[0] for p in corners];gap=max(F(-16)-F(max(x.u for x in xs),m.Q),F(min(x.l for x in xs),m.Q)-16)-TUBE-F(15)*(H/4)**2/8
                    resource=min(resource,gap)
            if t>n:Lrem+=H*(abs(s['v'][actor])+abs(s['v'][actor]+H*u[actor][0]))/2
        step(s,u,w['owners'][t],4)
        if t==n:after=state_record(s)
    check_generated(s,actor,target_domain)
    p=[m.I.raw(*x) for x in nominal['poses_Q'][actor]];center=[F(x.l+x.u,2*m.Q) for x in p];ep=et=F(0)
    v,d=F(nominal['v'][actor]),F(nominal['delta'][actor]);v1,d1=F(after['v'][actor]),F(after['delta'][actor]);a,omega=map(F,w['controls'][actor][n]);box=boxes[32]
    rv=min(v-F(box['v'][0]),F(box['v'][1])-v,H*(a-F(box['a'][0])),H*(F(box['a'][1])-a));rd=min(d-F(box['delta'][0]),F(box['delta'][1])-d,H*(omega-F(box['omega'][0])),H*(F(box['omega'][1])-omega))
    assert rv>=0 and rd>=0,'nonpositive physical cap'
    D=max(abs(d)+rd,abs(d1));assert D<=F(1,2)
    kap,kd,_=builder.trig_bounds(D);V=max(abs(v),abs(v1));C=F(3,2)
    tc=[F(*x) for x in target_domain['center_pose_Q']];pe=sum((s['poses'][actor][j]-tc[j]).abs_upper() for j in (0,1));te=(s['poses'][actor][2]-tc[2]).abs_upper()
    def coeff(length):return [F(1),C*V*H+length,C*H/2+C*V*H*H*kap/3+length*H*kap/2,C*V*H*H*V*kd/3+length*H*V*kd/2]
    ap=coeff(C*Lrem);ag=coeff(C*Lrem+RB);at=[F(0),F(1),H*kap/2,H*V*kd/2]
    inequalities=[dict(name='retained_position',a=ap,b=F(target_domain['position_l1_radius'])-pe-ep-ap[1]*et),dict(name='retained_yaw',a=at,b=F(target_domain['yaw_radius'])-te-et),dict(name='continuous_geometry',a=ag,b=min(road,pair,resource)-ep-ag[1]*et),dict(name='trig_range',a=at,b=F(1)-theta_max-et)]
    baseline=[source_ep,source_et,F(0),F(0)]
    for row in inequalities:row['baseline_cost']=sum(a*x for a,x in zip(row['a'],baseline));row['remaining']=row['b']-row['baseline_cost']
    active=[0,1]+([2] if rv else [])+([3] if rd else []);r=[None,None,rv,rd];limits=[]
    diagnostics=normal(dict(source=state_record(state),actor=actor,receiver_phase=n,target_phase=target,target_kind=kind,nominal_at_receiver=nominal,nominal_at_target=state_record(s),target_domain=target_domain,inequalities=inequalities,baseline=baseline,ep=ep,et=et,nominal_target_position_error=pe,nominal_target_yaw_error=te))
    for i in active:
        candidates=[(F(row['remaining'])/(2*len(active)*row['a'][i]),row['name']) for row in inequalities if row['a'][i]>0]
        if i>=2:candidates.append((r[i]-baseline[i],'physical_K_cap'))
        radius,lim=min(candidates)
        if radius<=0:
            key=put(HERE/'objects',dict(**diagnostics,failure='nonpositive_margin',axis=i,candidates=candidates));raise ValueError('nonpositive_margin: '+lim+' counterexample='+key)
        r[i]=baseline[i]+radius;limits.append(dict(axis=i,limiting_row=lim,baseline=baseline[i],increment=radius,radius=r[i]))
    for row in inequalities:
        row['slack']=row['b']-sum(a*x for a,x in zip(row['a'],r));assert row['slack']>=0
    if r[0]<source_ep or r[1]<source_et:
        key=put(HERE/'objects',dict(**diagnostics,failure='center_not_contained',radii=r,limiting_rows=limits));raise ValueError('center_not_contained: counterexample='+key)
    domain=dict(center_pose_Q=[[x.numerator,x.denominator] for x in center],position_l1_radius=str(r[0]),yaw_radius=str(r[1]),center_v=str(v),speed_radius=str(r[2]),center_delta=str(d),steering_radius=str(r[3]))
    certificate=normal(dict(rule='margin_allocation_v2_1',source=state_record(state),actor=actor,phase=n,domain=domain,reference_K=w['controls'][actor][n],reset_v=str(v1),reset_delta=str(d1),retained_target_phase=target,retained_target_kind=kind,retained_target_domain=target_domain,retained_controls_id=content_id({c:w['controls'][c][n+1:] for c in CARS}),old_witness=state['witness'],budget_before=w['ticks']-k,prefix_ticks=33,suffix_ticks=w['ticks']-(n+1),dimension=2+int(rv>0)+int(rd>0),allocation=inequalities,limiting_rows=limits,baseline=baseline,constants=dict(ep=ep,et=et,V=V,D_abs=D,kappa=kap,kappa_delta=kd,L_rem=Lrem,road=road,pair_surplus=pair,resource=resource),nominal_at_receiver=nominal,nominal_after_K=after,nominal_at_retained=state_record(s)))
    check_generated(restore(m,nominal),actor,domain)
    # Zero candidate's K is exactly the old actual next command.
    assert feedback(restore(m,nominal),actor,certificate)==tuple(map(F,w['controls'][actor][n]))
    key=put(HERE/'objects',certificate)
    return dict(phase=n,kind='generated:'+key,domain=domain,certificate_id=key)

def replacement_events(old,actor,row):
    events=copy.deepcopy(old);n=row['phase'];source=get(HERE/'objects',row['certificate_id'])['source']['cycle']
    for t,items in list(events.items()):
        if source<int(t)<=n+1:
            for c,kind in list(items):
                if c==actor:
                    assert kind in ('G_in','G_in_shifted') or kind.startswith('generated:'),'cannot replace hard receiver'
                    items.remove([c,kind])
            if not items:del events[t]
    events.setdefault(str(n),[]).append([actor,row['kind']]);return events
