"""Actual-anchor interval variational composition and local input-L2 bounds."""
from fractions import Fraction as F
import copy
import importlib.util
import itertools
import math
import time

from common import *
from rules import envelope,assert_step
spec=importlib.util.spec_from_file_location('region_variational_builder',HERE/'sources/certify_interfaces.py')
builder=importlib.util.module_from_spec(spec);spec.loader.exec_module(builder)
ORDER=['px','py','psi','v','delta','a','omega']


def pack(m,matrix,scale=10**12):
    return dict(shape=[len(matrix),len(matrix[0])],scale=scale,entries=[[i,j,x.l*scale//m.Q,-(-x.u*scale//m.Q)] for i,row in enumerate(matrix) for j,x in enumerate(row) if x.l or x.u])


def unpack(m,value):
    result=[[m.I(0) for _ in range(value['shape'][1])] for _ in range(value['shape'][0])]
    for i,j,lo,hi in value['entries']:
        low=F(lo,value['scale']);high=F(hi,value['scale']);result[i][j]=m.I.raw(m.I(low).l,m.I(high).u)
    return result


def local_profiles(knots,contracts):
    beta=norm_upper([H]*32);g=z=gp=hp=F(0);result=[]
    for row,box in zip(knots,contracts):
        bounds=envelope(row,box,beta);v=bounds["V_abs"];d=bounds["D_abs"]
        kap,kd,kdd=builder.trig_bounds(d);c=kap+v*kd;t=2*kd+v*kdd
        increment=g*H+beta*c*H*H/2
        hp=builder.up(hp+2*beta*increment+v*(z*H+beta*beta*t*H*H/2+g*g*H+g*beta*c*H*H+beta*beta*c*c*H**3/3))
        gp=builder.up(gp+beta*H+v*increment);g=builder.up(g+beta*c*H);z=builder.up(z+beta*beta*t*H)
        result.append(dict(Gp=gp,P1=g,Hp=hp,P2=z,Gc=builder.up(gp+builder.RB*g),Hc=builder.up(hp+builder.RB*(z+g*g)),v_upper=v,delta_abs_upper=d))
    return result


def cached_covers(m,row,state,actor,command):
    d=row['domain'];v=F(row['nominal_v']);delta=F(row['nominal_delta']);rv=F(d.get('speed_radius',0));rd=F(d.get('steering_radius',0));rt=F(d.get('yaw_radius',0))
    pose=row['nominal_pose_Q'][2];lo=F(pose[0],m.Q)-rt;hi=F(pose[1],m.Q)+rt
    a,w=command;ar=F(d.get('feedback_a_ref',a));wr=F(d.get('feedback_omega_ref',w))
    return lo<=F(state['poses'][actor][2].l,m.Q)<=F(state['poses'][actor][2].u,m.Q)<=hi and abs(state['v'][actor]-v)<=rv and abs(state['delta'][actor]-delta)<=rd and abs(a-ar)<=rv/H and abs(w-wr)<=rd/H


def geometry_margin(m,data,state,actor,nodes):
    from physics import geometry
    w=get(HERE/'objects',state['witness']);bounds=geometry(nodes,w['owners'][state['cycle']],4)
    reserve=F(get(HERE/'objects',state['contract_id'])['cap'][actor])
    assert reserve>0,'actual joint trajectory has no local geometric reserve'
    return reserve


def joint_nodes(state,actor,command):
    from physics import predicted_nodes
    w=get(HERE/'objects',state['witness']);k=state['cycle'];u={c:tuple(map(F,w['controls'][c][k])) for c in CARS};u[actor]=command
    return predicted_nodes(state,u,4)


def build(data,m,anchor,subdivisions=8):
    start=time.perf_counter();state=restore(m,anchor['state']);actor=anchor['actor'];u=[tuple(map(F,x)) for x in anchor['plan']['remaining'][:32]]
    assert anchor['m']==32 and anchor['rho']==str(RHO) and anchor['model_sha256']==sha(MODEL_PATH)
    assert anchor['direction_sign']==direction(actor) and anchor['physics_sha256']==content_id(data.cfg)
    states=[];singles=[];prefix=[];knots=[];geometry=[];S=[[] for _ in range(5)]
    for j,(a,w) in enumerate(u):
        pose=state['poses'][actor];v,d=state['v'][actor],state['delta'][actor];v1,d1=v+H*a,d+H*w
        assert_step(v,d,(a,w),anchor['contracts'][j])
        bounds=dict(pose_Q=pose_rows(pose),v=[str(v),str(v)],delta=[str(d),str(d)],a=[str(a),str(a)],omega=[str(w),str(w)])
        source=dict(anchor_id=content_id(anchor),step=j,state_order=ORDER[:5],input_order=ORDER[5:],direction_sign=anchor['direction_sign'],bounds=bounds,model_sha256=anchor['model_sha256'])
        tiny=dict(scale=m.Q,car=actor,direction_sign=anchor["direction_sign"],knots=[[str(x) for x in (v,v1,d,d1)]],pose_nodes_Q=[pose_rows(pose)])
        raw=builder.interval_jacobian(m,tiny,0,{},subdivisions)
        J=[[m.I(0) for _ in range(7)] for _ in range(5)]
        for r,c,lo,hi in raw['nonzero_intervals']:J[r][c]=m.I.raw(m.I(F(lo,raw['scale'])).l,m.I(F(hi,raw['scale'])).u)
        nextS=[]
        for r in range(5):nextS.append([sum((J[r][q]*S[q][k] for q in range(5)),m.I(0)) for k in range(2*j)]+[J[r][5],J[r][6]])
        S=nextS;prefix.append(pack(m,S));singles.append(dict(source=source,source_id=content_id(source),matrix=raw));states.append(state_record(state))
        nodes=joint_nodes(state,actor,(a,w))
        geometry.append(geometry_margin(m,data,state,actor,nodes));knots.append((v,v1,d,d1))
        legacy.execute(m,state,actor,(a,w),data.cfg,data.cert['source']>0)
    profiles=local_profiles(knots,anchor["contracts"][:32])
    return dict(anchor_id=content_id(anchor),anchor=anchor,source_kind='fresh actual joint state and time-aligned fixed peer trajectories',columns=ORDER,coordinate_branch=anchor['coordinate_branch'],direction_sign=anchor['direction_sign'],rho=str(RHO),trust_center=content_id(anchor),input_dimension=64,single_step_dimension=7,subdivisions=subdivisions,singles=singles,prefix_jacobians=prefix,local_profiles=profiles,geometry_margins=geometry,terminal_state=state_record(state),all_32_source_ranges_covered=True,physical_affine_rows_required=True,build_wall_s=time.perf_counter()-start)


def verify(data,m,cert,anchor,recompute=True):
    assert cert['anchor_id']==cert['trust_center']==content_id(anchor),'wrong trust center'
    assert cert['anchor']==anchor and cert['rho']==str(RHO) and cert['columns']==ORDER,'wrong order/rho'
    assert cert['direction_sign']==anchor['direction_sign']==direction(anchor['actor']),'wrong direction'
    assert len(cert['singles'])==len(cert['prefix_jacobians'])==len(cert['local_profiles'])==32
    for j,step in enumerate(cert['singles']):
        assert step['source_id']==content_id(step['source']) and step['source']['anchor_id']==content_id(anchor) and step['source']['step']==j
        assert step['matrix']['scale']==10**9 and step['matrix']['columns']==ORDER
        assert cert['prefix_jacobians'][j]['shape']==[5,2*(j+1)] and cert['prefix_jacobians'][j]['scale']==10**12
    if recompute:
        actual=build(data,m,anchor,cert['subdivisions'])
        for name in ('singles','prefix_jacobians','local_profiles','geometry_margins','terminal_state'):assert content_id(actual[name])==content_id(cert[name]),'derivative replay mismatch: '+name
    return True
