#!/usr/bin/env python3
"""B0 exact calibration, coverage and certificate identity rejection checks."""
import argparse
from common import *
from derivatives import build,verify,unpack,cached_covers
from socp import rows_for,evaluate


def exact_composition(n,sign):
    L=F(27,10);v=F(1,2)
    A0=[[F(int(i==j)) for j in range(5)] for i in range(5)]
    A0[0][3]=sign*H;A0[1][2]=sign*v*H;A0[1][4]=sign*v*v*H*H/(2*L);A0[2][4]=v*H/L
    B0=[[F(0)]*2 for _ in range(5)]
    B0[0][0]=sign*H*H/2;B0[3][0]=H;B0[1][1]=sign*v*v*H**3/(6*L);B0[2][1]=v*H*H/(2*L);B0[4][1]=H
    S=[[] for _ in range(5)];out=[]
    for k in range(n):
        S=[[sum(A0[i][q]*S[q][j] for q in range(5)) for j in range(2*k)]+B0[i] for i in range(5)];out.append(S)
    return out


def main():
    p=argparse.ArgumentParser();p.add_argument('--index',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if args.output.exists():p.error('preserve previous checks')
    args.output.mkdir(parents=True);data,m=setup();index=read(args.index);results=[]
    for actor,sign in [('A2',1),('B1',-1)]:
        state=legacy.initial_state(m,data.cfg);state['poses'][actor]=(m.I(0),m.I(0),m.I(0));state['v'][actor]=F(1,2)
        plan=A.initial_plan(data,legacy.initial_state(m,data.cfg));plan['remaining']=[(F(0),F(0))]*32+plan['remaining'][32:]
        anchor=anchor_for(data,m,state,plan);anchor['actor']=actor;anchor['direction_sign']=sign;anchor['fixture']='exact straight calibration only'
        cert=build(data,m,anchor,8);exact=exact_composition(32,sign)
        for j,packed in enumerate(cert['prefix_jacobians']):
            matrix=unpack(m,packed);assert packed['shape']==[5,2*(j+1)]
            for r in range(5):
                for col in range(2*(j+1)):assert F(matrix[r][col].l,m.Q)<=exact[j][r][col]<=F(matrix[r][col].u,m.Q)
        results.append(dict(test='exact_straight_1_3_32_and_all_prefixes',actor=actor,passed=True))
    item=index['curve'];anchor=get(HERE/'objects',item['anchor_id']);cert=get(HERE/'objects',item['derivative_id'])
    state=restore(m,anchor['state']);cache=read(HERE/'inputs/cache_examples.json');row=next(x for x in cache if x['phase']==5)
    # Rotate a source perturbation through the exact nominal flow image.
    nominal=[m.I.raw(*q) for q in data.cert['pose_nodes_Q'][5]];origin=[m.I.raw(*q) for q in data.cert['pose_nodes_Q'][0]]
    e=F(data.cert['source_radius'][1])*F(3,4);si,co=m.sincos(m.I(e));dx,dy=nominal[0]-origin[0],nominal[1]-origin[1]
    outside=copy.deepcopy(state);outside['poses']['A2']=(origin[0]+co*dx-si*dy,origin[1]+si*dx+co*dy,nominal[2]+e)
    assert in_flow(m,outside,data.cert,5)
    u=tuple(map(F,anchor['plan']['remaining'][0]));assert not cached_covers(m,row,outside,'A2',u)
    changed=anchor_for(data,m,outside,anchor['plan']);fresh=build(data,m,changed,8);verify(data,m,fresh,changed)
    results.append(dict(test='legal_flow_outside_cached_yaw_recomputed',passed=True,new_anchor_id=content_id(changed)))
    widened=copy.deepcopy(anchor);u=list(map(F,widened['plan']['remaining'][0]));u[0]+=2*F(row['domain']['speed_radius'])/H
    assert abs(u[0])<=1 and not cached_covers(m,row,state,'A2',u)
    widened['plan']['remaining'][0]=list(map(str,u));fresh=build(data,m,widened,8);verify(data,m,fresh,widened)
    results.append(dict(test='input_outside_cached_feedback_rectangle_recomputed',passed=True))
    for kind in ('state_order','input_order','B_direction','matrix_scale','object','phase'):
        bad=copy.deepcopy(cert)
        if kind=='state_order':bad['columns'][0],bad['columns'][1]=bad['columns'][1],bad['columns'][0]
        if kind=='input_order':bad['singles'][0]['matrix']['columns'][5:]=['omega','a']
        if kind=='B_direction':bad['direction_sign']=-1
        if kind=='matrix_scale':bad['singles'][0]['matrix']['scale']=10**8
        if kind=='object':bad['singles'][0]['source']['anchor_id']='wrong-object'
        if kind=='phase':bad['singles'][0]['source']['step']=1
        try:verify(data,m,bad,anchor)
        except AssertionError:results.append(dict(test='reject_'+kind,passed=True))
        else:raise AssertionError('bad derivative accepted: '+kind)
    first=[F(3,4000)]+[F(0)]*63;second=first.copy()
    assert norm_upper(first)<=RHO and norm_upper(second)<=RHO and norm_upper([x+y for x,y in zip(first,second)])>RHO
    recentered=copy.deepcopy(anchor);recentered['plan']['remaining'][0][0]=str(F(recentered['plan']['remaining'][0][0])+first[0])
    fresh=build(data,m,recentered,8);verify(data,m,fresh,recentered)
    try:verify(data,m,cert,recentered,False)
    except AssertionError:results.append(dict(test='accumulated_reference_offset_rejected_and_new_center_recertified',passed=True))
    else:raise AssertionError('old center accepted after recentering')
    write(args.output/'checks.json',dict(passed=True,cases=results,finite_difference_used_as_proof=False))
    print('B0 checks passed:',len(results))


if __name__=='__main__':main()
