#!/usr/bin/env python3
"""One new physical-waypoint connector: shared geometry, stopped/moving joins.

Floating-point shooting proposes rational inputs. The supplied outward interval
integrator alone verifies those inputs; old artifacts and scripts stay untouched.
This certifies a local addition, not the five-group closed-loop experiment.
"""
import argparse
from fractions import Fraction as F
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import sys


def speed_nodes(distance, first, last):
    h,a,vmax=F(1,5),F(4,5),F(3,2)
    n=max(1,math.ceil(abs(last-first)/(a*h)))
    for n in range(n,2001):
        high=[min(vmax,first+a*h*j,last+a*h*(n-j)) for j in range(n+1)]
        low=[max(F(0),first-a*h*j,last-a*h*(n-j)) for j in range(n+1)]
        upper=h*(sum(high)-(first+last)/2);lower=h*(sum(low)-(first+last)/2)
        if lower<=distance<=upper:
            ratio=(distance-lower)/(upper-lower) if upper>lower else F(0)
            nodes=[x+ratio*(y-x) for x,y in zip(low,high)]
            assert nodes[0]==first and nodes[-1]==last
            return nodes
    raise ValueError('no profile within the development duration bound')


def propose(graph,car,source,target,stopped):
    model=graph.model;points=graph.route_waypoints(car,source,target)
    pose=tuple(model.I(x) for x in (*points[0],F(0)))
    controls=[];joins=[];sign=graph.sign(car)
    for k,(start,end) in enumerate(zip(points,points[1:])):
        length,at=graph.curve_data(start,end)
        first=F(0) if k==0 or stopped else F(6,5)
        last=F(0) if k+2==len(points) or stopped else F(6,5)
        velocity=speed_nodes(F(round(length*10**10),10**10),first,last)
        delta=[];distance=F(0)
        for j,v in enumerate(velocity):
            if j:distance+=model.H*(velocity[j-1]+v)/2
            delta.append(F(round(at(min(float(distance),length))[3]*10**8),10**8))
        delta[0]=delta[-1]=F(0)
        velocity,delta=model.shoot(velocity,delta,pose,tuple(map(float,end)),sign,target_heading=0.)
        assert velocity[0]==first and velocity[-1]==last and delta[0]==delta[-1]==0
        for j in range(len(velocity)-1):
            row=(velocity[j],velocity[j+1],delta[j],delta[j+1])
            # Candidate pose used by the next shooting step is already outward-enclosed.
            pose=model.propagate(pose,*row,sign,4)[-1]
            controls.append(row)
        joins.append(dict(cut=len(controls),reference_pose_Q=[[x.l,x.u] for x in pose],
                          v=str(last),delta='0',waypoint=list(map(str,end))))
    return controls,joins,points


def verify(graph,car,source,target,knots):
    model=graph.model;H,Q,RB=model.H,model.Q,model.RB
    origin=graph.station(car,source);pose=tuple(model.I(x) for x in (*origin,F(0)))
    pose_nodes=[[[x.l,x.u] for x in pose]]
    p_radius,t_radius=graph.STAGE_RADII[source]
    fixed={(c,k):graph.body(c,k) for c in graph.CARS if c!=car for k in range(4)}
    boxes={key:model.axis_box(value) for key,value in fixed.items()}
    fixed_budget={key:model.I(graph.STAGE_RADII[key[1]][0]+RB*graph.STAGE_RADII[key[1]][1]).u for key in fixed}
    gaps={key:100*Q for key in fixed};road=100*Q;released=100*Q
    chord=model.I(15*(H/4)**2/8).u
    assert knots and knots[0][0]==knots[-1][1]==knots[0][2]==knots[-1][3]==0
    for j,(v0,v1,d0,d1) in enumerate(knots):
        assert 0<=min(v0,v1) and max(v0,v1)<=2
        assert max(abs(d0),abs(d1))<=F(1,2)
        assert max(abs((v1-v0)/H),abs((d1-d0)/H))<=1
        if j:assert knots[j-1][1]==v0 and knots[j-1][3]==d0
        nodes=model.propagate(pose,v0,v1,d0,d1,graph.sign(car),4)
        for a,b in zip(nodes,nodes[1:]):
            cs=model.corners(a,graph.sign(car))+model.corners(b,graph.sign(car))
            radius=max((x-model.I(origin[0])).abs_upper()+(y-model.I(origin[1])).abs_upper()
                       for x,y in cs)+F(chord,Q)
            error=chord+model.I(p_radius+t_radius*radius).u
            bb=model.axis_box(cs);road=min(road,model.road(bb,graph.ROADS)-error)
            if source:released=min(released,model.rear(bb,graph.sign(car))-int(graph.RELEASE*Q)-error)
            axes=model.normed_axes(a)
            for key in fixed:
                gaps[key]=min(gaps[key],model.gap(cs,fixed[key],bb,boxes[key],axes)-error-fixed_budget[key])
        pose=nodes[-1]
        pose_nodes.append([[x.l,x.u] for x in pose])
    end=graph.station(car,target)
    nominal_error=(pose[0]-model.I(end[0])).abs_upper()+(pose[1]-model.I(end[1])).abs_upper()
    displacement=(pose[0]-model.I(origin[0])).abs_upper()+(pose[1]-model.I(origin[1])).abs_upper()
    end_p=nominal_error+p_radius+t_radius*displacement;end_t=pose[2].abs_upper()+t_radius
    target_p,target_t=graph.STAGE_RADII[target]
    assert end_p<=target_p and end_t<=target_t,'receiver set inclusion failed'
    assert road>0 and (not source or released>=0),'road/release certificate failed'
    return dict(car=car,source=source,target=target,ticks=len(knots),
        road_lower_Q=road,released_lower_Q=released if source else None,
        peer_gap_lower_Q={c+':'+str(k):value for (c,k),value in gaps.items()},
        endpoint_position_upper=str(end_p),endpoint_heading_upper=str(end_t),
        endpoint_pose_Q=[[x.l,x.u] for x in pose],endpoint_inside=True,
        pose_nodes_Q=pose_nodes,
        inputs=[[str((v1-v0)/H),str((d1-d0)/H)] for v0,v1,d0,d1 in knots],
        knots=[[str(x) for x in row] for row in knots],
        source_radius=list(map(str,graph.STAGE_RADII[source])),
        receiver_radius=list(map(str,graph.STAGE_RADII[target])),
        integration='supplied fixed-point outward Simpson/Taylor, Q=10^20, 4 geometric subintervals/tick',
        shared_with_archival_verifier=['interval arithmetic','trigonometry','propagation','corners','road','gap'],
        optimization_controller_executed=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rebuild',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('use a new output directory')
    out=args.output;out.mkdir(parents=True)
    work=(args.rebuild/'work').resolve();input_check=json.loads((args.rebuild/'input_verification.json').read_text())
    assert json.loads((args.rebuild/'summary.json').read_text())['status']=='passed'
    for name,sha in input_check['files'].items():
        if name.endswith('.py') or name.endswith('motions.json') or name.endswith('joint_graph.json'):
            assert hashlib.sha256((work/name).read_bytes()).hexdigest()==sha,name
    path=work/'NTU_DMPC_Dissertation/certificate_graph.py'
    spec=importlib.util.spec_from_file_location('p0_archival_graph',path);graph=importlib.util.module_from_spec(spec);spec.loader.exec_module(graph)
    budgets=json.loads((args.rebuild/'recomputed_budgets.json').read_text())
    archived=json.loads((work/'NTU_DMPC_Dissertation/graph_certificate/joint_graph.json').read_text())
    edge=next(e for e in archived['winning_edges'] if e['source']=='0000' and e['actor']=='A1' and e['target']=='2000')
    shutil.copy2(Path(__file__),out/'cross_stop.py')
    write=lambda path,value:path.write_text(json.dumps(value,default=str,ensure_ascii=False,indent=2)+'\n')
    results={}
    try:
        for stopped,label in ((True,'S'),(False,'M')):
            print(label+': propose',flush=True)
            controls,joins,points=propose(graph,'A1',0,2,stopped)
            write(out/(label+'_proposed.json'),dict(knots=controls,joins=joins,waypoints=points))
            print(label+': interval verify',flush=True)
            cert=verify(graph,'A1',0,2,controls)
            for i,c in enumerate(graph.CARS):
                if c!='A1':assert cert['peer_gap_lower_Q'][c+':'+edge['source'][i]]>=graph.Q//2
            assert graph.static_state_safe(tuple(map(int,edge['source'])))
            cert.update(joins=joins,waypoints=[list(map(str,p)) for p in points],
                joint_source=edge['source'],joint_receiver=edge['target'],permission_owner='A1',
                other_cars='hold in source stage domains; static pairs and roads certified by graph source',
                budget_C=cert['ticks']+budgets['C'][edge['target']],
                budget_R=cert['ticks']+budgets['R'][edge['target']])
            assert cert['budget_C']<=budgets['C'][edge['source']]
            assert cert['budget_R']<=budgets['R'][edge['source']]
            results[label]=cert;write(out/(label+'_certificate.json'),cert)
            print(label+': certified '+str(cert['ticks'])+' ticks',flush=True)
        assert results['S']['waypoints']==results['M']['waypoints']
        assert all(F(x['v'])==0 for x in results['S']['joins'])
        assert all(F(x['v'])>0 for x in results['M']['joins'][:-1])
        summary=dict(passed=True,source=edge['source'],receiver=edge['target'],
            stopped_ticks=results['S']['ticks'],moving_ticks=results['M']['ticks'],
            saved_ticks=results['S']['ticks']-results['M']['ticks'],
            common_nominal_speed_before_shooting='1.5 m/s',common_nominal_acceleration_before_shooting='0.8 m/s^2',
            physical_limits='0<=v<=2, |a|<=1, |delta|<=.5, |omega|<=1',
            free_prefix_limit=32,coarse_reference_requires_moving_domain_cuts=True,
            full_P0_passed=False,closed_loop_experiment=False,
            scope='new full-source interval-certified local connector, not a global closed-loop time effect')
        write(out/'summary.json',summary);print(json.dumps(summary),flush=True)
    except Exception as error:
        write(out/'failure.json',dict(error=repr(error),completed=list(results),exit_code=1));raise
    return 0


if __name__=='__main__':raise SystemExit(main())
