#!/usr/bin/env python3
"""Offline actual-input acceptance, separate from the pilot controller.

Shares only the archived fixed-point arithmetic/trig/propagation primitives.
Uses eight subintervals, independently formed rectangles and separating axes;
the online trace uses four subintervals. No controller decision is trusted as a
safety or goal result. This is not an independent implementation of arithmetic.
"""
import argparse
from fractions import Fraction as F
import gzip
import hashlib
import importlib.util
import itertools
import json
import math
from pathlib import Path
import time

CARS=('A1','A2','B1','B2');H=F(1,5)


def sha(value):return hashlib.sha256(json.dumps(value,default=str,sort_keys=True,separators=(',',':')).encode()).hexdigest()
def write(path,value):path.write_text(json.dumps(value,default=str,ensure_ascii=False,indent=2)+'\n')


def bounds(k=32):
    beta=F(6,5);T=H*k;L=F(27,10);c=F(11,20);sec=F(64,49);rb=F(1807,500)
    p1=T/L*(beta*c+2*sec*beta)
    p2=T/L*(2*sec*beta*beta+4*sec*c*beta*beta)
    hp=T*(2*beta*p1+2*(p2+p1*p1));gp=T*(beta+2*p1)
    return dict(P1=p1,P2=p2,Hp=hp,Gp=gp,Hc=hp+rb*(p2+p1*p1),Gc=gp+rb*p1)


def accept(rebuild,objects,run,case,connection=False):
    began=time.perf_counter();work=(rebuild/'work').resolve()
    spec=importlib.util.spec_from_file_location('accept_interval',work/'M2b_首段自由度与递归可行性/route_zoh_certificate.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    cfg=json.loads((objects/case/'config.json').read_text());coord=json.loads((objects/case/'coordination.json').read_text())
    for key,value in [('L',F(27,10)),('length',F(9,2)),('width',F(9,5)),('rear',F(1)),('front',F(4,5))]:
        assert F(cfg['vehicle'][key])==value,'acceptor only supports the declared theory vehicle'
    summary=json.loads((run/'summary.json').read_text());cache={}
    def obj(key):
        if key not in cache:
            p=objects/'objects'/(key+'.json');assert hashlib.sha256(p.read_bytes()).hexdigest()==key
            cache[key]=json.loads(p.read_text())
        return cache[key]
    pose4={};pose8={};v={c:F(0) for c in CARS};delta=v.copy()
    for c in CARS:
        z=cfg['initial'][c];theta=m.I(F(z[2]))
        if c[0]=='B':theta-=m.I.raw(m.I(F('3.14159265358979323846264338327950288')).l,m.I(F('3.14159265358979323846264338327950289')).u)
        pose4[c]=(m.I(F(z[0])),m.I(F(z[1])),theta);pose8[c]=pose4[c]
    history=sha(dict(initial=cfg['initial'],model='bicycle-ZOH-.2-v1'))
    Q=m.Q;epsilon=m.I(15*(H/8)**2/8).u
    roads=[[m.I(F(x)).l for x in b] for b in cfg['road_rectangles']]
    def rectangle(p,c):
        si,co=m.sincos(p[2]);s=1 if c[0]=='A' else -1
        return [(p[0]+s*(a*co-b*si),p[1]+s*(a*si+b*co))
                for a in (F(-1),F(7,2)) for b in (F(-9,10),F(9,10))]
    def box(corners):return (min(x.l for x,y in corners),max(x.u for x,y in corners),min(y.l for x,y in corners),max(y.u for x,y in corners))
    def separation(a,b,pa,pb):
        ba,bb=box(a),box(b)
        best=max(bb[0]-ba[1],ba[0]-bb[1],bb[2]-ba[3],ba[2]-bb[3])
        if best>=Q//2+epsilon:return best
        for p in (pa,pb):
            angle=(p[2].l+p[2].u)/(2*Q)
            c=F(round(.999999*math.cos(angle)*10**6),10**6)
            s=F(round(.999999*math.sin(angle)*10**6),10**6)
            assert c*c+s*s<=1
            for nx,ny in ((c,s),(-s,c)):
                xa=[x*nx+y*ny for x,y in a];xb=[x*nx+y*ny for x,y in b]
                best=max(best,min(q.l for q in xb)-max(q.u for q in xa),min(q.l for q in xa)-max(q.u for q in xb))
        return best
    def node_contains(label,radii):
        ys=sorted(set(F(z[1]) for z in cfg['goals'].values()),reverse=True)
        staging=F(cfg['release_plane_abs_x'])+F(cfg['release_margin_m'])+F(cfg['vehicle']['rear'])+F(cfg['s_min'])
        for i,c in enumerate(CARS):
            k=int(label[i]);center=cfg['initial'][c][:2] if k==0 else cfg['goals'][c][:2] if k==3 else (staging if c[0]=='A' else -staging,ys[k-1])
            p,t=map(F,radii[k])
            assert sum((pose8[c][j]-F(center[j])).abs_upper() for j in (0,1))<=p
            assert pose8[c][2].abs_upper()<=t and v[c]==delta[c]==0
    def goals():
        found=[]
        for c in CARS:
            g=cfg['goals'][c];p=pose8[c]
            goaltheta=m.I(F(g[2]))
            if c[0]=='B':goaltheta-=m.I.raw(m.I(F('3.14159265358979323846264338327950288')).l,m.I(F('3.14159265358979323846264338327950289')).u)
            if v[c]==delta[c]==0 and sum((p[j]-F(g[j])).abs_upper() for j in (0,1))<=F(cfg['terminal']['position_m']) and (p[2]-goaltheta).abs_upper()<=F(cfg['terminal']['yaw_rad']):found.append(c)
        return found
    if connection:
        edge=summary.get('edge')
        if edge is None:
            seed=coord['initial_policy'][coord['initial']]
            edge=next(e for e in coord['options'] if e['kind']==('S' if summary.get('stopped') else 'M') and e['motion']==seed['motion'] and e['source']==seed['source'])
        events={0:dict(edge=edge,budget_after=summary['source_budget'])}
        if summary.get('handoff_edge'):
            events[summary['first_connector_ticks']]=dict(edge=summary['handoff_edge'],budget_after=coord['initial_U'][edge['target']])
    else:events={e['cycle']:e for e in json.loads((run/'coordination_events.json').read_text())}
    budget=summary.get('source_budget',summary.get('initial_budget_ticks'));previous_edge=None;phase_expected=0
    done={};minroad=100*Q;minpair=100*Q;count=0;new_input_count=0;decoded_U=coord['initial_U'].copy();policy=coord['initial_policy'].copy()
    with gzip.open(run/'execution.jsonl.gz','rt',encoding='utf8') as stream:
        for row in map(json.loads,stream):
            if count in events:
                event=events[count];edge=event['edge'];cert=obj(edge['object'])
                if previous_edge is not None:
                    assert phase_expected==previous_edge['ticks'],'connection deadline reset before actual receipt'
                    assert edge['source']==previous_edge['target']
                node_contains(edge['source'],cert['stage_radii'])
                if not connection:
                    allowed=[e for e in coord['options'] if e['source']==edge['source'] and
                        (not summary['method'].startswith('S') or e['kind']=='S') and e['ticks']+decoded_U[e['target']]<=decoded_U[e['source']]]
                    T=summary['method'] in ('S-T','M-T','M-R')
                    expected=min(allowed,key=lambda e:((e['ticks']+decoded_U[e['target']]) if T else e['ticks'],e['actor'],e['target'],e['object']))
                    assert edge==expected,'selection differs from frozen objective'
                    old=decoded_U.copy();policy[edge['source']]=edge
                    for s in coord['topological_order']:
                        decoded_U[s]=0 if s==coord['goal'] else policy[s]['ticks']+decoded_U[policy[s]['target']]
                    assert all(decoded_U[s]<=old[s] for s in old)
                    assert event['continuation_U']==decoded_U and event['continuation_policy']==policy
                    assert event['budget_after']==edge['ticks']+decoded_U[edge['target']]
                assert event['budget_after']<=budget
                budget=event['budget_after'];phase_expected=0;previous_edge=edge
                planned=[tuple(map(F,u)) for u in cert['inputs'][:32]]
                planned=(planned+[(F(0),F(0))]*32)[:32]
                plan_id=sha(dict(object=edge['object'],phase=0,controls=planned))
                predicted=[pose4[edge['actor']]];vv=v[edge['actor']];dd=delta[edge['actor']]
                for aa,ww in planned:
                    if vv==dd==0 and aa==ww==0:predicted.append(predicted[-1])
                    else:predicted.append(m.propagate(predicted[-1],vv,vv+H*aa,dd,dd+H*ww,1 if edge['actor'][0]=='A' else -1,4)[-1])
                    vv+=H*aa;dd+=H*ww
                margins=[F(cert['road_lower_Q'],Q)]+[F(cert['peer_gap_lower_Q'][c+':'+edge['source'][i]],Q)-F(1,2)
                        for i,c in enumerate(CARS) if c!=edge['actor']]
                if cert['released_lower_Q'] is not None:margins.append(F(cert['released_lower_Q'],Q))
                reserve=min(margins)/2
            assert row['phase']==phase_expected
            assert row.get('receiver_phase',min(phase_expected+32,cert['ticks']))==min(phase_expected+32,cert['ticks'])
            assert row.get('remaining_deadline',min(32,cert['ticks']-phase_expected))==min(32,cert['ticks']-phase_expected)
            flat=[c for group in row['groups'] for c in group]
            assert len(flat)==4 and set(flat)==set(CARS),'invalid group partition'
            if not connection:
                assert row['policy_version']==event['U_version']
                assert row['policy_sha256']==sha(dict(policy=policy,U=decoded_U))
                assert row['receiver_time_value']==cert['ticks']-min(phase_expected+32,cert['ticks'])+decoded_U[edge['target']]
                assert budget==cert['ticks']-phase_expected+decoded_U[edge['target']]
            assert row.get('actor',edge['actor'])==edge['actor'] and row.get('object',edge['object'])==edge['object']
            actor=edge['actor'];assert all(v[c]==delta[c]==0 for c in CARS if c!=actor)
            assert row.get('parent_plan',plan_id)==plan_id
            if row['optimizer'].get('accepted') and row['optimizer'].get('changed'):
                assert 'accepted_controls' in row,'missing reconstructable accepted input sequence'
                new=[tuple(map(F,u)) for u in row['accepted_controls']];assert len(new)==32
                difference=[tuple(a-b for a,b in zip(n,o)) for n,o in zip(new,planned)]
                proof=row['cone_proof'];t=F(proof['norm'])
                assert sum(x*x for pair in difference for x in pair)<=t*t
                free=min(32,cert['ticks']-phase_expected)
                assert not any(any(pair) for pair in difference[free:])
                assert sum(pair[0] for pair in difference[:free])==sum(pair[1] for pair in difference[:free])==0
                assert sum(a*a+w*w for a,w in new)<=sum(a*a+w*w for a,w in planned)
                bnd=bounds();rp,rt=F(proof['rp']),F(proof['rpsi'])
                assert bnd['Gc']*t+bnd['Hc']*t*t/2<=reserve
                assert bnd['Gp']*t+bnd['Hp']*t*t/2<=rp and bnd['P1']*t+bnd['P2']*t*t/2<=rt
                endphase=min(phase_expected+32,cert['ticks']);point=predicted[free]
                if endphase==cert['ticks']:
                    center=cert['waypoints'][-1]
                    mp=F(cert['receiver_radius'][0])-sum((point[j]-F(center[j])).abs_upper() for j in (0,1))
                    mt=F(cert['receiver_radius'][1])-point[2].abs_upper();distance=F(0)
                else:
                    nom=tuple(m.I.raw(*r) for r in cert['pose_nodes_Q'][endphase]);org=tuple(m.I.raw(*r) for r in cert['pose_nodes_Q'][0])
                    theta=point[2]-nom[2];si,co=m.sincos(theta);dx,dy=nom[0]-org[0],nom[1]-org[1]
                    ep=[point[0]-nom[0]-(co-1)*dx+si*dy,point[1]-nom[1]-si*dx-(co-1)*dy]
                    mp=F(cert['source_radius'][0])-sum(x.abs_upper() for x in ep)
                    mt=F(cert['source_radius'][1])-theta.abs_upper();distance=dx.abs_upper()+dy.abs_upper()
                assert rp<=mp/4 and rt<=min(mt/4,mp/(4*(distance+1))),'receiver inner-box inclusion failed'
                assert t<=F(proof['scale'])==min(rp/(bnd['Gp']+1),rt/(bnd['P1']+1),reserve/(bnd['Gc']+1),F(1,100))/2
                inflated=[predicted[0]]
                for j in range(1,33):
                    if j>free:inflated.append(inflated[free]);continue
                    bj=bounds(j);er=bj['Gp']*t+bj['Hp']*t*t/2;et=bj['P1']*t+bj['P2']*t*t/2
                    def expanded(x,r):return x+m.I.raw(-m.I(r).u,m.I(r).u)
                    inflated.append((expanded(predicted[j][0],er),expanded(predicted[j][1],er),expanded(predicted[j][2],et)))
                predicted=inflated;reserve-=bnd['Gc']*t+bnd['Hc']*t*t/2
                assert row['committed_plan']==sha(dict(parent=plan_id,delta=difference))
                planned=new;new_input_count+=1
            else:
                assert row['committed_plan'] in (plan_id,sha(dict(parent=plan_id,delta=[[F(0),F(0)]]*32)))
            vv,dd=v[actor],delta[actor]
            stops=[j['cut']-phase_expected for j in cert['joins'] if 0<j['cut']-phase_expected<=32]
            stopped=summary.get('stopped',False) if connection else summary['method'].startswith('S')
            for j,(aa,ww) in enumerate(planned):
                vv+=H*aa;dd+=H*ww
                assert 0<=vv<=2 and abs(dd)<=F(1,2) and max(abs(aa),abs(ww))<=1
                if stopped and j+1 in stops:assert vv==0
            endphase=min(phase_expected+32,cert['ticks'])
            target=(tuple(map(F,(cert['knots'][endphase][0],cert['knots'][endphase][2]))) if endphase<cert['ticks'] else (F(0),F(0)))
            assert (vv,dd)==target,'inherited complete input/terminal rows failed'
            terminal_v,terminal_delta=vv,dd
            u=tuple(map(F,row['actual']['executed_input']));assert u==planned[0]
            a,w=u;assert max(abs(a),abs(w))<=1
            next_v=v[actor]+H*a;next_d=delta[actor]+H*w
            assert 0<=v[actor]<=2 and 0<=next_v<=2 and max(abs(delta[actor]),abs(next_d))<=F(1,2)
            sign=1 if actor[0]=='A' else -1
            nodes4=m.propagate(pose4[actor],v[actor],next_v,delta[actor],next_d,sign,4)
            assert [[x.l,x.u] for x in nodes4[-1]]==row['actual']['actual_pose_Q'],'actual state does not replay from recorded inputs'
            fine=m.propagate(pose8[actor],v[actor],next_v,delta[actor],next_d,sign,8)
            for left,right in zip(fine,fine[1:]):
                bodies={c:rectangle(left,c)+rectangle(right,c) if c==actor else rectangle(pose8[c],c) for c in CARS}
                boxes={c:box(bodies[c]) for c in CARS}
                for c,bb in boxes.items():
                    r=max(min(bb[0]-b[0],b[1]-bb[1],bb[2]-b[2],b[3]-bb[3]) for b in roads)-(epsilon if c==actor else 0)
                    assert r>=0,'continuous road violation';minroad=min(minroad,r)
                    if c!=actor:assert max(bb[0]-18*Q,-18*Q-bb[1])>=0,'permission violation'
                if cert['source']>0:
                    rear=boxes[actor][0] if sign==1 else -boxes[actor][1]
                    assert rear-epsilon>=int(F(53,2)*Q),'released car reentry'
                for ca,cb in itertools.combinations(CARS,2):
                    lower=separation(bodies[ca],bodies[cb],left if ca==actor else pose8[ca],left if cb==actor else pose8[cb])-(epsilon if actor in (ca,cb) else 0)
                    assert lower>=Q//2,'continuous six-pair violation';minpair=min(minpair,lower)
            pose4[actor]=nodes4[-1];pose8[actor]=fine[-1];v[actor]=next_v;delta[actor]=next_d
            assert str(next_v)==row['actual']['v'] and str(next_d)==row['actual']['delta']
            history=sha(dict(parent=history,actor=actor,input=u));assert history==row['history']
            assert row['budget_before']==budget and row['budget_after']==budget-1
            budget-=1;assert budget>=0
            count+=1;phase_expected+=1
            for c in goals():done.setdefault(c,str(count*H))
            for c in done:assert c in goals(),'goal was not maintained'
            if phase_expected==cert['ticks']:node_contains(edge['target'],cert['stage_radii'])
            if phase_expected<cert['ticks']:
                tail=tuple(map(F,cert['inputs'][phase_expected+31])) if phase_expected+31<cert['ticks'] else (F(0),F(0))
                if terminal_v==terminal_delta==0 and tail==(0,0):next_pose=predicted[-1]
                else:next_pose=m.propagate(predicted[-1],terminal_v,terminal_v+H*tail[0],terminal_delta,terminal_delta+H*tail[1],sign,4)[-1]
                predicted=predicted[1:]+[next_pose]
                planned=planned[1:]+[tail]
                plan_id=sha(dict(parent=row['committed_plan'],operation='shift',tail=tail,phase=phase_expected))
            if not connection:assert row['application_messages']==(44 if row['phase']==0 else 40) and row['application_bytes']>0
    if connection:
        assert count==summary.get('actual_steps',summary['first_connector_ticks']) and budget==summary['remaining_budget']
    else:
        assert set(done)==set(CARS) and budget==0 and done==summary['arrival_s']
        assert max(F(x) for x in done.values())==F(summary['T_all_s'])
    return dict(passed=True,actual_steps=count,changed_input_sequences_rebuilt=new_input_count,
        goals=done,full_task=not connection,remaining_budget=budget,
        continuous_road_lower_m=float(F(minroad,Q)),continuous_pair_lower_m=float(F(minpair,Q)),
        independent_geometric_substeps=8,online_state_replay_substeps=4,wall_s=time.perf_counter()-began,
        shared_primitives=['fixed-point interval arithmetic','Taylor trigonometry','outward propagation'],
        separate_checks=['rectangle construction','separating axes','road','permission','actual goal','budget and policy replay','actual input/state replay'],
        candidate_check_scope='complete affine input/state/terminal rows, exact shift, conic remainder and receiver-box inclusion; cone formation separately checked in P0')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('rebuild','objects','run'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--case',default='asymmetric_baseline');p.add_argument('--connection',action='store_true')
    args=p.parse_args();dest=args.run/'independent_acceptance.json'
    if dest.exists():p.error('acceptance evidence already exists')
    try:result=accept(args.rebuild,args.objects,args.run,args.case,args.connection)
    except Exception as error:
        result=dict(passed=False,error=repr(error));write(dest,result);raise
    write(dest,result);print(json.dumps(result,indent=2));return 0


if __name__=='__main__':raise SystemExit(main())
