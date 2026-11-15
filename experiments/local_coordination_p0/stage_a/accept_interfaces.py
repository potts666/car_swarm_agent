#!/usr/bin/env python3
"""Controller-external Stage A replay; separate geometry, reset and suffix checks."""
import argparse
import copy
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
def read(p):return json.loads(Path(p).read_text())
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def identity(x):return hashlib.sha256(json.dumps(x,sort_keys=True,separators=(',',':'),default=str).encode()).hexdigest()
def commands(p):return [tuple(map(F,u)) for u in p['remaining']]
def pad(u):return u[:32]+[(F(0),F(0))]*max(0,32-len(u))
def packed(p):return [[x.l,x.u] for x in p]
def plan_id(p):return identity({k:v for k,v in p.items() if k!='id'})


class Replay:
    def __init__(self,m,cfg):
        self.m=m;self.cfg=cfg;self.v={c:F(0) for c in CARS};self.d=self.v.copy();self.p4={};self.p8={}
        for c in CARS:
            z=cfg['initial'][c];theta=m.I(F(z[2]))
            if c[0]=='B':theta-=m.I.raw(m.I(F('3.14159265358979323846264338327950288')).l,m.I(F('3.14159265358979323846264338327950289')).u)
            self.p4[c]=(m.I(F(z[0])),m.I(F(z[1])),theta);self.p8[c]=self.p4[c]
        self.history=identity(dict(initial=cfg['initial'],model='bicycle-ZOH-.2-v1'))
        self.road=self.pair=100*m.Q

    def clone(self):
        new=copy.copy(self);new.v=self.v.copy();new.d=self.d.copy();new.p4=self.p4.copy();new.p8=self.p8.copy();return new

    def rectangle(self,p,c):
        si,co=self.m.sincos(p[2]);sign=1 if c[0]=='A' else -1
        return [(p[0]+sign*(x*co-y*si),p[1]+sign*(x*si+y*co)) for x in (F(-1),F(7,2)) for y in (F(-9,10),F(9,10))]

    def step(self,c,u,released,record=None):
        m=self.m;Q=m.Q;a,w=u;nv=self.v[c]+H*a;nd=self.d[c]+H*w
        assert max(abs(a),abs(w))<=1 and 0<=self.v[c]<=2 and 0<=nv<=2 and max(abs(self.d[c]),abs(nd))<=F(1,2)
        assert all(self.v[o]==self.d[o]==0 for o in CARS if o!=c)
        sign=1 if c[0]=='A' else -1
        p4=m.propagate(self.p4[c],self.v[c],nv,self.d[c],nd,sign,4)[-1]
        nodes=m.propagate(self.p8[c],self.v[c],nv,self.d[c],nd,sign,8)
        epsilon=m.I(15*(H/8)**2/8).u
        def bounds(body):return min(x.l for x,y in body),max(x.u for x,y in body),min(y.l for x,y in body),max(y.u for x,y in body)
        roads=[[m.I(F(x)).l for x in r] for r in self.cfg['road_rectangles']]
        for left,right in zip(nodes,nodes[1:]):
            bodies={o:self.rectangle(left,o)+self.rectangle(right,o) if o==c else self.rectangle(self.p8[o],o) for o in CARS}
            boxes={o:bounds(b) for o,b in bodies.items()}
            for o,b in boxes.items():
                err=epsilon if o==c else 0
                margin=max(min(b[0]-r[0],r[1]-b[1],b[2]-r[2],r[3]-b[3]) for r in roads)-err
                assert margin>=0,'independent road';self.road=min(self.road,margin)
                if o!=c:assert max(b[0]-18*Q,-18*Q-b[1])>=0,'independent waiting protection'
            if released:
                rear=boxes[c][0] if sign==1 else -boxes[c][1]
                assert rear-epsilon>=F(53,2)*Q,'independent release guard'
            for x,y in itertools.combinations(CARS,2):
                bx,by=boxes[x],boxes[y];gap=max(by[0]-bx[1],bx[0]-by[1],by[2]-bx[3],bx[2]-by[3])
                if gap<Q//2+epsilon:
                    for pose in (left if x==c else self.p8[x],left if y==c else self.p8[y]):
                        angle=F(pose[2].l+pose[2].u,2*Q)
                        nx=F(round(.999999*math.cos(float(angle))*10**6),10**6);ny=F(round(.999999*math.sin(float(angle))*10**6),10**6)
                        assert nx*nx+ny*ny<=1
                        for ax,ay in ((nx,ny),(-ny,nx)):
                            px=[a*ax+b*ay for a,b in bodies[x]];py=[a*ax+b*ay for a,b in bodies[y]]
                            gap=max(gap,min(t.l for t in py)-max(t.u for t in px),min(t.l for t in px)-max(t.u for t in py))
                gap-=epsilon if c in (x,y) else 0
                assert gap>=Q//2,'independent six-pair gap';self.pair=min(self.pair,gap)
        self.p4[c]=p4;self.p8[c]=nodes[-1];self.v[c]=nv;self.d[c]=nd
        self.history=identity(dict(parent=self.history,actor=c,input=u))
        if record is not None:
            assert record['executed_input']==list(map(str,u)) and record['actual_pose_Q']==packed(p4)
            assert F(record['v'])==nv and F(record['delta'])==nd

    def box(self,c,row):
        d=row['domain'];center=[F(*p) for p in d['center_pose_Q']]
        for poses in (self.p4,self.p8):
            assert sum((poses[c][j]-center[j]).abs_upper() for j in (0,1))<=F(d['position_l1_radius'])
            assert (poses[c][2]-center[2]).abs_upper()<=F(d['yaw_radius'])
        assert abs(self.v[c]-F(d['center_v']))<=F(d['speed_radius'])
        assert abs(self.d[c]-F(d['center_delta']))<=F(d['steering_radius'])

    def flow(self,c,cert,k):
        nom=[self.m.I.raw(*p) for p in cert['pose_nodes_Q'][k]];origin=[self.m.I.raw(*p) for p in cert['pose_nodes_Q'][0]]
        P,T=map(F,cert['source_radius'])
        for poses in (self.p4,self.p8):
            p=poses[c];theta=p[2]-nom[2];si,co=self.m.sincos(theta);dx,dy=nom[0]-origin[0],nom[1]-origin[1]
            e=[p[0]-nom[0]-(co-1)*dx+si*dy,p[1]-nom[1]-si*dx-(co-1)*dy]
            assert sum(q.abs_upper() for q in e)<=P and theta.abs_upper()<=T
        assert self.v[c]==F(cert['knots'][k][0]) and self.d[c]==F(cert['knots'][k][2])

    def node(self,label,radii):
        ys=sorted({F(z[1]) for z in self.cfg['goals'].values()},reverse=True)
        x=F(self.cfg['release_plane_abs_x'])+F(self.cfg['release_margin_m'])+F(self.cfg['vehicle']['rear'])+F(self.cfg['s_min'])
        for i,c in enumerate(CARS):
            k=int(label[i]);center=self.cfg['initial'][c][:2] if k==0 else self.cfg['goals'][c][:2] if k==3 else (x if c[0]=='A' else -x,ys[k-1])
            P,T=map(F,radii[k])
            for poses in (self.p4,self.p8):assert sum((poses[c][j]-F(center[j])).abs_upper() for j in (0,1))<=P and poses[c][2].abs_upper()<=T
            assert self.v[c]==self.d[c]==0


def accept(inputs,rebuild,run):
    start=time.perf_counter();manifest=read(inputs/'manifest.json')
    assert all(digest(inputs/n)==h for n,h in manifest['files'].items())
    cfg=read(inputs/'config.json');w=read(inputs/'witness.json');rows={r['phase']:r for r in read(inputs/'interfaces.json')}
    chain=w['chain'];assert chain[0]['source']=='0000' and chain[-1]['target']=='3333'
    objects={e['object']:read(inputs/'objects'/(e['object']+'.json')) for e in chain}
    for i,e in enumerate(chain):
        assert digest(inputs/'objects'/(e['object']+'.json'))==e['object']
        assert objects[e['object']]['ticks']==len(objects[e['object']]['inputs'])==e['ticks']
        assert w['U'][e['source']]==sum(x['ticks'] for x in chain[i:])
        if i:assert chain[i-1]['target']==e['source']
    assert w['accepted_budget']==sum(e['ticks'] for e in chain)<=w['old_budget']
    edge=chain[0];cert=objects[edge['object']];c=edge['actor'];witness=identity(w)
    spec=importlib.util.spec_from_file_location('stage_a_accept_model',rebuild/'work/M2b_首段自由度与递归可行性/route_zoh_certificate.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    audit=Replay(m,cfg);audit.node(edge['source'],cert['stage_radii']);config=read(run/'config.json');summary=read(run/'summary.json')
    assert config['manifest_sha256']==digest(inputs/'manifest.json') and config['source_budget']==w['accepted_budget']
    expected=[tuple(map(F,u)) for u in cert['inputs']];budget=w['accepted_budget'];previous=None;old_id=None;accepted=rejected=reset_count=handoffs=shiftK=0;pending=None

    def validate(p,phase,base,parent):
        r=p['receiver'];u=commands(p);end=min(phase+32,cert['ticks']);length=end-phase
        assert plan_id(p)==p['id'] and p['object']==edge['object']
        assert p['phase']==phase and p['source_history']==audit.history and p['source_pose_Q']==packed(audit.p4[c])
        assert F(p['v0'])==audit.v[c] and F(p['delta0'])==audit.d[c]
        assert len(u)==cert['ticks']-phase and p['budget']==len(u)+w['U'][edge['target']]==budget
        assert p['witness']==witness and p['witness_version']==w['version'] and p['suffix_ticks']==w['U'][edge['target']]
        assert r['phase']==end and r['deadline']==length and r['domain_object']==edge['object']
        if p['id']==parent:
            assert u==base and r['component']==('flow' if end<cert['ticks'] else 'coarse_stopped')
            vv=audit.v[c]+H*sum(a for a,d in pad(u));dd=audit.d[c]+H*sum(d for a,d in pad(u))
            reference=cert['knots'][end] if end<cert['ticks'] else ['0']*4
            assert (vv,dd)==(F(reference[0]),F(reference[2]))
            return
        assert p['parent']==parent and p['branch']=='box_candidate' and r['component']=='box' and length==32
        row=rows[end];d=row['domain'];rr=p['pending_reset']
        assert r['domain']==d and r['row_sha256']==identity(row)
        assert rr['phase']==end and rr['next_phase']==end+1 and rr['row_sha256']==identity(row)
        assert u[33:]==base[33:] and sum(a*a+b*b for a,b in u[:32])<=sum(a*a+b*b for a,b in base[:32])
        v=audit.v[c]+H*sum(a for a,b in u[:32]);delta=audit.d[c]+H*sum(b for a,b in u[:32])
        K=(F(d['feedback_a_ref'])-(v-F(d['center_v']))/H,F(d['feedback_omega_ref'])-(delta-F(d['center_delta']))/H)
        assert tuple(map(F,rr['input']))==u[32]==K,'independent K binding'
        clone=audit.clone()
        for j,command in enumerate(u[:33]):
            if j==32:clone.box(c,row)
            clone.step(c,command,cert['source']>0)
        clone.flow(c,cert,end+1)

    count=0
    with gzip.open(run/'execution.jsonl.gz','rt') as stream:
        for row in map(json.loads,stream):
            phase=row['phase'];assert row['cycle']==count
            if row['kind']=='parked_handoff':
                assert count==cert['ticks'] and row['edge']==chain[1] and row['actor']==chain[1]['actor']!=c
                audit.node(edge['target'],cert['stage_radii']);nextcert=objects[chain[1]['object']]
                assert row['token']['context']['physical_budget']==budget
                command=tuple(map(F,nextcert['inputs'][0]));audit.step(row['actor'],command,nextcert['source']>0,row['actual']);handoffs+=1
            else:
                assert phase==count and row['actor']==c
                plan=row['plan'];parent=row['old_plan_id']
                if previous is not None:assert parent==old_id
                else:assert plan['branch']=='locked_inherited' and parent==plan['id'] and plan['ancestry']==[]
                bad=row['rejected']
                if bad:
                    assert plan['id']==parent and bad['old_plan_id']==parent and tuple(map(F,bad['old_first_input']))==expected[0]
                    try:validate(bad['proposal'],phase,expected,parent)
                    except (AssertionError,ValueError,KeyError):rejected+=1
                    else:raise AssertionError('independent acceptor accepted corrupted proposal')
                validate(plan,phase,expected,parent)
                if plan['id']!=parent:
                    accepted+=1;pending=plan['pending_reset'];assert phase==config['prefix_start']
                expected=commands(plan);token=row['token'];ctx=token['context'];other=[x for x in CARS if x!=c]
                partition=[[c,other[0]],[other[1]],[other[2]]] if phase%64==0 else [[c,other[0],other[1]],[other[2]]] if phase%64==1 else [[c],[other[0],other[1]],[other[2]]]
                assert ctx['groups']==partition and set(token['plans'])==set(CARS) and token['application_messages']==40
                assert ctx['history']==audit.history and ctx['budget']==budget and ctx['receiver']==plan['receiver'] and ctx['pending_reset']==plan['pending_reset'] and ctx['plan_id']==plan['id']
                assert ctx['witness']==witness and ctx['witness_version']==w['version'] and ctx['owner']==c
                assert ctx['joint_source']==edge['source'] and ctx['joint_target']==edge['target'] and ctx['permission_version']==w['version']
                assignment={car:j for j,group in enumerate(partition) for car in group}
                pairs=list(itertools.combinations(CARS,2))
                assert ctx['pair_contracts']==dict(inside=[list(p) for p in pairs if assignment[p[0]]==assignment[p[1]]],boundary=[list(p) for p in pairs if assignment[p[0]]!=assignment[p[1]]])
                if pending and phase==pending['phase']:
                    rr=rows[phase];audit.box(c,rr)
                    assert audit.v[c]!=F(rr['nominal_v']) and audit.d[c]!=F(rr['nominal_delta'])
                    assert summary['actual_box_state']==dict(pose_Q=packed(audit.p4[c]),v=str(audit.v[c]),delta=str(audit.d[c]))
                    d=rr['domain'];liveK=(F(d['feedback_a_ref'])-(audit.v[c]-F(d['center_v']))/H,F(d['feedback_omega_ref'])-(audit.d[c]-F(d['center_delta']))/H)
                    assert expected[0]==liveK
                audit.step(c,expected[0],cert['source']>0,row['actual'])
                if pending and phase==pending['phase']:
                    audit.flow(c,cert,phase+1);reset_count+=1
                    assert summary['actual_K_reset_state']==row['actual']
                nextplan=copy.deepcopy(plan);newphase=phase+1
                nextplan.update(branch='locked_derived_suffix' if plan['ancestry'] else 'locked_inherited',phase=newphase,parent=plan['id'],remaining=plan['remaining'][1:],budget=budget-1,source_history=audit.history,source_pose_Q=packed(audit.p4[c]),v0=audit.v[c],delta0=audit.d[c],receiver=dict(component='flow' if min(newphase+32,cert['ticks'])<cert['ticks'] else 'coarse_stopped',phase=min(newphase+32,cert['ticks']),domain_object=edge['object'],deadline=min(32,cert['ticks']-newphase)))
                if plan['pending_reset'] is not None and plan['pending_reset']['phase']<newphase:nextplan['pending_reset']=None
                nextplan['id']=plan_id(nextplan);assert row['shift']['new_id']==nextplan['id'] and row['shift']['parent']==plan['id']
                tail=expected[32] if len(expected)>32 else (F(0),F(0));assert tuple(map(F,row['shift']['appended_input']))==tail
                assert pad(expected[1:])==pad(expected)[1:]+[tail]
                if row['shift']['kind']=='saved_state_dependent_K':assert phase==config['prefix_start'] and tail==tuple(map(F,pending['input']));shiftK+=1
                expected=expected[1:];old_id=nextplan['id'];previous=plan
            assert row['budget_before']==budget and row['budget_after']==budget-1 and row['history']==audit.history
            budget-=1;count+=1
    assert accepted==reset_count==shiftK==handoffs==1 and rejected==6
    assert count==summary['actual_steps']==cert['ticks']+1 and budget==summary['remaining_budget']==w['U'][edge['target']]-1
    assert summary['passed'] and summary['duration_changed'] is False
    return dict(passed=True,actual_steps=count,accepted_box_candidates=accepted,actual_K_resets=reset_count,actual_saved_K_shifts=shiftK,
        damaged_proposals_independently_rejected=rejected,same_old_first_input_replayed_after_every_rejection=True,
        parked_handoffs_executed=handoffs,current_same_version_completion_witness_rebuilt=True,remaining_budget=budget,
        minimum_road_lower_m=float(F(audit.road,m.Q)),minimum_pair_lower_m=float(F(audit.pair,m.Q)),
        wall_s=time.perf_counter()-start,shared_primitives=['fixed-point interval arithmetic','trigonometry','propagation'],
        separate_checks=['box membership','live K','inverse flow containment','actual tail/shift','geometric bodies, roads, six pairs and permissions','witness/budget'],
        full_fourcar_task=False,derivative_bounds_connected=False)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('inputs','rebuild','run'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();dest=a.run/'independent_acceptance.json'
    if dest.exists():p.error('preserve old acceptance; use a fresh run')
    try:value=accept(a.inputs,a.rebuild,a.run)
    except Exception as error:
        dest.write_text(json.dumps(dict(passed=False,error=repr(error)),indent=2)+'\n');raise
    dest.write_text(json.dumps(value,indent=2)+'\n');print(json.dumps(value,indent=2))


if __name__=='__main__':
    if not __debug__:raise SystemExit('certification requires Python assertions')
    main()
