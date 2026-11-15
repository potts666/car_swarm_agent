"""External replay: independent bodies, six pairs, authority, source, domain and budget."""
import argparse,itertools,time
import numpy as np
from common import *
from physics import propagate
from derivatives import verify,unpack
from socp_rows import rows_for
from conic import assembly


class Audit:
    def __init__(self,initial_witness):
        coords={'A1':(-35,3),'A2':(-39,-3),'B1':(37,3),'B2':(40,-3)}
        self.state=dict(poses={c:(m.I(x),m.I(y),m.I(0)) for c,(x,y) in coords.items()},v={c:F(0) for c in coords},delta={c:F(0) for c in coords},cycle=0,history=content_id(dict(initial=coords,model=sha(MODEL_PATH),h=F(1,5))),witness=initial_witness,owner=None,permission_version=0,served=[])
        self.p8=copy.deepcopy(self.state['poses']);self.road=self.pair=F(100);self.overlap=0;self.valid=set();self.domains=read(HERE/'domain_checks.json')['domains'];self.numeric=[]
    def clone(self):
        result=copy.copy(self);result.state=copy.deepcopy(self.state);result.p8=copy.deepcopy(self.p8);result.valid=self.valid.copy();result.numeric=self.numeric.copy();return result
    def rect(self,p,c):
        sn,cs=m.sincos(p[2]);s=direction(c)
        return [(p[0]+s*(cs*x-sn*y),p[1]+s*(sn*x+cs*y)) for x in (F(-1),F(7,2)) for y in (-F(9,10),F(9,10))]
    def domain(self,c,kind):
        if kind.startswith('generated:'):
            r=get(HERE/'objects',kind.split(':')[1])['domain'];center=[F(*x) for x in r['center_pose_Q']]
            for poses in (self.state['poses'],self.p8):
                assert sum((poses[c][j]-center[j]).abs_upper() for j in (0,1))<=F(r['position_l1_radius'])
                assert (poses[c][2]-center[2]).abs_upper()<=F(r['yaw_radius'])
            assert abs(self.state['v'][c]-F(r['center_v']))<=F(r['speed_radius']) and abs(self.state['delta'][c]-F(r['center_delta']))<=F(r['steering_radius']);return
        sign=direction(c);world_wait={'A1':(-30,3),'A2':(-30,-3),'B1':(30,3),'B2':(30,-3)}
        r=self.domains[kind];local=F(r.get('center_local_x',-22 if kind=='G_in' else 34 if kind=='G_out' else 46));xy=world_wait[c] if kind in ('W_in','W_hold') else (local*sign,0 if kind in ('G_in','G_in_shifted','G_out') else world_wait[c][1])
        for poses in (self.state['poses'],self.p8):
            assert sum((poses[c][j]-xy[j]).abs_upper() for j in (0,1))<=F(r['position_l1']),('independent domain position',c,kind)
            assert poses[c][2].abs_upper()<=F(r['yaw'])
        assert F(r['v'][0])<=self.state['v'][c]<=F(r['v'][1]) and F(r['delta'][0])<=self.state['delta'][c]<=F(r['delta'][1])
    def geometry(self,nodes,owner):
        roads=((-52,52,F(-7,5),F(7,5)),(-52,-10,F(-11,2),F(11,2)),(10,52,F(-11,2),F(11,2)))
        moving={c:any(pose_rows(p)!=pose_rows(nodes[c][0]) for p in nodes[c][1:]) for c in CARS};eps=F(15)*(H/8)**2/8;pad={c:F(1,20)+F(1,10**8)+(eps if moving[c] else 0) for c in CARS}
        for j in range(8):
            rects={c:self.rect(nodes[c][j],c)+self.rect(nodes[c][j+1],c) for c in CARS};boxes={c:(min(x.l for x,y in b),max(x.u for x,y in b),min(y.l for x,y in b),max(y.u for x,y in b)) for c,b in rects.items()}
            for c,b in boxes.items():
                lo,hi,bt,tp=map(lambda x:F(x,m.Q),b);margin=max(min(lo-a,z-hi,bt-bot,top-tp) for a,z,bot,top in roads)-pad[c];assert margin>=0,'independent road';self.road=min(self.road,margin)
                if c!=owner:assert hi+pad[c]<-16 or lo-pad[c]>16,'independent unreserved resource intersection'
            for c,d in itertools.combinations(CARS,2):
                b,e=boxes[c],boxes[d];gap=F(max(e[0]-b[1],b[0]-e[1],e[2]-b[3],b[2]-e[3]),m.Q)
                for pose in (nodes[c][j],nodes[d][j]):
                    theta=float(F(pose[2].l+pose[2].u,2*m.Q));nx=F(round(.999999*math.cos(theta)*10**6),10**6);ny=F(round(.999999*math.sin(theta)*10**6),10**6);assert nx*nx+ny*ny<=1
                    for ax,ay in ((nx,ny),(-ny,nx)):
                        pp=[x*ax+y*ay for x,y in rects[c]];qq=[x*ax+y*ay for x,y in rects[d]];gap=max(gap,F(min(t.l for t in qq)-max(t.u for t in pp),m.Q),F(min(t.l for t in pp)-max(t.u for t in qq),m.Q))
                gap-=pad[c]+pad[d];assert gap>=F(1,2),'independent six-pair separation';self.pair=min(self.pair,gap)
        if self.state['cycle']>=55 and sum(moving.values())>=2:self.overlap+=1
    def step(self,commands,owner,record=None):
        s=self.state;p4={};n8={};vv={};dd={}
        for c in CARS:
            a,w=commands[c];v,d=s['v'][c],s['delta'][c];nv,nd=v+H*a,d+H*w
            assert max(abs(a),abs(w))<=1 and -1<=v<=2 and -1<=nv<=2 and max(abs(d),abs(nd))<=F(1,2)
            sign=direction(c);p4[c]=propagate(s['poses'][c],v,nv,d,nd,sign,4)[-1];n8[c]=propagate(self.p8[c],v,nv,d,nd,sign,8);vv[c]=nv;dd[c]=nd
        self.geometry(n8,owner);self.p8={c:ns[-1] for c,ns in n8.items()};s.update(poses=p4,v=vv,delta=dd,cycle=s['cycle']+1,history=content_id(dict(parent=s['history'],commands=commands,owner=owner)))
        if record:
            assert record['commands']=={c:list(map(str,u)) for c,u in commands.items()} and record['poses_Q']=={c:pose_rows(p) for c,p in p4.items()}
            assert record['v']=={c:str(v) for c,v in vv.items()} and record['delta']=={c:str(d) for c,d in dd.items()} and record['history']==s['history']
    def expected_tables(self,w,parent):
        data=read(HERE/'rational_inputs.json');u={c:[['0','0'] for _ in range(2802)] for c in CARS};owner=[None]*2802;starts={};events={}
        for c in CARS:u[c][:len(data['preparation'][c])]=data['preparation'][c]
        for rank,c in enumerate(w['order']):
            start=55+663*rank;starts[c]=start;route=get(HERE/'objects',data['routes'][c]);u[c][start:start+758]=route['controls'];owner[start:start+663]=[c]*663
            for off,kind in ((0,'W_in'),(2,'W_hold'),(101,'G_in'),(663,'G_out'),(758,'target')):events.setdefault(str(start+off),[]).append([c,kind])
        if parent:
            rank=w['adoption_rank'];assert parent['order'][:rank]==w['order'][:rank]
            for c in w['order'][:rank]:
                u[c]=parent['controls'][c]
                for key in list(events):
                    events[key]=[e for e in events[key] if e[0]!=c]
                    if not events[key]:del events[key]
                for key,entries in parent['events'].items():
                    for entry in entries:
                        if entry[0]==c:events.setdefault(key,[]).append(entry)
        events={key:sorted(entries) for key,entries in events.items()}
        assert w['controls']==u and w['owners']==owner and w['starts']==starts and w['events']==events and w['goal_times']=={c:k+758 for c,k in starts.items()}
    def validate_witness(self,wid):
        if wid in self.valid:return
        w=get(HERE/'objects',wid);assert sorted(w['order'])==sorted(CARS) and w['model_sha256']==sha(MODEL_PATH) and w['ticks']==2802
        assert w['domain_certificate']==read(HERE/'domain_checks.json')['certificate_id']
        assert not w.get('optimization_origin'),'new numerical policy must be independently checked at its actual source'
        parent=get(HERE/'objects',w['parent_witness']) if w['parent_witness'] else None
        if parent:self.validate_witness(w['parent_witness'])
        self.expected_tables(w,parent);self.valid.add(wid)
    def numeric_update(self,event):
        from generation import anchor_for,data_for
        from receivers import synthesize,replacement_events
        from rules import quantize
        old=get(HERE/'objects',self.state['witness']);raw=get(HERE/'objects',event['raw_result_id']);pol=get(HERE/'objects',event['policy_result_id']);anchor=get(HERE/'objects',event['anchor_id']);cert=get(HERE/'objects',event['derivative_id']);actor=event['actor'];k=self.state['cycle']
        assert raw['history']==self.state['history']==event['source_history'] and raw['source_witness']==self.state['witness']==event['parent_witness']
        assert anchor['state']==normal(state_record(self.state)) and anchor_for(self.state,actor)==anchor
        assert raw['anchor_id']==event['anchor_id'] and raw['derivative_id']==event['derivative_id'] and raw['assembly_id']==event['assembly_id']
        assert raw['generation_id']==event['generation_id'] and get(HERE/'objects',raw['generation_id'])['anchor_id']==event['anchor_id']
        assert pol['raw_result_id']==event['raw_result_id'] and pol['admission_policy']=='certificate_suboptimal_v1' and pol['accepted']==event['accepted']
        cfg=raw['settings'];assert raw['solver_version']=='0.11.1' and cfg['tol_feas']==cfg['tol_gap_abs']==cfg['tol_gap_rel']==1e-10 and cfg['time_limit']==5 and cfg['max_iter']==300
        assert raw['phase']==k and raw['round']==event['round']==1
        verify(data_for(anchor),m,cert,anchor,recompute=True);rows=rows_for(data_for(anchor),m,anchor,cert,'B-JL');model=get(HERE/'objects',raw['assembly_id']);assert content_id(model['physical_rows'])==content_id(rows)
        # Independent terminal Taylor faces, including nonzero yaw and signed speed.
        domain=anchor['receiver']['domain'];center=[F(*x) for x in domain['center_pose_Q']];J=unpack(m,cert['prefix_jacobians'][-1]);p=[m.I.raw(*x) for x in cert['terminal_state']['poses_Q'][actor]];prof=cert['local_profiles'][-1];expected={}
        for sx,sy in itertools.product((-1,1),repeat=2):expected[f'position_{sx}_{sy}']=(F((sx*(p[0]-center[0])+sy*(p[1]-center[1])).u,m.Q)-F(domain['position_l1_radius']),[sx*J[0][j]+sy*J[1][j] for j in range(64)],F(3,2)*F(prof['Hp']))
        for sx in (-1,1):expected[f'yaw_{sx}']=(F((sx*(p[2]-center[2])).u,m.Q)-F(domain['yaw_radius']),[sx*x for x in J[2]],F(prof['P2']))
        for row in rows:
            if row['kind']=='terminal':
                g,jac,hess=expected[row['name']];assert F(row['g_upper'])==g and list(map(F,row['linear']))==[F(t.l+t.u,2*m.Q) for t in jac]
                assert F(row['gradient_error'])==norm_upper([F(t.u-t.l,2*m.Q) for t in jac]) and F(row['hessian'])==hess
        inner=[];e=F(4,10**12)
        for row in rows:
            eta=(norm_upper(row['linear'])+F(row['gradient_error'])+F(row['hessian'])*RHO)*e+F(row['hessian'])*e*e/2;rr=copy.deepcopy(row);rr['g_upper']=F(rr['g_upper'])+(eta if F(rr['g_upper'])+eta<=0 else 0);inner.append(rr)
        ref=np.asarray([[float(F(x)) for x in u] for u in old['controls'][actor][k:k+32]]).ravel();rebuilt=assembly(inner,ref,F(1,10000))[-1]
        for key in ('P','q','A','b','cone_dimensions'):assert model[key]==rebuilt[key]
        receiver=get(HERE/'objects',anchor['receiver']['certificate_id']);assert receiver['old_witness']==self.state['witness'] and receiver['source']==normal(state_record(self.state))
        self.inclusion(receiver)
        for attempt in pol['attempts']:
            delta,proof=quantize(raw['x'],F(attempt['alpha']),anchor['contracts']);assert list(map(F,attempt['delta']))==delta and attempt['quantization']==proof
            norm=norm_upper(delta)
            violations=[row['name'] for row in rows if F(row['g_upper'])+sum(F(a)*b for a,b in zip(row['linear'],delta))+F(row['gradient_error'])*norm+F(row['hessian'])*norm*norm/2>0]
            assert attempt['cone']['violations']==violations
        if not pol['accepted']:
            assert event['accepted_witness']==self.state['witness'];self.numeric.append(dict(actor=actor,phase=k,passed=True,accepted=False,status=raw['status']));return
        assert raw['status'] in ('Solved','AlmostSolved') and all(math.isfinite(x) for name in ('x','s','z') for x in raw[name]);attempt=pol['attempts'][-1];assert attempt['passed'] and attempt['quantization']['passed'] and not attempt['cone']['violations']
        delta=list(map(F,attempt['delta']));wid=event['accepted_witness'];new=get(HERE/'objects',wid);assert pol['accepted_witness']==wid and new['optimization_origin']==event['raw_result_id'] and new['parent_witness']==self.state['witness']
        for c in CARS:
            if c==actor:
                assert new['controls'][c][:k]==old['controls'][c][:k] and new['controls'][c][k+33:]==old['controls'][c][k+33:]
                assert [F(a)-F(b) for u,v in zip(new['controls'][c][k:k+32],old['controls'][c][k:k+32]) for a,b in zip(u,v)]==delta
            else:assert new['controls'][c]==old['controls'][c]
        for name in ('order','owners','starts','goal_times','ticks','domain_certificate'):assert new[name]==old[name]
        assert new['events']==replacement_events(old['events'],actor,anchor['receiver']) and new['receiver_contract']==anchor['receiver']
        assert H*sum(F(x)**2 for u in new['controls'][actor][k:k+32] for x in u)<=H*sum(F(x)**2 for u in old['controls'][actor][k:k+32] for x in u)
        trial=self.clone()
        for j in range(receiver['retained_target_phase']-k):
            if j==32:
                trial.domain(actor,anchor['receiver']['kind']);u=tuple(map(F,receiver['reference_K']));K=(u[0]-(trial.state['v'][actor]-F(domain['center_v']))/H,u[1]-(trial.state['delta'][actor]-F(domain['center_delta']))/H);assert tuple(map(F,new['controls'][actor][k+32]))==K
            trial.step({c:tuple(map(F,new['controls'][c][k+j])) for c in CARS},new['owners'][k+j])
            if j==32:assert trial.state['v'][actor]==F(receiver['reset_v']) and trial.state['delta'][actor]==F(receiver['reset_delta'])
        trial.domain(actor,receiver['retained_target_kind']);self.valid.add(wid);self.numeric.append(dict(actor=actor,phase=k,passed=True,accepted=True,status=raw['status']))
    def inclusion(self,receiver):
        # Rebuild the positive error coefficients and allocation, without controller flags.
        C=F(3,2);c=receiver['constants'];V=F(c['V']);L=F(c['L_rem']);kap=F(c['kappa']);kd=F(c['kappa_delta']);r=receiver['domain'];radii=[F(r[k]) for k in ('position_l1_radius','yaw_radius','speed_radius','steering_radius')];baseline=list(map(F,receiver['baseline']))
        def coeff(d):return [F(1),C*V*H+d,C*H/2+C*V*H**2*kap/3+d*H*kap/2,C*V*H**2*V*kd/3+d*H*V*kd/2]
        expected=[coeff(C*L),[F(0),F(1),H*kap/2,H*V*kd/2],coeff(C*L+RB),[F(0),F(1),H*kap/2,H*V*kd/2]]
        for row,a in zip(receiver['allocation'],expected):
            assert list(map(F,row['a']))==a and sum(x*y for x,y in zip(a,radii))<=F(row['b']) and F(row['remaining'])==F(row['b'])-sum(x*y for x,y in zip(a,baseline))
        assert all(a>=b for a,b in zip(radii,baseline))
        assert receiver['prefix_ticks']+receiver['suffix_ticks']==receiver['budget_before']==2802-receiver['source']['cycle']
    def frame(self,row):
        s=self.state;k=s['cycle'];assert row['cycle']==k and row['parent_plan_id']==content_id(dict(witness=s['witness'],cycle=k,history=s['history'],remaining=2802-k))
        wid=row['witness_id'];self.validate_witness(wid);w=get(HERE/'objects',wid);ctx=row['token']['context'];owner=w['owners'][k];version=s['permission_version']+int(owner!=s['owner'])
        assert ctx['source_snapshot']==normal(state_record(s)) and ctx['history']==s['history'] and ctx['cycle']==k and ctx['permission_version']==version and ctx['owner']==owner
        assert ctx['resource_id']==content_id(dict(road=ROADS,x_abs_max=16)) and ctx['certificate_id']==w['domain_certificate'] and ctx['witness_id']==wid and ctx['parent_plan_id']==row['parent_plan_id'] and ctx['remaining_budget']==2802-k and ctx['receiver_chain']==w['events']
        assert ctx['plan_id']==content_id(dict(witness=wid,cycle=k,history=s['history'],remaining=2802-k))
        if owner!=s['owner']:
            if s['owner']:
                self.domain(s['owner'],'G_out');c=s['owner'];rect=self.rect(self.p8[c],c);near=min(F(x.l,m.Q) for x,y in rect) if c[0]=='A' else -max(F(x.u,m.Q) for x,y in rect);assert near-TUBE>16
            if owner:assert owner not in s['served'];self.domain(owner,'W_in')
        ids={}
        for c in CARS:
            st=w['starts'][c];action='initial_preparation' if k<55 else 'core_reservation' if owner==c else 'outside_exit_tail' if st+663<=k<st+758 else 'hold';assert ctx['actions'][c]==action
            commands=w['controls'][c][k:k+32]+[['0','0']]*max(0,32-(2802-k))
            ids[c]=content_id(dict(car=c,context=ctx,controls=commands,permission=action,source_pose_Q=pose_rows(s['poses'][c]),v=str(s['v'][c]),delta=str(s['delta'][c]),certificate_id=w['domain_certificate']))
            if action=='hold':assert s['v'][c]==s['delta'][c]==0 and w['controls'][c][k]==['0','0']
        token=row['token'];assert token['plans']==ids and token['application_messages']==40 and token['commit_id']==content_id(dict(context=ctx,plans=ids))
        if owner!=s['owner'] and owner:s['served'].append(owner)
        s.update(owner=owner,permission_version=version,witness=wid);self.step({c:tuple(map(F,w['controls'][c][k])) for c in CARS},owner,row['actual'])
        for c,kind in w['events'].get(str(k+1),[]):self.domain(c,kind)
        assert row['budget_before']==2802-k and row['budget_after']==2801-k and row['shifted_plan_id']==content_id(dict(witness=wid,cycle=k+1,history=s['history'],remaining=2801-k))


def accept(run):
    began=time.perf_counter();cfg=read(run/'config.json');summary=read(run/'summary.json');audit=Audit(cfg['initial_witness']);fork=None;events=[];choices=[]
    with gzip.open(run/'execution.jsonl.gz','rt') as stream:
        for row in map(json.loads,stream):
            event=row.get('event')
            if event and event['kind']=='local_socp':audit.numeric_update(event)
            if event and event['kind']=='resource_choice':
                eligible=[c for c in CARS if c not in audit.state['served']];assert event['choice']['eligible']==eligible
                previous=choices[-1] if choices else None;preferred=[c for c in eligible if previous and (direction(c)!=direction(previous))==(cfg['selection_case']=='opposite')];chosen=min(preferred or eligible);assert event['choice']['chosen']==chosen;choices.append(chosen)
                for c in eligible:audit.domain(c,'W_in')
            audit.frame(row)
            if event:assert event['commit_id']==row['token']['commit_id'];events.append(event)
    for c in CARS:audit.domain(c,'target')
    assert summary['passed'] and audit.state['cycle']==summary['actual_steps']==2802 and summary['remaining_budget']==0 and audit.overlap==summary['overlap_ticks']>0
    assert audit.state['history']==summary['final']['history'] and choices==[c['chosen'] for c in summary['selection']] and len(audit.numeric)==1
    return dict(passed=True,actual_steps=2802,logical_time_s='2802/5',remaining_budget=0,selection=choices,actual_overlap_ticks=audit.overlap,accepted_numeric_candidates=sum(e['accepted'] for e in audit.numeric),reverse_new_candidate_accepted=all(e['accepted'] for e in audit.numeric),minimum_road_m=float(audit.road),minimum_pair_m=float(audit.pair),local_socp=audit.numeric,version_rejection=fork,all_four_goals=True,wall_s=time.perf_counter()-began,shared=['integer intervals/trigonometry and exact ZOH propagation','variational generator and cone compiler','immutable configuration/identity primitives'],independent=['bodies/road/six pairs, 8-subsegment continuous envelopes','source snapshots, four-public-plan IDs and authority versions','waiting/entry/exit/goal domain membership','completion table construction, raw-to-rational commands, terminal Taylor rows','actual resource clear, outside-tail actions, same-old-witness rejection replay, budgets'])


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--case',choices=('A','B'),required=True);args=parser.parse_args();run=HERE/'results'/args.case;dest=run/'independent_acceptance.json';assert not dest.exists()
    try:result=accept(run)
    except Exception as error:write(dest,dict(passed=False,error=repr(error)));raise
    write(dest,result);print(json.dumps(result,indent=2))
if __name__=='__main__':main()
