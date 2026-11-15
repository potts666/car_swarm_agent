"""Independent 8-subsegment physical, progress, contract and raw-input reconstruction."""
import argparse,itertools,time
import numpy as np
from common import *
from audit_base import Audit
from physics import propagate
from progress import ETA,MOTION,chart
from generation import generate,data_for
from derivatives import verify
from socp_rows import rows_for,evaluate
from conic import assembly
from local_update import reserved_rows
from rules import quantize

class Independent(Audit):
    def geometry(self,nodes,owner):self.last_nodes=nodes;super().geometry(nodes,owner)
    def projected(self,p,ch):
        if ch['kind']=='hold':return m.I(0)
        origin=[m.I.raw(*q) for q in ch['origin']];phi=origin[2];si,co=(m.I(0),m.I(1)) if phi.l==phi.u==0 else m.sincos(phi);sg=direction(ch['car']);dx=p[0]-origin[0];dy=p[1]-origin[1];along=sg*(dx*co+dy*si);across=sg*(-dx*si+dy*co)
        if ch['kind']=='straight':assert across.abs_upper()<=F(1,5);return ch['task_sign']*along
        kappa=m.tan(m.I(F(ch['curvature'])))/m.L;den=m.I(1)-kappa*across;assert den.l>0;ratio=kappa*along/den;radius=ratio.abs_upper();assert radius<F(9,10);value=m.I(0);term=ratio
        for j in range(100):value+=term*F((-1)**j,2*j+1);term=term*ratio*ratio
        pad=m.I(radius**201/201).u;value=m.I.raw(value.l-pad,value.u+pad);sign=1 if F(ch['curvature'])>0 else -1
        return ch['task_sign']*sign*value/(sign*kappa)
    def progress_check(self,before,w,commands,record):
        active=[];moving=[]
        for c in CARS:
            v=before['v'][c];a,omega=commands[c];v1=v+H*a
            travel=(v*v+v1*v1)/(2*abs(a)) if v*v1<0 else H*(abs(v)+abs(v1))/2;ch=chart(w,c,before['cycle']);nodes=self.last_nodes[c]
            if ch['kind']=='hold':lo=hi=F(0)
            elif ch['kind']=='straight' and before['poses'][c][2].l==before['poses'][c][2].u==0 and before['delta'][c]==omega==0:lo=hi=ch['task_sign']*(H*v+H*H*a/2)
            else:
                inc=self.projected(nodes[-1],ch)-self.projected(nodes[0],ch);lo=F(inc.l,m.Q);hi=F(inc.u,m.Q)
            done=before['cycle']+1==w['goal_times'][c]
            if done:self.domain(c,'target')
            r=record[c];assert F(r['travel'])==travel and r['motion']==(travel>MOTION);assert r['chart_id']==content_id(ch) and r['mode']==ch['mode'];assert r['active']==(lo>=ETA or done),('independent activity',c,before['cycle'],str(lo),r['progress_lower'])
            if r['active']:active.append(c)
            if r['motion']:moving.append(c)
        return active,moving
    def face_check(self,cid,k):
        contract=get(HERE/'objects',cid);faces=get(HERE/'objects',contract['faces_id']);pairs=list(itertools.combinations(CARS,2));pad=TUBE+F(15)*(H/8)**2/8
        for sub,pairno,nx,ny,beta in faces[str(k)]:
            a,b=pairs[pairno];nx,ny,beta=map(F,(nx,ny,beta));assert nx*nx+ny*ny<=1
            for j in (2*sub,2*sub+1):
                aa=self.rect(self.last_nodes[a][j],a)+self.rect(self.last_nodes[a][j+1],a);bb=self.rect(self.last_nodes[b][j],b)+self.rect(self.last_nodes[b][j+1],b);pp=[x*nx+y*ny for x,y in aa];qq=[x*nx+y*ny for x,y in bb]
                assert F(max(p.u for p in pp),m.Q)+pad<=beta-SAFE/2,'new physical plane upper'
                assert F(min(q.l for q in qq),m.Q)-pad>=beta+SAFE/2,'new physical plane lower'

def reconstruct_event(audit,event):
    actual=audit.state;assert normal(state_record(actual))==event['actual_source'];contract=get(HERE/'objects',event['contract_id']);reference=copy.deepcopy(actual);reference.update(witness=contract['reference'],contract_id=event['contract_id']);assert normal(state_record(reference))==event['anchor_source'];old=get(HERE/'objects',actual['witness']);anchorplan=get(HERE/'objects',reference['witness']);k=actual['cycle']
    for c in CARS:assert anchorplan['controls'][c][:k]==old['controls'][c][:k]
    assert anchorplan['owners'][:k]==old['owners'][:k];assert max(anchorplan['goal_times'].values())<=max(old['goal_times'].values());expected=copy.deepcopy(anchorplan['controls']);events=copy.deepcopy(anchorplan['events']);checks=[]
    if event['physical_change']:
        assert contract['transition']['actual_source']==normal(state_record(actual));assert set(contract['transition']['closure'])==set(CARS);assert len(event['tokens'])==1 and set(event['tokens'][0]['request']['participants'])==set(CARS)
        from proposals import candidate
        rebuilt=candidate(old,actual,read(HERE/'pre_registration.json')['shifts']);assert rebuilt==anchorplan,'analytic wait/launch construction mismatch'
    for a in event['admissions']:
        job=a['job'];car=job['actor'];g0=get(HERE/'objects',job['generation_id']);anchor=get(HERE/'objects',job['anchor_id']);assert anchor['state']==normal(state_record(reference));g,gid=generate(reference,car);assert g['passed'] and g['anchor_id']==job['anchor_id'] and g['physical_rows_id']==g0['physical_rows_id'];cert=get(HERE/'objects',job['derivative_id']);verify(data_for(anchor),m,cert,anchor,True);rows=rows_for(data_for(anchor),m,anchor,cert,'B-JL');assert content_id(rows)==g0['physical_rows_id'];raw=get(HERE/'objects',a['raw_result_id']);assert raw['source']==normal(state_record(reference)) and raw['anchor_id']==job['anchor_id'];cfg=raw['settings'];assert cfg['tol_feas']==cfg['tol_gap_abs']==cfg['tol_gap_rel']==1e-10 and cfg['max_threads']==1
        inner,res=reserved_rows(rows);ref=np.asarray([float(F(x)) for u in anchor['plan']['remaining'][:32] for x in u]);model=get(HERE/'objects',job['assembly_id']);rebuilt=assembly(inner,ref,F(1,10000))[-1]
        for key in ('P','q','A','b','cone_dimensions'):assert model[key]==rebuilt[key],key
        p=a['candidate'];delta=[F(x)-F(y) for u,v in zip(p['controls'][:32],anchorplan['controls'][car][k:k+32]) for x,y in zip(u,v)]
        if a['accepted']:
            assert raw['status'] in ('Solved','AlmostSolved') and evaluate(rows,delta)['passed'];assert any((lambda q:q[1]['passed'] and q[0]==delta)(quantize(raw['x'],alpha,anchor['contracts'])) for alpha in (F(1),F(1,2),F(1,4)))
        assert p['controls'][33:]==anchorplan['controls'][car][k+33:] and p['atlas_id']==contract['atlas_id'] and p['obligations_id']==content_id(contract['obligations']);audit.inclusion(get(HERE/'objects',g0['receiver']['certificate_id']));checks.append(dict(actor=car,accepted=a['accepted'],status=raw['status'],zero_anchor=g['zero_anchor']['passed'],progress_rows=[r['name'] for r in rows if r['kind']=='progress'],fresh_source=True))
    from protocol import local_source
    for token in event['tokens']:
        q=token['request'];assert q['actual_source']==normal(state_record(actual)) and token['commit_id']==content_id(q)
        for c,p in q['plans'].items():
            assert p['source']==local_source(reference,c);expected[c][k:]=p['controls'];from receivers import replacement_events
            events=replacement_events(events,c,p['receiver'])
    new=get(HERE/'objects',event['selected_witness']);assert new['controls']==expected and new['events']==events and new['owners']==anchorplan['owners'];assert new['activity_contract_id']==event['contract_id'];return checks

def accept(arm,trace=None,source=None,write_result=True):
    began=time.perf_counter();reg=read(HERE/'pre_registration.json');out=HERE/'results'/arm if arm else None;summary=read(out/'summary.json') if out else None;events={e['cycle']:e for e in summary['events']} if summary else {};audit=Independent(reg['initial_witness']);audit.state['contract_id']=None
    if source:audit.state=restore(m,source);audit.p8=copy.deepcopy(audit.state['poses'])
    else:audit.validate_witness(reg['initial_witness'])
    trace=trace or out/'execution.jsonl.gz';checked=[];activities=[];minB=2802;first_phase=True;physical=None;count=0
    with gzip.open(trace,'rt') as stream:
        for row in map(json.loads,stream):
            s=audit.state;k=s['cycle'];assert row['cycle']==k and row['source']==normal(state_record(s))
            if k in events:checked.append(dict(cycle=k,checks=reconstruct_event(audit,events[k])));physical=events[k]['contract_id']
            before=copy.deepcopy(s);w=get(HERE/'objects',row['witness_id']);owner=w['owners'][k]
            if owner!=s['owner']:
                if s['owner']:audit.domain(s['owner'],'G_out')
                if owner:audit.domain(owner,'W_in');assert owner not in s['served'];s['served'].append(owner)
                s['permission_version']+=1
            assert row['permission_version']==s['permission_version'];old=get(HERE/'objects',before['witness']);b0={c:max(old['goal_times'][c]-k,0) for c in CARS};b1={c:max(w['goal_times'][c]-k-1,0) for c in CARS};assert row['b_before']==b0 and row['b_after']==b1 and row['B_after']==max(b1.values()) and row['B_after']==max(row['B_before']-1,0)
            for c,kind in w['events'].get(str(k),[]):
                if kind.startswith('generated:'):
                    audit.domain(c,kind);from receivers import feedback
                    assert tuple(map(F,w['controls'][c][k]))==feedback(s,c,get(HERE/'objects',kind.split(':')[1]))
            s.update(witness=row['witness_id'],owner=owner);commands={c:tuple(map(F,w['controls'][c][k])) for c in CARS};audit.step(commands,owner,row['actual']);active,moving=audit.progress_check(before,w,commands,row['progress'])
            if physical:audit.face_check(physical,k)
            elif arm=='R-old' and k>=reg['cycle']:audit.face_check(reg['old_contract'],k)
            for c,kind in w['events'].get(str(k+1),[]):audit.domain(c,kind)
            for c in CARS:
                if k+1>=w['goal_times'][c]:audit.domain(c,'target');assert s['v'][c]==s['delta'][c]==0
            cid=w.get('activity_contract_id')
            if cid:
                for c,quota in get(HERE/'objects',cid)['obligations'].get(str(k),{}).items():assert c in active,'remaining absolute-time quota lost'
            activities.append(dict(cycle=k,A=active,V=moving,O=row.get('O',[])));count+=1
    for c in CARS:audit.domain(c,'target')
    if summary:assert normal(state_record(audit.state))==summary['final'] and count==summary['steps']==2802
    result=dict(passed=True,arm=arm,steps=count,all_goals=True,B_final=0,checks=checked,activity_id=put(HERE/'objects',activities),minimum_road_m=float(audit.road),minimum_pair_m=float(audit.pair),wall_s=time.perf_counter()-began,final_history=audit.state['history'],shared=['integer interval/trig and signed ZOH','path-chart definitions, variational generator and cone compiler'],independent=['8-subsegment bodies/road/six pairs and physical support faces','100-term interval path projection and exact first-step progress/activity','actual history, analytic advance, raw-to-quantized commands, K/tails and deadlines'])
    if write_result:write(out/'independent_acceptance.json',result)
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--arm',required=True);r=accept(p.parse_args().arm);print(json.dumps(r,indent=2))
