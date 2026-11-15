"""Independent reconstruction using the old independent 8-subsegment geometry audit."""
import argparse,time,itertools
import numpy as np
from common import *
from audit_base import Audit
from generation import generate,data_for
from derivatives import verify
from conic import assembly
from rules import quantize
from socp_rows import rows_for,evaluate
from local_update import reserved_rows
from strategy import deadlines

def event_check(audit,event):
    s=audit.state;s['contract_id']=event['source']['contract_id'];assert normal(state_record(s))==event['source'],'actual history/source mismatch';old=get(HERE/'objects',s['witness']);k=s['cycle'];new=get(HERE/'objects',event['selected_witness']);checks=[];expected=copy.deepcopy(old['controls']);events=copy.deepcopy(old['events'])
    # Regenerate source, variational matrices, rows and receiving/K from content inputs.
    for admission in event['admissions']:
        job=admission['job'];c=job['actor'];g0=get(HERE/'objects',job['generation_id']);g,gid=generate(s,c);assert g['passed'];assert g['anchor_id']==job['anchor_id'] and g['physical_rows_id']==g0['physical_rows_id']
        anchor=get(HERE/'objects',job['anchor_id']);cert=get(HERE/'objects',job['derivative_id']);verify(data_for(anchor),m,cert,anchor,True);rows=rows_for(data_for(anchor),m,anchor,cert,'B-JL');assert content_id(rows)==g0['physical_rows_id']
        raw=get(HERE/'objects',admission['raw_result_id']);assert raw['source']==normal(state_record(s)) and raw['anchor_id']==job['anchor_id'] and raw['derivative_id']==job['derivative_id']
        model=get(HERE/'objects',job['assembly_id']);inner,res=reserved_rows(rows);ref=np.asarray([float(F(x)) for u in old['controls'][c][k:k+32] for x in u]);rebuilt=assembly(inner,ref,F(1,10000))[-1]
        for key in ('P','q','A','b','cone_dimensions'):assert model[key]==rebuilt[key],'cone reconstruction '+key
        cfg=raw['settings'];assert cfg['tol_feas']==cfg['tol_gap_abs']==cfg['tol_gap_rel']==1e-10 and cfg['max_iter']==300 and cfg['time_limit']==5 and cfg['max_threads']==1
        p=admission['candidate'];d=[F(x)-F(y) for u,v in zip(p['controls'][:32],old['controls'][c][k:k+32]) for x,y in zip(u,v)]
        if admission['accepted']:
            assert raw['status'] in ('Solved','AlmostSolved');matches=[]
            for alpha in (F(1),F(1,2),F(1,4)):
                dq,qproof=quantize(raw['x'],alpha,anchor['contracts'])
                if qproof['passed'] and dq==d:matches.append(alpha)
            assert matches and evaluate(rows,d)['passed']
            assert H*sum(F(x)**2 for u in p['controls'][:32] for x in u)<=H*sum(F(x)**2 for u in old['controls'][c][k:k+32] for x in u)
        assert p['controls'][33:]==old['controls'][c][k+33:] and p['deadline']==max(old['goal_times'][c]-k,0)
        receiver=get(HERE/'objects',anchor['receiver']['certificate_id']);audit.inclusion(receiver);contract=get(HERE/'objects',s['contract_id']);ag=list(map(F,receiver['allocation'][2]['a']));r=receiver['domain'];radii=[F(r[x]) for x in ('position_l1_radius','yaw_radius','speed_radius','steering_radius')];r0=list(map(F,receiver['baseline']));assert sum(a*(x+y) for a,x,y in zip(ag,radii,r0))<=F(contract['cap'][c]),'suffix product-contract budget'
        checks.append(dict(actor=c,accepted=admission['accepted'],first_changed=p['controls'][0]!=old['controls'][c][k],raw_status=raw['status'],generation_recomputed=True,derivatives_recomputed=True,rows_recomputed=True))
    from protocol import local_source
    for token in event['tokens']:
        req=token['request'];assert token['commit_id']==content_id(req)
        for c,p in req['plans'].items():
            assert p['source']==local_source(s,c);expected[c][k:]=p['controls'];from receivers import replacement_events
            events=replacement_events(events,c,p['receiver'])
    assert new['controls']==expected and new['events']==events
    for name in ('owners','ticks','goal_times','starts','model_sha256','domain_certificate'):assert new[name]==old[name]
    s['contract_id']=None;return checks

def accept(arm,full=True):
    began=time.perf_counter();out=HERE/'results'/arm;cfg=read(out/'config.json');summary=read(out/'summary.json');audit=Audit(cfg['initial_witness']);audit.state['contract_id']=None;audit.validate_witness(cfg['initial_witness']);checked=[];events={e['cycle']:e for e in summary['events']};minimum_B=2802
    trace_path=out/'execution_source_reconstructed.jsonl.gz'
    if not trace_path.exists():trace_path=out/'execution.jsonl.gz'
    with gzip.open(trace_path,'rt') as stream:
        for row in map(json.loads,stream):
            s=audit.state;k=s['cycle'];assert row['cycle']==k
            if k in events:checked.append(dict(cycle=k,candidates=event_check(audit,events[k])))
            assert row['source']==normal(state_record(s)),'frame source mismatch';w=get(HERE/'objects',row['witness_id']);owner=w['owners'][k];domains=audit.domains
            if owner!=s['owner']:
                if s['owner']:audit.domain(s['owner'],'G_out')
                if owner:audit.domain(owner,'W_in');assert owner not in s['served'];s['served'].append(owner)
                s['permission_version']+=1
            s.update(owner=owner,witness=row['witness_id']);assert row['permission_version']==s['permission_version'] and row['served']==s['served'];before={c:max(w['goal_times'][c]-k,0) for c in CARS};after={c:max(v-1,0) for c,v in before.items()};assert row['b_before']==before and row['b_after']==after and row['B_before']==max(before.values()) and row['B_after']==max(after.values())
            for c,kind in w['events'].get(str(k),[]):
                if kind.startswith('generated:'):
                    audit.domain(c,kind);r=get(HERE/'objects',kind.split(':')[1]);from receivers import feedback
                    assert tuple(map(F,w['controls'][c][k]))==feedback(s,c,r)
            commands={c:tuple(map(F,w['controls'][c][k])) for c in CARS};audit.step(commands,owner,row['actual'])
            for c,kind in w['events'].get(str(k+1),[]):audit.domain(c,kind)
            for c in CARS:
                if k+1>=w['goal_times'][c]:audit.domain(c,'target');assert s['v'][c]==s['delta'][c]==0
            assert row['B_after']==max(2802-k-1,0)
    assert normal(state_record(audit.state))==summary['final'];assert audit.state['cycle']==2802
    for c in CARS:audit.domain(c,'target')
    result=dict(passed=True,arm=arm,steps=2802,all_goals=True,B_final=0,checks=checked,minimum_road_m=float(audit.road),minimum_pair_m=float(audit.pair),wall_s=time.perf_counter()-began,shared=['integer interval/trigonometry and signed ZOH','variational builder and SOCP compiler','configuration and hash primitives'],independent=['8-subsegment bodies/road/six pairs and resource clearance','source/history and raw-to-quantized local commands','cone matrices, row reconstruction, receiving inclusion, actual K and all later domains','unique absolute ticks and per-car deadlines'])
    write(out/'independent_acceptance.json',result);return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--arm',required=True);r=accept(p.parse_args().arm);print(json.dumps(r,indent=2))
