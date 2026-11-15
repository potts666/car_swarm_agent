"""R-old replay and two numerical arms, with a single durable native-call ledger."""
import argparse,multiprocessing as mp,time
from common import *
from worker_io import Channel
from local_update import native,prepare_job,admit
from protocol import reference_state,anchor_candidate,prepare,commit,select_for_execution,groups
from contracts import build
from generation import generate
from execution import advance
from strategy import certify_combination

def numerical_calls(jobs,arm,rn):
    ctx=mp.get_context('spawn');workers=[];boot=time.time_ns();ready=[]
    try:
        for job in jobs:
            parent,child=ctx.Pipe();p=ctx.Process(target=native,args=(child,));p.start();child.close();workers.append((p,Channel(parent),job))
        for p,ch,job in workers:
            if not ch.poll(60):raise TimeoutError('worker boot: no solve released')
            ready.append(ch.recv());ch.send(job)
        for p,ch,job in workers:
            if not ch.poll(60):raise TimeoutError('worker assembly: no solve released')
            assert ch.recv()['assembled']
        ledger=read(HERE/'solver_calls.json');assert len(ledger)+len(jobs)<=16;indices=[]
        for job in jobs:
            event=f'{arm}:r{rn}:{job["actor"]}';assert event not in [x['event'] for x in ledger],'already registered native call'
            indices.append(len(ledger));ledger.append(dict(event=event,arm=arm,round=rn,actor=job['actor'],generation_id=job['generation_id'],registered_ns=time.time_ns(),completed=False,source_version=content_id(read(HERE/'executed_source_versions.json'))))
        write(HERE/'solver_calls.json',ledger);release=time.time_ns()
        for p,ch,job in workers:ch.send(release)
        results=[]
        for p,ch,job in workers:
            if ch.poll(15):results.append(ch.recv())
            else:p.terminate();results.append(dict(status='Timeout',release_ns=release))
        for idx,raw,job in zip(indices,results,jobs):
            raw.update(event=ledger[idx]['event'],arm=arm,round=rn,actor=job['actor'],source=job['source'],generation_id=job['generation_id'],anchor_id=job['anchor_id'],derivative_id=job['derivative_id'],assembly_id=job['assembly_id']);rid=put(HERE/'objects',raw);ledger[idx].update(completed=True,status=raw['status'],raw_result_id=rid)
        write(HERE/'solver_calls.json',ledger)
        overlap=max(0,min(r.get('ended_ns',0) for r in results)-max(r.get('started_ns',0) for r in results));pipe=[dict(actor=j['actor'],parent_sent_bytes=ch.sent,parent_received_bytes=ch.received) for p,ch,j in workers]
        return results,dict(release_ns=release,received_ns=time.time_ns(),startup_s=(release-boot)/1e9,overlap_ns=overlap,pipe=pipe,intervals=[{k:r.get(k) for k in ('pid','started_ns','ended_ns','cpu_s')} for r in results],ready=ready)
    finally:
        for p,ch,job in workers:
            p.join(1)
            if p.is_alive():p.terminate();p.join()
            ch.close()

def run(arm):
    reg=read(HERE/'pre_registration.json');assert reg['passed'];out=HERE/'results'/arm;assert not out.exists();out.mkdir();s=initial_state();s['witness']=reg['initial_witness'];versions={c:0 for c in CARS};events=[];began=time.perf_counter();physical_base=None
    if arm!='R-old':
        assert read(HERE/'results/protocol_checks.json')['passed'] and read(HERE/'results/generation_regression.json')['passed'];assert read(HERE/'results/pool_certification.json')['passed'] and read(HERE/'results/atlas_certification.json')['passed']
        if arm=='L-new':assert read(HERE/'results/repair_checks.json')['passed']
    write(out/'config.json',dict(arm=arm,initial_witness=s['witness'],registration_id=content_id(reg),new_solver_limit=16,logical_h=str(H)))
    with gzip.open(out/'execution.jsonl.gz','wt') as stream:
        while s['cycle']<get(HERE/'objects',s['witness'])['ticks']:
            k=s['cycle'];event=None;wid=s['witness']
            if arm!='R-old' and k in (reg['cycle'],reg['next_cycle']):
                rn=1+int(k==reg['next_cycle']);cycle_start=time.time_ns();old_groups=groups([]) if rn==1 else [list(CARS)]
                if rn==1:
                    assert normal(state_record(s))==reg['actual_source'];cid=reg['new_contract'];physical_base=cid
                else:cid=build(s,reg['obligations'],base=physical_base)
                reference=reference_state(s,cid);active=reg['active'];jobs=[];gids={}
                for c in CARS:
                    if rn==1:gid=next(x['generation_id'] for x in reg['zero_checks'] if x['actor']==c);g=get(HERE/'objects',gid)
                    else:g,gid=generate(reference,c)
                    assert g['passed'],g;gids[c]=gid
                    if c in active:jobs.append(prepare_job(reference,c,gid))
                # Both rounds' activity and all zero anchors are prior gates to their allocations.
                raws,nt=numerical_calls(jobs,arm,rn);plans={};admissions=[]
                for job,raw in zip(jobs,raws):
                    p,a=admit(reference,job,raw);plans[p['actor']]=p;admissions.append(a)
                for c in CARS:
                    if c not in plans:plans[c]=anchor_candidate(reference,c,gids[c])
                partition=[list(CARS)] if arm=='C-new' or rn==1 else groups([]);tokens=[];prepare_times=[]
                for people in partition:
                    if arm=='L-new' and rn==2 and not set(people).intersection(active):continue
                    start=time.perf_counter();request=prepare(s,plans,people,cid,versions);prepare_times.append(time.perf_counter()-start);tokens.append(commit(s,request,versions))
                wid,choice,selection=select_for_execution(s,tokens,versions,cid);chosen_ns=time.time_ns();proof=certify_combination(s,wid)
                event=normal(dict(arm=arm,round=rn,cycle=k,actual_source=state_record(s),anchor_source=state_record(reference),contract_id=cid,old_contract_id=reg['old_contract'] if rn==1 else physical_base,jobs=jobs,admissions=admissions,plans=plans,tokens=tokens,choice=choice,selected_witness=wid,full_proof_id=put(HERE/'objects',proof),groups_before=old_groups,groups_after=partition,physical_change=rn==1,group_event='merge_for_physical_boundary_and_calendar' if rn==1 else 'split_after_interfaces_frozen' if arm=='L-new' else 'fixed_all_four',timing=dict(**nt,cycle_start_ns=cycle_start,selection_ns=chosen_ns,source_to_selection_s=(chosen_ns-cycle_start)/1e9,release_to_selection_s=(chosen_ns-nt['release_ns'])/1e9,prepare_s=prepare_times,dependency_check_s=[t['timing']['dependency_check_s'] for t in tokens],commit_s=[t['timing']['commit_s'] for t in tokens],selection_s=selection['selection_s'],full_combination_audit_s=proof['wall_s'])))
                events.append(event);write(out/f'round_{rn}.json',event)
                for people in partition:
                    for c in people:versions[c]+=1
            row=advance(s,wid);row['event_round']=event['round'] if event else None;row['O']=reg['active'] if event else [];stream.write(json.dumps(row,separators=(',',':'))+'\n')
    from certify_domains import check_domain
    for c in CARS:check_domain(s,c,'target',read(HERE/'domain_checks.json')['domains'])
    summary=normal(dict(passed=True,arm=arm,steps=s['cycle'],logical_time_s=str(H*s['cycle']),events=events,final=state_record(s),wall_s=time.perf_counter()-began,new_calls=len(reg['active'])*len(events),all_goals=True))
    write(out/'summary.json',summary);print(dict(passed=True,arm=arm,steps=s['cycle'],accepted=[sum(a['accepted'] for a in e['admissions']) for e in events]),flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--arm',choices=('R-old','C-new','L-new'),required=True);run(p.parse_args().arm)
