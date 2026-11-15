"""Bounded three-arm experiment; ledger registration precedes every solve release."""
import argparse,multiprocessing as mp,time
from common import *
from local_update import prepare_job,native,admit
from protocol import prepare,commit,select_for_execution,groups
from contracts import build,bind
from execution import advance
from strategy import certify_combination

def calls(jobs,arm,roundno,output):
    ctx=mp.get_context('spawn');workers=[];startup=time.time_ns()
    for job in jobs:
        parent,child=ctx.Pipe();p=ctx.Process(target=native,args=(child,));p.start();child.close();workers.append((p,parent,job))
    ready=[]
    try:
        for p,pipe,job in workers:
            if not pipe.poll(60):raise TimeoutError('worker boot timeout, zero solver release')
            ready.append(pipe.recv());pipe.send(job)
        for p,pipe,job in workers:
            if not pipe.poll(60):raise TimeoutError('worker assembly timeout, zero solver release')
            assert pipe.recv()['assembled']
        online=time.time_ns();ledger=read(HERE/'solver_calls.json');assert len(ledger)+len(jobs)<=12
        ids=[]
        for job in jobs:
            key=f'{arm}:r{roundno}:{job["actor"]}';assert key not in [c['event'] for c in ledger]
            ids.append(len(ledger));ledger.append(dict(event=key,arm=arm,round=roundno,actor=job['actor'],generation_id=job['generation_id'],anchor_id=job['anchor_id'],registered_ns=time.time_ns(),source_version=content_id(read(HERE/'executed_source_versions.json')),completed=False))
        write(HERE/'solver_calls.json',ledger);results=[]
        if arm=='F-S':
            for p,pipe,job in workers:
                release=time.time_ns();pipe.send(release)
                if pipe.poll(15):results.append(pipe.recv())
                else:results.append(dict(status='Timeout',release_ns=release));p.terminate()
        else:
            release=time.time_ns()
            for p,pipe,job in workers:pipe.send(release)
            for p,pipe,job in workers:
                if pipe.poll(15):results.append(pipe.recv())
                else:results.append(dict(status='Timeout',release_ns=release));p.terminate()
        for idx,raw,job in zip(ids,results,jobs):
            raw.update(event=ledger[idx]['event'],arm=arm,round=roundno,actor=job['actor'],source=job['source'],generation_id=job['generation_id'],anchor_id=job['anchor_id'],derivative_id=job['derivative_id'],assembly_id=job['assembly_id']);rid=put(HERE/'objects',raw);ledger[idx].update(completed=True,status=raw['status'],raw_result_id=rid)
        write(HERE/'solver_calls.json',ledger)
        overlap=max(0,min(r.get('ended_ns',0) for r in results)-max(r.get('started_ns',0) for r in results))
        timing=dict(worker_startup_s=(online-startup)/1e9,common_release_ns=online,results_received_ns=time.time_ns(),solve_intervals=[{k:r.get(k) for k in ('pid','started_ns','ended_ns','cpu_s','release_ns')} for r in results],overlap_ns=overlap,ready=ready)
        return results,timing
    finally:
        for p,pipe,job in workers:
            p.join(1)
            if p.is_alive():p.terminate();p.join()
            pipe.close()

def run(arm):
    assert read(HERE/'results/protocol_checks.json')['passed'];assert read(HERE/'results/generation_regression.json')['passed'];assert read(HERE/'results/terminal_hold.json')['passed'];reg=read(HERE/'pre_registration.json');out=HERE/'results'/arm;assert not out.exists();out.mkdir();s=initial_state();s['witness']=reg['initial_witness'];versions={c:0 for c in CARS};events=[];began=time.perf_counter();initial=s['witness'];write(out/'config.json',dict(arm=arm,initial_witness=initial,registration_id=content_id(reg),versions=versions))
    with gzip.open(out/'execution.jsonl.gz','wt') as trace:
        while s['cycle']<get(HERE/'objects',s['witness'])['ticks']:
            k=s['cycle'];event=None
            if k in (reg['cycle'],reg['next_cycle']):
                rn=1+int(k==reg['next_cycle']);cycle_start=time.time_ns()
                if rn==1:bind(s,reg['contract_id']);assert normal(state_record(s))==reg['state']
                else:s['contract_id']=None;bind(s,build(s))
                contract=get(HERE/'objects',s['contract_id']);actors=reg['actors'];jobs=[]
                for c in actors:
                    gid=next(x['generation_id'] for x in reg['scan'][-1]['checks'] if x['actor']==c and 'freedom' in x) if rn==1 else None
                    jobs.append(prepare_job(s,c,gid))
                # Both priors passed before this arm/round is allowed to use its allocation.
                from generation import generate
                extra_jobs={}
                if arm!='D-P':
                    for c in CARS:
                        if c not in actors:
                            g,gid=generate(s,c);assert g['passed'],g;extra_jobs[c]=dict(generation_id=gid)
                results,timing=calls(jobs,arm,rn,out);candidates={};admissions=[]
                for job,raw in zip(jobs,results):
                    p,a=admit(s,job,raw);candidates[p['actor']]=p;admissions.append(a)
                partition=[list(CARS)] if arm!='D-P' else groups([]);tokens=[];txn_start=time.time_ns()
                for people in partition:
                    relevant=[c for c in people if c in candidates]
                    if not relevant:continue
                    # Fixed groups carry all four participants, including unchanged anchors.
                    if arm!='D-P':
                        from check_protocol import anchor_candidate
                        for c in people:
                            if c not in candidates:
                                job=extra_jobs[c];candidates[c]=anchor_candidate(s,c,job['generation_id'])
                    req=prepare(s,candidates,people,s['contract_id'],versions);tokens.append(commit(s,req,versions))
                wid,choice=select_for_execution(s,tokens,versions);selection_ns=time.time_ns()
                proof=certify_combination(s,wid)
                event=normal(dict(arm=arm,round=rn,cycle=k,source=state_record(s),jobs=jobs,admissions=admissions,tokens=tokens,selected_witness=wid,choice=choice,combination_proof_id=put(HERE/'objects',proof),partition=partition,timing=dict(**timing,cycle_start_ns=cycle_start,selection_ns=selection_ns,cycle_wall_s=(selection_ns-timing['common_release_ns'])/1e9,source_to_selection_s=(selection_ns-cycle_start)/1e9,group_commit_s=(selection_ns-txn_start)/1e9,full_audit_s=proof['wall_s']),prior_receiver_replacement=[dict(actor=c,old=[x for x in get(HERE/'objects',s['witness'])['events'].items() if any(e[0]==c and e[1].startswith('generated:') for e in x[1])],new=candidates[c]['receiver']) for c in actors]))
                events.append(event);write(out/f'round_{rn}.json',event)
                for c in candidates:versions[c]+=1
                s['contract_id']=None
            else:wid=s['witness']
            row=advance(s,wid);row['event_round']=event['round'] if event else None;trace.write(json.dumps(row,separators=(',',':'))+'\n')
    from certify_domains import check_domain
    for c in CARS:check_domain(s,c,'target',read(HERE/'domain_checks.json')['domains'])
    value=dict(passed=True,arm=arm,steps=s['cycle'],logical_time_s=str(H*s['cycle']),events=events,final=state_record(s),wall_s=time.perf_counter()-began,new_calls=4,all_goals=True)
    write(out/'summary.json',value);print(dict(arm=arm,passed=True,steps=s['cycle'],accepted=[sum(a['accepted'] for a in e['admissions']) for e in events]),flush=True)
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--arm',choices=('F-S','F-P','D-P'),required=True);run(parser.parse_args().arm)
