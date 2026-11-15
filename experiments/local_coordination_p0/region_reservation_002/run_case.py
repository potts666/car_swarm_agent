"""Two full controlled traces; real authorization, simultaneous ZOH and local update."""
import argparse,time
from common import *
from witness import make,propose_choice,plan_id
from protocol import prepare,commit,select_for_execution
from physics import step
from certify_domains import check_domain
from local_update import solve


def advance(state,wid):
    parent=plan_id(state,state['witness']);request=prepare(state,wid,parent);token=commit(state,request);commands=select_for_execution(state,token);actual=step(state,commands,state['owner'],4)
    checks=[];w=get(HERE/'objects',wid)
    for c,kind in w['events'].get(str(state['cycle']),[]):checks.append(check_domain(state,c,kind,read(HERE/'domain_checks.json')['domains']))
    return dict(cycle=state['cycle']-1,parent_plan_id=parent,witness_id=wid,token=token,actual=actual,budget_before=2803-state['cycle'],budget_after=2802-state['cycle'],shifted_plan_id=plan_id(state,wid),domain_checks=checks)


def damage_fork(state,proposed,output):
    trial=copy.deepcopy(state);old=trial['witness'];request=prepare(trial,proposed,plan_id(trial,old));bad=copy.deepcopy(request);bad['permission_version']+=9
    try:commit(trial,bad)
    except ValueError as error:reason=str(error)
    else:raise AssertionError('damaged version committed')
    assert state_record(trial)==state_record(state);output.mkdir();records=[]
    with gzip.open(output/'execution.jsonl.gz','wt') as stream:
        for j in range(32):
            row=advance(trial,old);records.append(row);stream.write(json.dumps(row,default=str,separators=(',',':'))+'\n')
    value=dict(passed=True,kind='damaged_permission_version',source=state_record(state),old_witness=old,proposed_witness=proposed,damaged_request=bad,rejection=reason,invalid_commits=0,actual_steps=32,first_actual=records[0]['actual']['commands'],final=state_record(trial),remaining_budget=2802-trial['cycle'])
    write(output/'summary.json',value);return value


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--case',choices=('opposite','same_direction'),required=True);args=parser.parse_args();output=HERE/'results'/args.case;assert not output.exists();output.mkdir();began=time.perf_counter()
    state=initial_state();wid=put(HERE/'objects',make(list(CARS)));state['witness']=wid;write(output/'config.json',dict(case=args.case,initial_witness=wid,logical_h=str(H),source_commit=read(HERE/'plan.json')['source_commit'],formal_batch=False));events=[];overlap=[];minimum_road=minimum_pair=F(100);failure=None;selection=[]
    with gzip.open(output/'execution.jsonl.gz','wt') as stream:
        try:
            for cycle in range(2802):
                assert state['cycle']==cycle;event=None;w=get(HERE/'objects',state['witness']);wid=state['witness']
                if cycle>=55 and (cycle-55)%663==0 and (cycle-55)//663<4:
                    new,choice=propose_choice(state,w,args.case);wid=put(HERE/'objects',new);event=dict(kind='resource_choice',cycle=cycle,old_witness=state['witness'],new_witness=wid,choice=choice)
                    if cycle==718:damage_fork(state,wid,output/'version_rejection')
                    selection.append(choice)
                if cycle in (787,788):
                    actor=w['owners'][cycle];wid,event=solve(state,actor,args.case,output/('local_socp_round_'+str(cycle-786)))
                row=advance(state,wid);row['event']=event
                if event:event['commit_id']=row['token']['commit_id'];events.append(event)
                if cycle>=55 and len(row['actual']['bounds']['active'])>=2:overlap.append(dict(cycle=cycle,cars=row['actual']['bounds']['active']))
                minimum_road=min(minimum_road,F(row['actual']['bounds']['road']));minimum_pair=min(minimum_pair,F(row['actual']['bounds']['pair']));stream.write(json.dumps(row,default=str,separators=(',',':'))+'\n')
            for c in CARS:check_domain(state,c,'target',read(HERE/'domain_checks.json')['domains'])
            assert overlap and state['cycle']==2802
        except Exception as error:failure=repr(error)
    summary=dict(passed=failure is None,error=failure,case=args.case,actual_steps=state['cycle'],actual_task_time_s=str(H*state['cycle']),remaining_budget=2802-state['cycle'],wall_s=time.perf_counter()-began,minimum_road_m=str(minimum_road),minimum_pair_m=str(minimum_pair),overlap_ticks=len(overlap),overlap=overlap,selection=selection,events=events,final=state_record(state),local_socp_accepted=any(e['kind']=='local_socp' and e['accepted'] for e in events),numerical_proposals=sum(e['kind']=='local_socp' for e in events),accepted_numerical_proposals=sum(e['kind']=='local_socp' and e['accepted'] for e in events),formal_batch=False)
    write(output/'events.json',events);write(output/'summary.json',summary);print(json.dumps({k:v for k,v in summary.items() if k not in ('overlap','events','final')},indent=2));return 0 if failure is None else 1
if __name__=='__main__':raise SystemExit(main())
