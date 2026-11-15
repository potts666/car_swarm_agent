"""Zero-call finite pool scan, then full physical/receiving certification."""
import time,collections
from common import *
from execution import advance
from proposals import pool_for,candidate,launch_controls
from progress import ETA,distance
from contracts import build,bind,physical_diff
from generation import generate

def scan(repair):
    oldreg=read(HERE.parent/'dynamic_groups_001/pre_registration.json');initial=oldreg['initial_witness'];old=get(HERE/'objects',initial);s=initial_state();s['witness']=initial;states=[];vel=[]
    for k in range(old['ticks']):states.append(state_record(s));vel.append(s['v'].copy());advance(s,initial)
    states.append(state_record(s));counts=collections.Counter();summaries=[];winners={};oldfour=0
    for k,record in enumerate(states[:-33]):
        s=restore(m,record);pool=pool_for(old,s);scores=[]
        base_active=[c for c in CARS if distance(s['v'][c],F(old['controls'][c][k][0]))>=ETA];oldfour+=int(len(base_active)==4)
        for shifts in pool:
            active=[];progress={};failure=None
            for c in CARS:
                shift=shifts.get(c,0);a=F(old['controls'][c][k][0]);v=s['v'][c]
                if shift:
                    route=get(HERE/'objects',read(HERE/'rational_inputs.json')['routes'][c]);controls,pr=launch_controls(route,repair);r=k-(old['starts'][c]-shift);first=next(j for j,u in enumerate(controls) if any(F(x) for x in u))
                    if r>first:failure='rewrites_movement';break
                    if v!=0 or s['delta'][c]!=0:failure='not_stationary';break
                    a=F(controls[r][0]) if 0<=r<len(controls) else F(0);v=F(0)
                d=distance(v,a);progress[c]=d
                if d>=ETA:active.append(c)
            if failure:counts[failure]+=1;scores.append(dict(shifts=shifts,failure=failure));continue
            added=[c for c in active if c not in base_active and s['v'][c]==0];A=len(active);P=sum(min(progress[c]/ETA,F(1)) for c in CARS);score=(-A,-P,max(old['goal_times'][c]-shifts.get(c,0)-k for c in CARS),H*sum(F(x)**2 for c in CARS for u in old['controls'][c][k:k+32] for x in u),content_id(shifts))
            scores.append(normal(dict(shifts=shifts,A=A,P=P,active=active,added=added,progress=progress,score=score)))
            counts['pool_candidate']+=1
            if added and A>=2 and A not in winners:winners[A]=dict(cycle=k,state=record,shifts=shifts,active=active,added=added,progress=normal(progress),pool=scores)
        valid=[x for x in scores if 'score' in x]
        # Recompute M3 using rational values; record the finite pool maximum per source.
        if valid:
            best=min(valid,key=lambda x:tuple(F(y) for y in x['score'][:4])+(x['score'][4],));summaries.append(dict(cycle=k,pool_size=len(pool),old_active=base_active,best=best,failures=collections.Counter(x.get('failure','valid') for x in scores)))
            if best['added'] and best['A']>=2:
                A=best['A'];saved=winners.get(A)
                if saved and saved['cycle']==k:winners[A]=dict(cycle=k,state=record,shifts=best['shifts'],active=best['active'],added=best['added'],progress=best['progress'],pool=scores)
    return dict(initial_witness=initial,winners=winners,counts=dict(counts),sources_scanned=len(summaries),initial_four_motion_regression_ticks=oldfour,scan_id=put(HERE/'objects',summaries))

def main():
    assert read(HERE/'solver_calls.json')==[]
    initial=scan(False);write(HERE/'results/scan_initial.json',initial)
    result=scan(True);write(HERE/'results/scan_repaired.json',result);assert result['winners'],'no enlarged activity in finite pool'
    winner=result['winners'][max(result['winners'])];s=restore(m,winner['state']);old=get(HERE/'objects',s['witness']);proposal=candidate(old,s,winner['shifts']);wid=put(HERE/'objects',proposal);oldcid=build(s);newstate=copy.deepcopy(s);newstate['witness']=wid;obligations={str(winner['cycle']+j):{c:str(ETA) for c in winner['active']} for j in (0,1)};cid=build(newstate,obligations);bind(newstate,cid);diff=physical_diff(oldcid,cid);assert diff['physical_changed'] and len(diff['closure'])>=2;write(HERE/'physical_contract_diff.json',diff);checks=[]
    for c in CARS:
        g,gid=generate(newstate,c);checks.append(dict(actor=c,generation_id=gid,passed=g['passed'],failure=g.get('failure')));write(HERE/'results/zero_generation.progress.json',checks);assert g['passed'],checks[-1]
    successor=copy.deepcopy(newstate);advance(successor,wid);successor['contract_id']=None;cid2=build(successor,obligations,base=cid);bind(successor,cid2);checks2=[]
    for c in winner['active']:
        g,gid=generate(successor,c);checks2.append(dict(actor=c,generation_id=gid,passed=g['passed'],failure=g.get('failure')));assert g['passed'],checks2[-1]
    value=dict(passed=True,initial_witness=result['initial_witness'],cycle=winner['cycle'],next_cycle=winner['cycle']+1,actual_source=winner['state'],anchor_source=state_record(newstate),candidate_witness=wid,old_contract=oldcid,new_contract=cid,active=winner['active'],added=winner['added'],shifts=winner['shifts'],obligations=obligations,zero_checks=checks,second_baseline_checks=checks2,second_baseline_source=state_record(successor),scan=result,first_pool=winner['pool'],calls_at_freeze=0,priority='first certified A=4 else A=3 else enlarged A=2; complete bounded pool scan before numerical output')
    write(HERE/'pre_registration.json',value);write(HERE/'proposal_index.json',dict(initial_scan=initial,repaired_scan=result,pool=winner['pool'],selected=winner['shifts']));print(dict(passed=True,cycle=winner['cycle'],active=winner['active'],added=winner['added'],calls=0),flush=True)
if __name__=='__main__':main()
