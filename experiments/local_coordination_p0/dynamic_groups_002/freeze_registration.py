"""Content-reuse prior zero-call certificates, correct full-tail score, bind atomic migration."""
from common import *
from proposals import pool_for,launch_controls
from progress import ETA,distance
from contracts import build,bind,physical_diff
from generation import generate
from execution import advance
from protocol import reference_state

def main():
    assert read(HERE/'solver_calls.json')==[];reg=read(HERE/'pre_registration.json');
    if not (HERE/'results/pre_registration.pre_transition.json').exists():write(HERE/'results/pre_registration.pre_transition.json',reg)
    old=get(HERE/'objects',reg['initial_witness']);energy=[];E=F(0)
    for k in range(old['ticks']-1,-1,-1):E+=H*sum(F(x)**2 for c in CARS for x in old['controls'][c][k]);energy.append(E)
    energy=list(reversed(energy));v={c:F(0) for c in CARS};d=v.copy();rows=[];winners={};reasons={};gains={}
    for c in CARS:
        route=get(HERE/'objects',read(HERE/'rational_inputs.json')['routes'][c]);u,proof=launch_controls(route,True);gains[c]=H*sum(F(x)**2-F(y)**2 for a,b in zip(u,route['controls']) for x,y in zip(a,b))
    for k in range(old['ticks']-33):
        state=dict(cycle=k,v=v.copy(),delta=d.copy());scores=[];baseline=[c for c in CARS if distance(v[c],F(old['controls'][c][k][0]))>=ETA]
        for shifts in pool_for(old,state):
            progress={};failure=None
            for c in CARS:
                vel=v[c];a=F(old['controls'][c][k][0]);shift=shifts.get(c,0)
                if shift:
                    route=get(HERE/'objects',read(HERE/'rational_inputs.json')['routes'][c]);u,_=launch_controls(route,True);r=k-old['starts'][c]+shift;first=next(j for j,z in enumerate(u) if any(F(x) for x in z))
                    if r>first:failure='rewrites_executed_motion';break
                    vel=F(0);a=F(u[r][0]) if 0<=r<len(u) else F(0)
                progress[c]=distance(vel,a)
            if failure:reasons[failure]=reasons.get(failure,0)+1;continue
            active=[c for c in CARS if progress[c]>=ETA];added=[c for c in active if c not in baseline and v[c]==0];cost=energy[k]+sum(gains[c] for c in shifts);score=(-len(active),-sum(min(progress[c]/ETA,F(1)) for c in CARS),max(old['goal_times'][c]-shifts.get(c,0)-k for c in CARS),cost,content_id(shifts));scores.append(dict(shifts=shifts,score=score,active=active,added=added,progress=progress))
        best=min(scores,key=lambda p:p['score']);rows.append(normal(dict(cycle=k,pool_size=len(pool_for(old,state)),old_active=baseline,best=best)))
        if best['added'] and len(best['active'])>=2:winners.setdefault(len(best['active']),dict(cycle=k,**best))
        for c in CARS:v[c]+=H*F(old['controls'][c][k][0]);d[c]+=H*F(old['controls'][c][k][1])
    chosen=winners[max(winners)];assert chosen['cycle']==reg['cycle'] and chosen['shifts']==reg['shifts'],'M3 full-tail scoring changed choice'
    write(HERE/'proposal_index.json',dict(rule=dict(deltas=[1,2,4,8],width=4,depth=2,max_pool=16,J='exact full saved control-input cost including analytic launch repair'),scan_id=put(HERE/'objects',rows),sources_scanned=len(rows),winners=normal(winners),rejection_classes=reasons,old_score_replaced='early diagnostic scan used 32-step old-plan score; corrected to actual full-tail cost before any solve',initial_rejection_scan=read(HERE/'results/scan_initial.json')))
    actual=restore(m,reg['actual_source']);c=copy.deepcopy(get(HERE/'objects',reg['new_contract']));diff=physical_diff(reg['old_contract'],reg['new_contract']);c['transition']=dict(actual_source=state_record(actual),old_contract=reg['old_contract'],closure=diff['closure'],old_faces_id=get(HERE/'objects',reg['old_contract'])['faces_id'],old_calendar_id=get(HERE/'objects',reg['old_contract'])['owner_calendar_id']);cid=put(HERE/'objects',c);ref=reference_state(actual,cid);checks=[]
    for car in CARS:
        g,gid=generate(ref,car);checks.append(dict(actor=car,generation_id=gid,passed=g['passed'],failure=g.get('failure')));write(HERE/'results/zero_transition.progress.json',checks);assert g['passed'],checks[-1]
    successor=copy.deepcopy(actual);advance(successor,reg['candidate_witness']);successor['contract_id']=None;cid2=build(successor,reg['obligations'],base=cid);ref2=reference_state(successor,cid2);second=[]
    for car in reg['active']:
        g,gid=generate(ref2,car);second.append(dict(actor=car,generation_id=gid,passed=g['passed'],failure=g.get('failure')));assert g['passed'],second[-1]
    reg.update(new_contract=cid,anchor_source=state_record(ref),zero_checks=checks,second_baseline_checks=second,second_baseline_source=state_record(ref2),proposal_index_id=content_id(read(HERE/'proposal_index.json')),scoring='M3 full-tail exact actual input cost; complete pool per source; all certified before calls')
    write(HERE/'physical_contract_diff.json',physical_diff(reg['old_contract'],cid));write(HERE/'pre_registration.json',reg);print(dict(passed=True,cycle=reg['cycle'],active=reg['active'],new_calls=0,full_tail_M3=True),flush=True)
if __name__=='__main__':main()
