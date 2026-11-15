"""Materialize and certify every retained candidate at the preregistered source."""
import sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from common import *
from proposals import candidate,pool_for,launch_controls
from progress import ETA,step_progress
from physics import predicted_nodes
from contracts import build,bind
from generation import generate

def main():
    assert read(HERE/'solver_calls.json')==[];reg=read(HERE/'pre_registration.json');source=restore(m,reg['actual_source']);old=get(HERE/'objects',source['witness']);rows=[];previous=read(HERE/'results/pool_certification.progress.json') if (HERE/'results/pool_certification.progress.json').exists() else []
    for i,shifts in enumerate(pool_for(old,source)):
        began=time.perf_counter();entry=dict(index=i,shifts=shifts,passed=False)
        prior=next((x for x in previous if x['index']==i and x['shifts']==shifts),None)
        if prior:
            if prior.get('passed'):
                get(HERE/'objects',prior['witness_id']);get(HERE/'objects',prior['contract_id'])
                for x in prior['zero_checks']:get(HERE/'objects',x['generation_id'])
            prior['reused_prior_completed_certificate']=True;rows.append(prior);write(HERE/'results/pool_certification.progress.json',rows);print(dict(index=i,reused=True,passed=prior['passed']),flush=True);continue
        try:
            plan=candidate(old,source,shifts) if shifts else old;wid=put(HERE/'objects',plan);s=copy.deepcopy(source);s['witness']=wid;u={c:tuple(map(F,plan['controls'][c][s['cycle']])) for c in CARS};stats=step_progress(s,u,predicted_nodes(s,u,4),plan);active=[c for c in CARS if stats[c]['active']];obligations={str(s['cycle']+j):{c:str(ETA) for c in active} for j in (0,1)}
            if shifts==reg['shifts']:
                cid=reg['new_contract'];checks=reg['zero_checks'];assert all(x['passed'] for x in checks);entry['reused_selected_prior']=True
            else:
                cid=build(s,obligations);bind(s,cid);checks=[]
                for c in CARS:
                    g,gid=generate(s,c);checks.append(dict(actor=c,generation_id=gid,passed=g['passed'],failure=g.get('failure')));assert g['passed'],checks[-1]
            assert max(plan['goal_times'].values())<=max(old['goal_times'].values());J=H*sum(F(x)**2 for c in CARS for u0 in plan['controls'][c][source['cycle']:] for x in u0);P=sum(min(F(stats[c]['progress_lower'])/ETA,F(1)) for c in CARS);score=(-len(active),-P,max(plan['goal_times'].values())-source['cycle'],J,content_id(shifts));entry.update(passed=True,witness_id=wid,contract_id=cid,active=active,progress=stats,zero_checks=checks,score=normal(score))
        except (AssertionError,ValueError,KeyError) as e:entry.update(failure=repr(e),rejection_layer='physical/resource/receiver/zero-anchor prerequisite')
        entry['wall_s']=time.perf_counter()-began;rows.append(entry);write(HERE/'results/pool_certification.progress.json',rows);print(dict(index=i,shifts=shifts,passed=entry['passed'],failure=entry.get('failure')),flush=True)
    valid=[x for x in rows if x['passed']];selected=min(valid,key=lambda x:tuple(F(y) for y in x['score'][:4])+(x['score'][4],));assert selected['shifts']==reg['shifts'] and len(selected['active'])==3
    write(HERE/'results/pool_certification.json',dict(passed=True,new_calls=0,pool_size=len(rows),valid_count=len(valid),candidates=rows,selected_index=selected['index'],M3_verified_after_full_certification=True));print(dict(passed=True,selected=selected['shifts'],calls=0),flush=True)
if __name__=='__main__':main()
