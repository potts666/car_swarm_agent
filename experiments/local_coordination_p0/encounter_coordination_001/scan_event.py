"""Zero-solve ascending state scan and full finite encounter-pool certification."""
import itertools
from common import *
from initialize import run as initialize
from preflight import simultaneous
from plan_io import load,save,record,source
from encounter import active_pairs,propose,deadline_check,score
from local_mpc import prepare
from audit import audit

def run():
 initialize();old=simultaneous(load('results/init_0.json.gz'),True);save('results/R_old.json.gz',old)
 prior=HERE.parent/'autonomy_init_002';previous=__import__('json').loads(__import__('gzip').decompress((prior/'results/parallel_anchor.json.gz').read_bytes()));assert content_id(old)==content_id(previous)
 scan=[];chosen=None
 for k in range(old['B0']):
  state=source(old,k);events,active=active_pairs(old,state);scan.append({'cycle':k,'source_id':content_id(record(state)),'actual_v':state['v'],'active':active,'event_pairs':[e['actors'] for e in events],'reason':None if events else 'no two nonzero-speed opposite active requests in available straight mode'})
  if not events:continue
  for event in events:
   a,b=event['actors'];save('results/independent_intents.json.gz',event);proposals=[('R_old',old)]
   for passing,yielding in ((a,b),(b,a)):
    proposals.append((passing+'_passes_'+yielding+'_existing_stop',propose(old,state,passing,yielding)))
    proposals.append((passing+'_passes_'+yielding+'_analytic_brake',propose(old,state,passing,yielding,True)))
   table=[]
   for name,plan in proposals:
    save(f'results/proposals/{name}.json.gz',plan);limits=deadline_check(old,plan,k);entry={'id':name,'plan_id':content_id(plan),'deadlines':limits,'score':score(plan,k),'physical_passed':False,'numerical_window':False,'existing_stop_compatible_parallel':'existing_stop' in name}
    try:
     proof=audit(plan,output=HERE/f'results/proposals/{name}_acceptance.json');entry['physical_passed']=proof['passed'];entry['complete_suffix']=True
    except (AssertionError,ValueError) as e:entry['physical_error']=repr(e)
    if entry['physical_passed']:
     try:
      jobs=[prepare(plan,state,c) for c in CARS];entry['numerical_window']=all(len(j['T_columns'])>0 for j in jobs) and not (plan.get('encounter',{}).get('braking'))
      if plan.get('encounter',{}).get('braking'):entry['numerical_limitation']='analytic brake/long hold has unique own input in first32; keep as zero-call yielding diagnostic'
     except (AssertionError,KeyError) as e:entry['numerical_limitation']=repr(e)
    entry['eligible']=entry['physical_passed'] and limits['per_car_passed'] and entry['numerical_window'];table.append(entry);write(HERE/'results/proposal_table.json',table);print(json.dumps({key:entry[key] for key in ('id','physical_passed','eligible','deadlines')},default=str),flush=True)
   eligible=[r for r in table if r['eligible']];ranked=sorted(eligible,key=lambda r:(-r['score']['A'],-r['score']['P'],r['score']['B'],r['score']['J'],r['id']))
   if ranked and ranked[0]['id']!='R_old':
    selected=ranked[0];plan=load(f"results/proposals/{selected['id']}.json.gz");save('results/E_anchor.json.gz',plan);chosen={'passed':True,'source_cycle':k,'source':record(state),'source_id':content_id(record(state)),'old_id':content_id(old),'new_id':content_id(plan),'actors':[a,b],'selected':selected['id'],'event':event,'ranked':[r['id'] for r in ranked],'predicate':'nonzero actual speed + effective next signed straight progress; closed intervals of state-derived independent resource requests overlap','request_variant':'original set0, no disturbance','fallback_limit':{'request_sets':2,'offsets':[0,1,4,8],'used':0}};write(HERE/'event_registration.json',chosen);break
  if chosen:break
 write(HERE/'results/event_scan.json',{'passed':chosen is not None,'rows':scan,'optimizer_calls':0})
 assert chosen is not None,'No complete event candidate; numerical gate remains closed'
 print(json.dumps({'passed':True,'source_cycle':chosen['source_cycle'],'actors':chosen['actors'],'selected':chosen['selected'],'B':load('results/E_anchor.json.gz')['B0'],'native_calls':0}))
if __name__=='__main__':run()
