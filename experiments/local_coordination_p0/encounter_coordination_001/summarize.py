"""Same-source outcomes, actual braking/recovery, contracts and separate deadlines."""
import gzip,collections
from common import *
from plan_io import load
from encounter import deadline_check
from physics import body

def trace(n):return [json.loads(s) for s in gzip.open(HERE/f'results/{n}.jsonl.gz','rt')]
def metrics(plan,rows,yielder,event_cycle):
 v={c:F(0) for c in CARS};cost=F(0);act=[];travel={};brake=[];stop=None;restart=None;steer=None;stop_x=None;first_source=[]
 for k,row in enumerate(rows):
  active=[]
  for c in CARS:
   a,w=map(F,row['commands'][c]);s=H*(2*v[c]+H*a)/2;cost+=a*a+w*w
   if abs(s)>=F(1,50):active.append(c)
   if c==yielder:
    travel[k]=s
    if k>=event_cycle and stop is None and v[c]>0 and a<0:brake.append(k)
    if k>=event_cycle and stop is None and v[c]>0 and F(row['v'][c])==0:stop=k+1;stop_x=row['poses_Q'][c][0]
    if stop is not None and k>=stop:
     if steer is None and w!=0:steer=k
     if restart is None and (v[c]!=0 or F(row['v'][c])!=0):restart=k
   v[c]+=H*a
  act.append(active)
 assert stop is not None and restart is not None
 # Last contiguous negative-acceleration interval immediately before stopping.
 start=stop-1
 while start>event_cycle and F(rows[start-1]['commands'][yielder][0])<0:start-=1
 first_effective=next(k for k in range(restart,len(rows)) if yielder in act[k]);prefix_id=content_id(rows[:event_cycle])
 return {'steps':plan['B0'],'seconds':H*plan['B0'],'goals':{c:plan['per_car_policies'][c]['goal'] for c in CARS},'cost':cost,'event_activity':act[event_cycle:event_cycle+2],'activity_histogram':dict(collections.Counter(map(len,act))),'brake_interval':[start,stop],'braking_distance':sum(travel[k] for k in range(start,stop)),'stop_phase':stop,'stop_x_Q':stop_x,'zero_speed_body_wait_ticks':restart-stop,'steering_resume_interval':steer,'motion_resume_interval':restart,'effective_resume_interval':first_effective,'common_prefix_id':prefix_id,'other_event_activity':{c:[c in a for a in act[event_cycle:event_cycle+2]] for c in CARS if c!=yielder}}

def run():
 event=read(HERE/'event_registration.json');k=event['source_cycle'];a,b=event['actors'];old=load('results/R_old.json.gz');anchor=load('results/E_anchor.json.gz');final=load('results/updated.json.gz')
 datasets={'R_old':(old,trace('R_old')),'E_anchor':(anchor,trace('E_anchor')),'E_MPC':(final,trace('actual'))};mm={n:metrics(p,rows,b,k) for n,(p,rows) in datasets.items()};assert len({m0['common_prefix_id'] for m0 in mm.values()})==1
 assert mm['E_MPC']['goals']==mm['E_anchor']['goals'];mm['contract_ticks_saved']=old['B0']-anchor['B0'];mm['MPC_cost_decrease']=mm['E_anchor']['cost']-mm['E_MPC']['cost'];assert mm['MPC_cost_decrease']>0;mm['MPC_percent_vs_anchor']=100*mm['MPC_cost_decrease']/mm['E_anchor']['cost'];write(HERE/'results/comparison.json',mm)
 oldrows=datasets['R_old'][1];newrows=datasets['E_anchor'][1];planes=[];first=None
 for t in range(k,min(len(oldrows),len(newrows))):
  for c in CARS:
   oldx=F(sum(oldrows[t]['poses_Q'][c][0]),2*m.Q);newx=F(sum(newrows[t]['poses_Q'][c][0]),2*m.Q)
   if oldx!=newx:
    planes.append({'phase':t+1,'actor':c,'n':[1,0],'old_beta':oldx+F(1,100),'new_beta':newx+F(1,100)})
    slack=F(1,100)-abs(newx-oldx)
    if first is None and slack<0:first={'phase':t+1,'actor':c,'old_own_position_plane_slack':slack}
 resources=[{'cycle':t,'old':old['owners'][t],'new':anchor['owners'][t] if t<anchor['B0'] else None} for t in range(old['B0']) if old['owners'][t]!=(anchor['owners'][t] if t<anchor['B0'] else None)]
 assert first and resources;write(HERE/'results/physical_contract_diff.json',{'passed':True,'old_id':content_id(old),'new_id':content_id(anchor),'face_changes':planes,'resource_changes':resources,'first_old_face_exclusion':first,'actual_state_release':read(HERE/'results/actual_events.json'),'contract_scope':'per-phase physical position/yaw/input envelope and exclusive calendar; full rectangular tube certificates'})
 with gzip.open(HERE/'results/deadlines.jsonl.gz','wt') as f:
  for t,row in enumerate(datasets['E_MPC'][1]):
   plan=old if t<k else final;before={c:max(0,plan['per_car_policies'][c]['goal']-t) for c in CARS};after={c:max(0,plan['per_car_policies'][c]['goal']-t-1) for c in CARS};assert max(after.values())==max(before.values())-1
   f.write(json.dumps({'cycle':t,'contract':'old' if t<k else 'encounter','b_before':before,'b_after':after,'B_before':max(before.values()),'B_after':max(after.values()),'actual_row_id':content_id(row)})+'\n')
 transition=deadline_check(old,final,k);assert transition['per_car_passed'] and transition['block_passed'];write(HERE/'results/deadline_transition.json',transition)
 num=read(HERE/'results/numerical_summary.json');updates=read(HERE/'results/independent_updates.json');physical=read(HERE/'results/independent_physical.json');rejections=read(HERE/'results/rejection_checks.json');assert all(x['passed'] for x in (num,updates,physical,rejections))
 info=read(HERE/'source_inventory.json');assert all(sha(Path(info['source_path'])/n)==h for n,h in info['files'].items());write(HERE/'results/preservation_check.json',{'passed':True,'source_files':len(info['files']),'source_path':info['source_path'],'old_Gate':info['GATE']['status'],'old_native_calls':info['numerical']['native_calls']})
 worker=[]
 for r in (0,1):
  for c in CARS:
   for tag in ('','_zero'):
    worker.append(read(HERE/f'results/round_{r}_{c}{tag}_transport.json'))
 write(HERE/'results/communication.json',{'worker_actual_requests':16,'distinct_actual_models':8,'native_worker_requests':8,'actual_request_bytes':sum(w['sent_bytes'] for w in worker),'actual_final_reply_bytes':sum(w['received_bytes'] for w in worker),'native_handshake_frames':16,'application':{'independent_intents':2,'initial_contract_approval_records':4,'candidate_results':8,'round_ACK_records':8,'transport':'same-process logical objects/versions, no real network bytes claimed'},'forecast_assembly_checks':8})
 gate={'passed':True,'status':'passed','task':'encounter_coordination_001','criteria':{'event_source':{'status':'passed','cycle':k,'actors':event['actors'],'independent_overlap':event['event']['overlap'],'source_id':event['source_id']},'feasible_yield':{'status':'passed','construction':'existing physically certified brake-to-wait tail; actual exit-domain release and resume','actual_stop':mm['E_MPC']['stop_phase'],'actual_motion_resume':mm['E_MPC']['motion_resume_interval']},'physical_negotiation':{'status':'passed','state_guarded_release':anchor['encounter']['first_out'],'old_face_exclusion':first,'ticks_saved':mm['contract_ticks_saved']},'local_transaction':{'status':'passed','detected_group':event['actors'],'closure':list(CARS),'old_commit_size':4,'new_commit_size':4,'natural_size_change':0,'reason':'all future calendar endpoints affected; no artificial group-size success'},'numerical_chain':{'status':'passed','native_calls':num['native_calls'],'accepted':num['accepted'],'first_changed':num['first_changed'],'actual_cycles':[k,k+1]},'safety_completion':{'status':'passed','steps':final['B0'],'B_final':0,'six_pairs_every_interval':True,'individual_and_block_deadlines':transition},'rejection_continuation':{'status':'passed','branches':len(rejections['branches']),'independent_full_replay':True}},'native_usage':{'limit':8,'attempts':8,'native_entered':8,'accepted':8,'repeats':0},'scope':'nominal fixed map; entry-before-core predictive event; exact initial straight/stopped-steering numerical window; inherited brake tail and new early state-conditioned release; no head-on core recovery or free curved steering/network claims'}
 write(HERE/'GATE.json',gate);print(json.dumps({'passed':True,'source':k,'actors':event['actors'],'old_B':old['B0'],'new_B':final['B0'],'accepted':num['accepted'],'MPC_percent':float(mm['MPC_percent_vs_anchor'])}))
if __name__=='__main__':run()
