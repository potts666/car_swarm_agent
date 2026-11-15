"""One actual-state observer and release/retiming loop for all three arms."""
import argparse,copy,gzip,time,traceback
from common import *
from plan_io import load,save,source,restore,record
from canonical import point,pose_error
from release import clear,verify_clear
from schedules import propose,deadlines,validate_prefix,intervals_valid,goals
from frames import certify,slot_descriptors,bind,contains,discover,blocks,own_resource_proof,validate_publication,validate_discovery
from contracts import guards,continuation,physical_id
from wire import Wire
from physics import step

def frozen():
 for n,h in read(HERE/'executed_source_versions.json').items():assert sha(HERE/n)==h,n

def submit(old,new,state,proof,meta,oldframe,newframe,wire,name,fault=None):
 verify_clear(old,state,proof);validate_publication(oldframe);validate_publication(newframe);contains(newframe,new);closure=discover(old,new,oldframe,newframe,state['cycle']);validate_discovery(closure,discover(old,new,oldframe,newframe,state['cycle']));participants=closure['participants'];proposalid=content_id({'source_token':proof['token'],'new_physical':physical_id(new),'new_frame':newframe['outer_frame_id']});ack=[]
 for c in participants:
  msg={'proposal_id':proposalid,'source_token':proof['token'],'actor':c,'old_outer_slot':oldframe['outer_slots'][c],'new_outer_slot':newframe['outer_slots'][c],'inner_version':new['inner_version'],'own_plan_id':content_id(new['controls'][c]),'deadline':new['per_car_policies'][c]['goal']};wire.send('resource_directory',c,'prepare_release',msg,'internal' if meta['kind']=='internal' else 'extension')
  if not(fault=='missing_ACK' and c==participants[-1]):ack.append(wire.send(c,'resource_directory','ACK_release',msg,'internal' if meta['kind']=='internal' else 'extension'))
 valid=True;reason='all affected endpoints accepted'
 if fault=='stale_K':
  broken=copy.deepcopy(proof);broken['source']['K_id']='damaged_fixture';valid=False
  try:verify_clear(old,state,broken)
  except AssertionError as e:reason=repr(e)
  else:raise AssertionError('stale K not rejected')
 if len(ack)!=len(participants):valid=False;reason='missing real affected endpoint ACK fixture'
 selected=new if valid else old
 for c in participants:wire.send('resource_directory',c,'commit_release' if valid else 'retain_old_obligation',{'proposal_id':proposalid,'own_plan_id':content_id(selected['controls'][c]),'outer_slot':selected['outer_slots'][c]},'internal' if meta['kind']=='internal' else 'extension')
 return selected,{'proposal_id':proposalid,'accepted':valid,'reason':reason,'participants':participants,'ACK':[a['actor'] for a in ack],'closure':closure,'old_outer_frame':oldframe['outer_frame_id'],'new_outer_frame':selected['outer_frame_id'],'old_slots':oldframe['outer_slots'],'new_slots':selected['outer_slots'],'source_token':proof['token'],'fixture':fault,'name':name}

def candidate(old,state,outgoing,proof,wire,stem,fault=None):
 k=state['cycle'];oldframe=read(HERE/old['frame_file']);j=old['order'].index(outgoing)
 if j+1==len(old['order']):return old,None,{'reason':'no successor with strictly improvable deadline'}
 nxt=old['order'][j+1];lower=max(k,old['per_car_policies'][nxt]['approach_end']);upper=old['per_car_policies'][nxt]['service_start']-1;attempts=[]
 for start in range(lower,upper+1):
  assert len(attempts)<load('results/R_initial.json.gz')['B0'],'finite candidate bound'
  try:
   new,meta=propose(old,state,outgoing,start);meta['owner_domain']=own_resource_proof(new)
   if meta['kind']=='internal':contains(oldframe,new);newframe=oldframe
   else:newframe=certify(new)
   new=bind(new,newframe);cert=continuation(new,state,f'{stem}/candidate_{start}');assert cert['passed'];verify_clear(old,state,proof)
  except (AssertionError,ValueError) as e:
   attempts.append({'start':start,'accepted_physical':False,'limitation':repr(e)});continue
  attempts.append({'start':start,'accepted_physical':True,'candidate_id':physical_id(new),'certificate':f'results/{stem}/candidate_{start}_certificate.json'});selected,transaction=submit(old,new,state,proof,meta,oldframe,newframe,wire,stem,fault);save(f'results/{stem}/old.json.gz',old);save(f'results/{stem}/candidate.json.gz',new);save(f'results/{stem}/source.json.gz',record(state));result={'cycle':k,'clear':proof,'meta':meta,'search_range':[lower,upper],'maximum_checks':load('results/R_initial.json.gz')['B0'],'attempts':attempts,'transaction':transaction,'source':record(state),'old_plan':f'results/{stem}/old.json.gz','candidate':f'results/{stem}/candidate.json.gz','checkpoint':f'results/{stem}/source.json.gz'};write(HERE/f'results/{stem}/event.json',result);return selected,result,None
 return old,None,{'reason':'no certified earlier integer start','range':[lower,upper],'attempts':attempts}

def numeric_round(plan,state,actors,label):
 from curve_model import generate
 from local_mpc import admit
 from contracts import binding
 from transport import exchange
 frozen();start=time.monotonic();jobs={};admissions={};stats=[];save(f'results/numeric/{label}_checkpoint.json.gz',{'plan':plan,'state':record(state)})
 for c in actors:
  job,cert=generate(plan,state,c,'O-E');binding(job,plan,state);jobs[c]=job;save(f'results/numeric/{label}_{c}_job.json.gz',job);save(f'results/numeric/{label}_{c}_derivatives.json.gz',cert)
 for c in actors:
  job=jobs[c];stem=HERE/f'results/workers/{label}_{c}';stem.parent.mkdir(parents=True,exist_ok=True);assembled,ameta=exchange(job,'assemble',stem.with_name(stem.name+'_precheck'))
  def release(info):
   frozen();ledger=read(HERE/'solver_calls.json');assert len(ledger)<8 and not any(r['label']==label and r['actor']==c for r in ledger);assert info==assembled['assembly'];ledger.append({'attempt':len(ledger)+1,'label':label,'actor':c,'cycle':state['cycle'],'model_id':job['model_id'],'source_history':state['history'],'outer_slot':job['outer_frame_id'],'released_at':time.time(),'assembly_id':info['matrix_id'],'status':'released'});write(HERE/'solver_calls.json',ledger)
  raw,meta=exchange(job,'solve',stem,on_ready=release);ledger=read(HERE/'solver_calls.json');ledger[-1].update(status=raw['status'],native_entered_at=raw['native_entered_at'],native_finished_at=raw['native_finished_at'],solve_time=raw['solve_time'],r_prim=raw['r_prim'],r_dual=raw['r_dual'],obj_val=raw['obj_val'],obj_val_dual=raw['obj_val_dual']);write(HERE/'solver_calls.json',ledger);binding(job,plan,state,raw);ad=admit(job,raw,state) if raw['status'] in ('Solved','AlmostSolved') else {'accepted':False,'model_id':job['model_id'],'reason':raw['status']};admissions[c]=ad;write(HERE/f'results/numeric/{label}_{c}_admission.json',ad);stats.append({'actor':c,'status':raw['status'],'accepted':ad['accepted'],'solve_time':raw['solve_time'],'transport':meta,'assembly_transport':ameta});print('WORKER',label,c,raw['status'],'accepted',ad['accepted'],flush=True)
 new=copy.deepcopy(plan)
 for c,ad in admissions.items():
  if ad['accepted']:new['controls'][c][state['cycle']:state['cycle']+32]=ad['commands']
 # Every actual combination is inside the common published corridor and same
 # frozen owner/receiver contract; check all2^2 complete tails explicitly.
 mixes=[]
 for mask in range(4):
  p=copy.deepcopy(plan)
  for b,c in enumerate(actors):
   if mask&(1<<b) and admissions[c]['accepted']:p['controls'][c][state['cycle']:state['cycle']+32]=admissions[c]['commands']
  path=f'numeric/{label}_mix{mask}';save(f'results/{path}_plan.json.gz',p);proof=continuation(p,state,path);mixes.append({'mask':mask,'plan':f'results/{path}_plan.json.gz','certificate':f'results/{path}_certificate.json','passed':proof['passed']})
 wire=Wire('E-MPC_numeric');groups=blocks(plan,state['cycle']);acks=[]
 for group in groups:
  if not set(group)&set(actors):continue
  version=content_id({'group':group,'own_inputs':{c:content_id(new['controls'][c]) for c in group},'slots':{c:new['outer_slots'][c] for c in group},'history':state['history']})
  for c in group:
   msg={'group':group,'inner_commit_id':version,'actor':c,'own_plan_id':content_id(new['controls'][c]),'outer_slot':new['outer_slots'][c],'source_history':state['history']};wire.send(group[0],c,'prepare_MPC',msg,'steady');acks.append(wire.send(c,group[0],'ACK_MPC',msg,'steady'));wire.send(group[0],c,'commit_MPC',msg,'steady')
 save(f'results/numeric/{label}_committed.json.gz',new);write(HERE/f'results/numeric/{label}_pending_event.json',{'label':label,'source':record(state),'plan_before':physical_id(plan),'plan_after':physical_id(new),'actors':actors,'jobs':{c:j['model_id'] for c,j in jobs.items()},'workers':stats,'ACK':acks,'groups':groups,'mixes':mixes,'wall_before_execution':time.monotonic()-start,'old_new_obligation':{c:{'old_K':jobs[c]['receiver']['K_id'],'receiver_phase':state['cycle']+32,'new_own_plan':content_id(new['controls'][c])} for c in actors}});return new

def run(name,mode,start_checkpoint=None,fault=None):
 reg=read(HERE/'event_selection_rule.json');wire=Wire(name);initial=load('results/initial_bound.json.gz');plan=copy.deepcopy(initial);state=initial_state();successes=0;events=[];observations=[];deadlines_trace=[];numeric=[];windows=[];pending=None;pending_queue=[];seen=set();fault_used=False;firsthits={c:{} for c in CARS};startclock=time.monotonic()
 if start_checkpoint:
  plan=start_checkpoint['plan'];state=restore(start_checkpoint['state']);successes=plan['release_count']
 tracepath=HERE/f'results/{name}_actual.jsonl.gz';sourceScan=HERE/f'results/{name}_observer.jsonl.gz'
 with gzip.open(tracepath,'wt') as trace,gzip.open(sourceScan,'wt') as scanner:
  while state['cycle']<plan['B0']:
   k=state['cycle'];proofs={c:clear(plan,state,c) for c in CARS}
   for c,p in proofs.items():
    for term in ('geometric_clear','future_clear','receiver_valid','post_exit_phase','full_Clear'):
     if p[term] and term not in firsthits[c]:firsthits[c][term]=k
    scanner.write(json.dumps({'cycle':k,'actor':c,**{n:p[n] for n in ('geometric_clear','support_gap','future_clear','future_min_gap','post_exit_phase','receiver_valid','body_pose_error','full_Clear')}},default=str)+'\n')
   owner=plan['owners'][k];event=None
   if owner and proofs[owner]['full_Clear']:
    idx=plan['order'].index(owner);eligible=idx+1<len(plan['order']) and plan['per_car_policies'][plan['order'][idx+1]]['service_start']>k
    if mode=='fixed':
     if eligible and not any(r['actor']==owner for r in observations):observations.append({'actor':owner,'cycle':k,'Clear':proofs[owner],'unused_obligation':plan['release_cuts'].get(owner,plan['per_car_policies'][owner]['goal'])-k,'calendar_unchanged':True})
    elif eligible:
     key=(state['history'],content_id(plan),owner)
     if key not in seen:
      seen.add(key);stem=f'{name}/release_attempt_{len(events)+1}';use_fault=fault if fault and not fault_used else None;old=plan;plan,event,restriction=candidate(plan,state,owner,proofs[owner],wire,stem,use_fault)
      if event:
       if use_fault:fault_used=True
       events.append(event);write(HERE/f'results/{name}_events.json',events)
       if event['transaction']['accepted']:
        successes+=1;assert plan['release_count']==successes;print('RELEASE',name,k,owner,'->',event['meta']['incoming'],event['meta']['kind'],'goals',goals(plan),'closure',event['transaction']['participants'],flush=True)
        if mode=='numeric' and successes<=2:
         item={'event':successes,'not_before':k,'actors':None,'round':1}
         if pending:pending_queue.append(item)
         else:pending=item
      elif restriction:observations.append({'actor':owner,'cycle':k,'restriction':restriction})
   if mode=='numeric' and pending:
    from check_zero import source_models
    if pending['round']==1:
     moving=[c for c in CARS if state['v'][c]!=0 and abs(H*(2*state['v'][c]+H*F(plan['controls'][c][k][0]))/2)>=F(1,50)];pair=None
     for actors in itertools_pairs(moving):
      try:source_models(plan,state,actors,f'actual_window{pending["event"]}_probe',transport=False);pair=list(actors);break
      except (AssertionError,ValueError) as e:observations.append({'cycle':k,'window':pending['event'],'actors':actors,'limitation':repr(e)})
     if pair:pending['actors']=pair;pending['first_cycle']=k
    if pending['actors']:
     label=f'e{pending["event"]}_r{pending["round"]}';plan=numeric_round(plan,state,pending['actors'],label);numeric.append(label);pending['label']=label
   guards(plan,state);before=record(state);Bbefore=max(max(d-k,0) for d in goals(plan).values());t=time.monotonic();actual=step(state,{c:plan['controls'][c][k] for c in CARS},plan['owners'][k],8);actual['cycle']=state['cycle'];trace.write(json.dumps(actual,default=str)+'\n');deadlines_trace.append({'cycle':k,'deadlines':goals(plan),'b':{c:max(d-k,0) for c,d in goals(plan).items()},'B_before':Bbefore,'B_after':max(max(d-state['cycle'],0) for d in goals(plan).values()),'Phi':sum(goals(plan).values())})
   if mode=='numeric' and pending and pending.get('label'):
    label=pending['label'];entry=read(HERE/f'results/numeric/{label}_pending_event.json');entry.update(successor=record(state),actual_first=actual,execution_wall=time.monotonic()-t);write(HERE/f'results/numeric/{label}_event.json',entry)
    if pending['round']==1:pending['round']=2;pending.pop('label');windows.append({'event':pending['event'],'first_cycle':pending['first_cycle'],'actors':pending['actors']})
    else:pending=pending_queue.pop(0) if pending_queue else None
   if mode=='anchor' and event and event['transaction']['accepted'] and successes<=2:
    # Forecast window discovery uses subsequent real anchor states in check_zero.
    pass
  save(f'results/{name}_final_plan.json.gz',plan);save(f'results/{name}_final_state.json.gz',record(state))
 save(f'results/{name}_deadline_trace.json.gz',deadlines_trace);write(HERE/f'results/{name}_observer_summary.json',{'first_hits':firsthits,'opportunities_or_limits':observations});result={'passed':True,'name':name,'mode':mode,'start_cycle':start_checkpoint['state']['cycle'] if start_checkpoint else 0,'final_cycle':state['cycle'],'goals':goals(plan),'events':events,'successes':len([e for e in events if e['transaction']['accepted']]),'windows':windows,'numeric_rounds':numeric,'fault_used':fault_used,'wall':time.monotonic()-startclock,'trace':str(tracepath.relative_to(HERE)),'plan':f'results/{name}_final_plan.json.gz','B_final':0};write(HERE/f'results/{name}_summary.json',result);return result

def itertools_pairs(cars):return __import__('itertools').combinations(cars,2)
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--analytic',action='store_true');ap.add_argument('--numeric',action='store_true');args=ap.parse_args()
 try:
  if args.analytic:
   p=load('results/R_initial.json.gz');frame=certify(p,slot_descriptors(p,True));p=bind(p,frame);save('results/initial_bound.json.gz',p);write(HERE/'results/initialization_ACK.json',{c:{'outer_slot':p['outer_slots'][c],'ACK':True} for c in CARS});run('R-fixed','fixed');run('E-anchor','anchor')
  elif args.numeric:
   assert not read(HERE/'solver_calls.json');frozen();run('E-MPC','numeric')
 except Exception as e:write(HERE/f'results/{"analytic" if args.analytic else "numeric"}_failure.json',{'layer':'engine','error':repr(e),'traceback':traceback.format_exc(),'formal_attempts':len(read(HERE/'solver_calls.json'))});raise
