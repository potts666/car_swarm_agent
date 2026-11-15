"""Real-source rejection continuations and zero-solve external-frame expansion."""
import copy,time,traceback
from common import *
from plan_io import load,save,restore,record
from contract import validate_binding,patch,replay_suffix,guard
from frame import validate_frame_plan,expansion_closure,certify,lease_table,physical_id
from protocol import Wire,choose
import sys
def propose(*args):
 # The legacy initializer temporarily imports from sources; never leak that
 # search path into subsequently spawned own-model workers.
 previous=sys.path[:]
 try:
  from encounter import propose as construct
  return construct(*args)
 finally:sys.path[:]=previous
from receivers import reference
from physics import step,body
from curve_model import generate,evaluate
from transport import exchange

def run():
 assert read(HERE/'results/numerical_complete.json')['passed'];sel=read(HERE/'results/source_selection.json');D=sel['D'];X=sel['X'];frame=read(HERE/'outer_frame.json');cp=load('results/L_r2_checkpoint.json.gz');state=restore(cp['state']);old,anchor=cp['old'],cp['anchor'];jobs={c:load(f'results/L_r2_{c}_job.json.gz') for c in D+[X]};ads={c:read(HERE/f'results/L_r2_{c}_admission.json') for c in jobs};assert all(a['accepted'] for a in ads.values());rejections=[]
 for name in ('damaged_D_certificate','D_missing_ACK'):
  damaged=copy.deepcopy(ads)
  if name=='damaged_D_certificate':
   bad=copy.deepcopy(read(HERE/f'results/workers/L_r2_{D[0]}_response.json'));bad['model_id']='damaged_fixture';rejected=False
   try:validate_binding(jobs[D[0]],anchor,state,bad)
   except AssertionError:rejected=True
   assert rejected;write(HERE/'results/rejections/damaged_raw_fixture.json',bad);damaged[D[0]]={'accepted':False,'model_id':jobs[D[0]]['model_id'],'reason':'raw identity rejected; own legal anchor retained'}
  candidate=patch(anchor,state,list(jobs.values()),[damaged[c] for c in jobs]);acks=D+[X] if name=='damaged_D_certificate' else [D[0],X];selected,event=choose(old,candidate,state,frame,D,X,'L',Wire(name),acks=acks,phase='fixture');assert event['X_committed'];assert selected['controls'][X]==candidate['controls'][X]
  if name=='D_missing_ACK':assert not event['D_committed'] and all(selected['controls'][c]==old['controls'][c] for c in D)
  k=state['cycle'];commands={c:selected['controls'][c][k] for c in CARS};test=copy.deepcopy(state);guard(selected,test);actual=step(test,commands,selected['owners'][k],8);assert tuple(map(F,commands[X]))==tuple(map(F,ads[X]['commands'][0])) and tuple(map(F,commands[X]))!=tuple(map(F,old['controls'][X][k]))
  stem=f'rejections/{name}';save(f'results/{stem}_plan.json.gz',selected);certificate=replay_suffix(selected,state,stem);changed=[t for t in range(k,k+32) if selected['controls'][X][t]!=old['controls'][X][t]];result={'name':name,'fixture':True,'source':record(state),'transaction':event,'plan':f'results/{stem}_plan.json.gz','certificate':f'results/{stem}_certificate.json','actual_first':actual,'actual_successor':record(test),'external_new_first_executed':True,'external_changed_output_ticks':changed,'external_candidate_model':jobs[X]['model_id'],'illegal_commits':0,'illegal_executions':0,'passed':certificate['passed']};rejections.append(result);write(HERE/f'results/{stem}_event.json',result);print('PASS rejection',name,'X new first continues',flush=True)
 # Fork only after the actual second-round first action, not its prediction.
 current=load('results/L_final_plan.json.gz');source=restore(load('results/L_final_source.json.gz'));path_before=sys.path[:];future=propose(current,source,*D);assert sys.path==path_before,'proposal import leaked module search path';future['frame_branch']='extended';future['outer_frame_id']=frame['outer_frame_id'];future=json.loads(json.dumps(future,default=str));assert all(future['controls'][c][:source['cycle']]==current['controls'][c][:source['cycle']] for c in CARS)
 try:validate_frame_plan(future,frame)
 except AssertionError as e:restriction=repr(e)
 else:raise AssertionError('out-of-frame extension unexpectedly admitted')
 closure=expansion_closure(current,future,frame);assert closure['participants']==sorted(CARS)
 # A definite corner leaves B2's old published single-chord body box.
 reserve=TUBE+F(1,40)+15*H*H/8;example=None
 for k in range(855,924):
  oldpose=tuple(m.I.raw(*q) for q in reference(current)[k]['poses_Q']['B2']);newpose=tuple(m.I.raw(*q) for q in reference(future)[k]['poses_Q']['B2']);box=m.axis_box(body(oldpose,'B2'));bounds=[F(box[0],m.Q)-reserve,F(box[1],m.Q)+reserve,F(box[2],m.Q)-reserve,F(box[3],m.Q)+reserve]
  for corner in body(newpose,'B2'):
   if F(corner[0].u,m.Q)<bounds[0] or F(corner[0].l,m.Q)>bounds[1] or F(corner[1].u,m.Q)<bounds[2] or F(corner[1].l,m.Q)>bounds[3]:example={'cycle':k,'actor':'B2','old_allowed_box':bounds,'outside_corner_Q':[[z.l,z.u] for z in corner]};break
  if example:break
 assert example;write(HERE/'results/extension_old_frame_rejection.json',{'rejected':True,'restriction':restriction,'closure':closure,'outside_old_occupancy':example,'new_intent_registered_before_discovery':True,'old_lease':frame['leases'],'new_lease':lease_table(future,True)})
 newframe=certify('extended_frame',['extended']);future['outer_frame_id']=newframe['outer_frame_id'];validate_frame_plan(future,newframe);wire=Wire('extension');votes={}
 # Proposed occupancy catalog was completely registered by certify before queries.
 for c in closure['participants']:
  payload={'old_outer_frame_id':frame['outer_frame_id'],'new_outer_frame_id':newframe['outer_frame_id'],'own_new_plan':content_id(future['controls'][c]),'lease_table':newframe['leases'],'deadline':future['per_car_policies'][c]['goal']};wire.send('frame_directory',c,'prepare_new_frame',payload,'extension');votes[c]=wire.send(c,'frame_directory','ACK_frame',payload,'extension')
 branches=[]
 for name,acks in [('success',set(CARS)),('missing_ACK',set(CARS)-{'A2'})]:
  selected=future if set(closure['participants'])<=acks else current
  if name=='missing_ACK':wire.send('fixture_dropper','frame_directory','drop_ACK_fixture',{'actor':'A2','reason':'counterfactual missing delivery; prior frame remains effective'},'fixture')
  passed=set(closure['participants'])<=acks
  for c in closure['participants']:wire.send('frame_directory',c,'commit_new_frame' if passed else 'retain_old_frame',{'outer_frame_id':selected['outer_frame_id'],'ACK_fixture_missing':None if passed else 'A2'},'extension' if passed else 'fixture')
  save(f'results/extension/{name}_plan.json.gz',selected);cert=replay_suffix(selected,source,f'extension/{name}');branches.append({'name':name,'plan':f'results/extension/{name}_plan.json.gz','certificate':f'results/extension/{name}_certificate.json','source':record(source),'ACK':sorted(acks),'participants':closure['participants'],'new_frame_committed':passed,'outer_frame_id':selected['outer_frame_id'],'goals':cert['goals'],'passed':cert['passed']});print('PASS extension',name,flush=True)
 # Fixed new outer frame permits fresh local own-model updates without global ACK.
 split=[]
 for c in D+[X]:
  job,cert=generate(future,source,c,'O-E');validate_binding(job,future,source);assert evaluate(job,[F(0)]*64)['passed'];save(f'results/extension/split_{c}_job.json.gz',job);stem=HERE/f'results/extension/split_{c}';result,meta=exchange(job,'assemble',stem);split.append({'actor':c,'model_id':job['model_id'],'assembly':result['assembly'],'participants':D if c in D else [c],'outer_dependency':newframe['outer_frame_id'],'own_tail_candidate':'zero-increment actual shifted legal tail; no new numerical claim','optimizer_calls':0})
 # Recompute dependency components from the committed public lease relation.
 components=[{c} for c in CARS];joining=[]
 for lease in newframe['leases'].values():
  members=set(lease['members']);touching=[g for g in components if g & members];joined=set().union(*touching);components=[g for g in components if not g & members]+[joined]
  if len(members)>1:joining.append({'edge':sorted(members),'reason':'same public lease requires atomic inner owner partition'})
 blocks=sorted([sorted(g) for g in components]);assert sorted(D) in blocks and [X] in blocks and [sel['fourth']] in blocks
 split_discovery={'blocks':blocks,'internal_edges':joining,'potential_cross_edges':'registered directory contains all six pairs; every cross edge discharged by new fixed-frame separation certificate','new_frame':newframe['outer_frame_id'],'private_changes_need_no_external_reACK':True}
 write(HERE/'results/post_branches.json',{'passed':True,'rejections':rejections,'extension':{'source':record(source),'closure':closure,'old_frame_rejection':restriction,'outside_occupancy':example,'branches':branches,'new_outer_frame':newframe['outer_frame_id'],'post_frame_independent_blocks':blocks,'post_frame_dependency_discovery':split_discovery,'split_zero_candidates':split,'old_frame_remains_valid_until_all_ACK':True},'new_optimizer_calls':0})
if __name__=='__main__':
 try:run()
 except Exception as e:write(HERE/'results/post_branches_failure.json',{'layer':'fallback_or_extension','error':repr(e),'traceback':traceback.format_exc(),'new_optimizer_calls':0});raise
