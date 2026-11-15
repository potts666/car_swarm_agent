"""Post-failure common-rule regression; no new numerical attempts."""
import copy,gzip
from common import *
from preflight import simultaneous
from local_mpc import prepare,assemble
from physics import step
from transaction import check_binding,select
from discovery import certify
from audit import audit

def run():
 root=HERE/'results/repair_v2';root.mkdir(exist_ok=True);old=json.loads(gzip.decompress((HERE/'results/init_0.json.gz').read_bytes()));new=simultaneous(old,True)
 write(root/'plan_summary.json',{'source_plan':content_id(old),'new_plan':content_id(new),'B0':new['B0'],'goal_ticks':{c:new['per_car_policies'][c]['goal'] for c in CARS},'old_goal_ticks':{c:old['per_car_policies'][c]['goal'] for c in CARS},'all_absolute_deadlines_nonincrease':True,'new_native_attempts':0,'scope':'zero-call repaired fork; not relabeled numerical v1'})
 (root/'anchor.json.gz').write_bytes(gzip.compress(json.dumps(new,sort_keys=True,default=str).encode(),mtime=0))
 s=initial_state();zero=[]
 with gzip.open(root/'actual.jsonl.gz','wt') as f:
  for k in range(new['B0']):
   if k<2:
    idx=certify(old if k==0 else new,new,participants=CARS,cycle=k);(root/f'round_{k}_index.json.gz').write_bytes(gzip.compress(json.dumps(idx,sort_keys=True,default=str).encode(),mtime=0));jobs={}
    for c in CARS:
     j=prepare(new,s,c);check_binding(j,new,s,c,j['receiving_certificate']);P,q,A,b,cones,settings=assemble(j)
     import clarabel
     solver=clarabel.DefaultSolver(P,q,A,b,cones,settings)
     write(root/f'round_{k}_{c}_model.json',j);jobs[c]={'model_id':j['model_id'],'zero_anchor_passed':True,'native_constructed':True,'solve_called':False}
    zero.append({'round':k,'actual_source':{c:pose_rows(s['poses'][c]) for c in CARS},'jobs':jobs,'index':content_id(idx)})
   row=step(s,{c:new['controls'][c][k] for c in CARS},new['owners'][k],4);row['cycle']=s['cycle'];f.write(json.dumps(row,default=str)+'\n')
 audit(new,root/'actual.jsonl.gz',root/'acceptance.json');write(root/'zero_two_rounds.json',{'passed':True,'rounds':zero,'native_attempts':0})
 print(json.dumps({'passed':True,'B0':new['B0'],'native_attempts':0}))
if __name__=='__main__':run()
