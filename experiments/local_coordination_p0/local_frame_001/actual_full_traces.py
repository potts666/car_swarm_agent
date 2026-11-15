"""Stitch recorded actual first actions to the content-bound accepted real tail."""
import copy,gzip,hashlib
from common import *
from plan_io import load
from physics import step
from audit import audit

def run():
 events=read(HERE/'results/events.json');digests={}
 for org in ('C','L'):
  first=next(e for e in events if e['organization']==org and e['round']==1);last=next(e for e in events if e['organization']==org and e['round']==2);state=initial_state();old=load('results/old.json.gz');rows=[]
  for k in range(first['source']['cycle']):
   row=step(state,{c:old['controls'][c][k] for c in CARS},old['owners'][k],8);row['cycle']=state['cycle'];rows.append(row)
  row=copy.deepcopy(first['actual_first']);row['cycle']=first['successor']['cycle'];rows.append(row);mix=next(m for m in last['mixes'] if m['D_new'] and m['X_new']);cert=read(HERE/mix['certificate']);assert cert['source']==last['source']['cycle']
  data=''.join(json.dumps(r,default=str)+'\n' for r in rows).encode()+gzip.decompress((HERE/cert['trace']).read_bytes());(HERE/f'results/actual_{org}.jsonl.gz').write_bytes(gzip.compress(data,mtime=0));digests[org]=hashlib.sha256(data).hexdigest();assert len(data.splitlines())==old['B0']
 assert digests['C']==digests['L'];result=audit(load('results/C_final_plan.json.gz'),trace=str(HERE/'results/actual_C.jsonl.gz'));assert result['passed'];write(HERE/'results/actual_trace_acceptance.json',{'passed':True,'full_trace_uncompressed_sha':digests,'C_L_identical_actual_rows':True,'independent_trace_physics':result,'new_optimizer_calls':0});print('PASS full actual trace composition and independent trace audit',flush=True)
if __name__=='__main__':run()
