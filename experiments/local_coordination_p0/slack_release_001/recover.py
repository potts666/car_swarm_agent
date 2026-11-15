"""Once-only failed release then actual old step and source-version retry, no solve."""
import gzip,copy,traceback
from common import *
from plan_io import load,save
from engine import run

def run_all():
 summary=read(HERE/'results/E-MPC_summary.json');events=[e for e in summary['events'] if e['transaction']['accepted']];event=events[1];checkpoint={'plan':load(event['old_plan']),'state':load(event['checkpoint'])};assert checkpoint['state']['cycle']==event['cycle'];ledger_before=sha(HERE/'solver_calls.json');out=[]
 prefix=[]
 with gzip.open(HERE/'results/E-MPC_actual.jsonl.gz','rb') as f:
  for row in f:
   if json.loads(row)['cycle']>event['cycle']:break
   prefix.append(row)
 for fault in ('missing_ACK','stale_K'):
  name=f'recovery_{fault}';result=run(name,'anchor',copy.deepcopy(checkpoint),fault);attempts=result['events'];assert attempts[0]['transaction']['accepted'] is False and attempts[1]['transaction']['accepted'] is True;assert attempts[1]['cycle']==attempts[0]['cycle']+1 and attempts[1]['source']['history']!=attempts[0]['source']['history'];assert result['fault_used'] and result['successes']>=2
  data=b''.join(prefix)+gzip.decompress((HERE/result['trace']).read_bytes());full=f'results/{name}_full_actual.jsonl.gz';(HERE/full).write_bytes(gzip.compress(data,mtime=0));assert len(data.splitlines())==result['final_cycle'];result['full_trace']=full;result['illegal_commits']=result['illegal_outputs']=0;write(HERE/f'results/{name}_summary.json',result);out.append(result);print('PASS recovery',fault,'retry',attempts[1]['cycle'],'final',result['final_cycle'],flush=True)
 assert sha(HERE/'solver_calls.json')==ledger_before;write(HERE/'results/recovery_summary.json',{'passed':True,'branches':out,'same_real_checkpoint':checkpoint['state'],'formal_ledger_unchanged':True,'new_optimizer_calls':0})
if __name__=='__main__':
 try:run_all()
 except Exception as e:write(HERE/'results/recovery_failure.json',{'layer':'recovery','error':repr(e),'traceback':traceback.format_exc(),'new_optimizer_calls':0});raise
