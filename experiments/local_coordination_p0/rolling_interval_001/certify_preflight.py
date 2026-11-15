"""Independent rational ranks and actual worker-byte/matrix fingerprints."""
from common import *
from plan_io import load

def rank(rows):
 A=[[F(x) for x in row] for row in rows];r=0
 for j in range(64):
  pivot=next((i for i in range(r,len(A)) if A[i][j]),None)
  if pivot is None:continue
  A[r],A[pivot]=A[pivot],A[r];v=A[r][j];A[r]=[x/v for x in A[r]]
  for i in range(r+1,len(A)):
   v=A[i][j]
   if v:A[i]=[x-v*y for x,y in zip(A[i],A[r])]
  r+=1
 return r

def run():
 result=read(HERE/'results/rolling_preflight.json');assert result['passed'];proofs=[]
 for p in sorted((HERE/'results/preflight_samples').glob('*_job.json.gz')):
  job=load(str(p.relative_to(HERE)));lattice=job['lattice'];er=rank(lattice['E']);tr=rank(lattice['T']);assert tr==len(lattice['T']) and er+tr==64;assert all(sum(F(a)*F(b) for a,b in zip(eq,col))==0 for eq in lattice['E'] for col in lattice['T']);stem=str(p)[:-len('_job.json.gz')];response=read(stem+'_response.json');request=Path(stem+'_request.json');assert response['request_sha']==sha(request) and read(request)['job']==job;info=response['assembly'];matrix=load(info['matrix_object']);assert info['matrix_id']==content_id(matrix);assert info['model_id']==job['model_id'];proofs.append({'model_id':job['model_id'],'rank_E':er,'rank_T':tr,'matrix_id':info['matrix_id'],'request_sha':sha(request),'worker_solve_called':response['solve_called']})
 rows=[json.loads(x) for x in (HERE/'results/preflight_cycles.jsonl').read_text().splitlines()];reg=result['interval'];assert len(rows)==4*(reg['end_exclusive']-reg['start']) and all(r['passed'] for r in rows);assert {(r['cycle'],r['actor']) for r in rows}=={(k,c) for k in range(reg['start'],reg['end_exclusive']) for c in CARS}
 write(HERE/'results/preflight_rank_worker_certificate.json',{'passed':True,'samples':proofs,'all_car_cycles':len(rows),'new_optimizer_calls':0});print('PASS rational rank + genuine worker fingerprint samples',len(proofs),flush=True)
if __name__=='__main__':run()
