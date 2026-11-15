"""Independent saved-byte/matrix reassembly checks; never calls construct/solve."""
import gzip,time
from common import *
from plan_io import load
from local_mpc import assemble

def run():
 beg=time.monotonic();text=(HERE/'results/rolling_cycles.jsonl').read_text();ns=[json.loads(x) for x in text.rsplit('\n',1)[0].splitlines()];checked=[]
 for n in ns:
  for c,row in n['rows'].items():
   if not row.get('raw_object'):continue
   job=load(f'results/objects/{row["model_object"]}.json.gz');raw=load(f'results/objects/{row["raw_object"]}.json.gz');key=content_id({'model':job['model_id'],'raw':row['raw_object'],'compiler':sha(HERE/'local_mpc.py')});path=HERE/f'results/matrix_stream_checks/{key}.json'
   if path.exists():checked.append(read(path));continue
   stem=HERE/f'results/workers/k{n["cycle"]}_{c}';payload=gzip.decompress(Path(str(stem)+'_request.json.gz').read_bytes());request=json.loads(payload);assert request['mode']=='solve' and request['job']==job;assert hashlib.sha256(payload).hexdigest()==raw['request_sha'];ready=json.loads(gzip.decompress(Path(str(stem)+'_ready.json.gz').read_bytes()));assert ready['assembly']==raw['assembly'];info=raw['assembly'];matrix=load(info['matrix_object']);assert content_id(matrix)==info['matrix_id'];assert len(raw['x'])==65 and len(raw['s'])==len(raw['z'])==info['shape'][0]
   P,q,A,b,cones,settings=assemble(job)
   def sparse(x):return {'shape':list(x.shape),'indptr':x.indptr.tolist(),'indices':x.indices.tolist(),'data':x.data.tolist()}
   rebuilt={'P':sparse(P),'q':q.tolist(),'A':sparse(A),'b':b.tolist(),'cones':[str(v) for v in cones]};assert rebuilt==matrix;assert settings.max_iter==200 and settings.tol_gap_abs==settings.tol_gap_rel==settings.tol_feas==1e-10
   result={'passed':True,'cycle':n['cycle'],'actor':c,'model_id':job['model_id'],'raw_id':row['raw_object'],'matrix_id':info['matrix_id'],'request_sha':raw['request_sha'],'solver_variables':65,'physical_inputs':64,'dimensions_x_s_z_passed':True,'byte_decode_and_recompiled_matrices_equal':True,'new_optimizer_calls':0};write(path,result);checked.append(result)
 result={'passed':True,'completed_cycles_checked':len(ns),'native_raw_checked':len(checked),'checks':checked,'shared_compiler':'same frozen local_mpc.assemble, independent byte/matrix/raw traversal; no solver construction or native solve','new_optimizer_calls':0,'wall_seconds':time.monotonic()-beg};write(HERE/'results/matrix_stream_certificate.json',result);print(json.dumps({k:v for k,v in result.items() if k!='checks'}),flush=True)
if __name__=='__main__':run()
