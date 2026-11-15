"""One byte/JSON/assembly path for zero-call checks and native worker release."""
import multiprocessing as mp,time,traceback
from common import *
from local_mpc import construct,native

def worker(conn):
 try:
  raw=conn.recv_bytes();request=json.loads(raw);job=request['job'];mode=request['mode']
  assert mode in ('assemble','solve')
  assert job['model_id']==content_id({k:v for k,v in job.items() if k!='model_id'})
  if mode=='assemble':
   solver,info=construct(job);result={'status':'assembly_passed','assembly':info,'solve_called':False}
  else:
   def notify(info):
    conn.send_bytes(json.dumps({'event':'native_ready','assembly':info}).encode());release=json.loads(conn.recv_bytes());assert release=={'event':'release_solve','model_id':job['model_id']}
   result=native(job,notify)
  result.update(model_id=job['model_id'],request_sha=hashlib.sha256(raw).hexdigest(),received_bytes=len(raw));conn.send_bytes(json.dumps({'event':'result','result':result}).encode())
 except Exception as e:conn.send_bytes(json.dumps({'event':'result','result':{'status':'worker_error','error':repr(e),'traceback':traceback.format_exc()}}).encode())
 finally:conn.close()

def exchange(job,mode,stem,on_ready=None):
 assert mode=='assemble' or on_ready is not None
 ctx=mp.get_context('spawn');parent,child=ctx.Pipe();process=ctx.Process(target=worker,args=(child,));start=time.time();process.start();child.close()
 payload=json.dumps({'mode':mode,'job':job},sort_keys=True,separators=(',',':')).encode();Path(str(stem)+'_request.json').write_bytes(payload);parent.send_bytes(payload);ready=None;response=None
 try:
  while True:
   if not parent.poll(30):raise TimeoutError('worker channel timeout')
   response=parent.recv_bytes();Path(str(stem)+'_last_frame.json').write_bytes(response);frame=json.loads(response)
   if frame['event']=='native_ready':
    assert mode=='solve' and ready is None and frame['assembly']['model_id']==job['model_id'];ready=frame;write(str(stem)+'_ready.json',ready);on_ready(frame['assembly']);parent.send_bytes(json.dumps({'event':'release_solve','model_id':job['model_id']}).encode())
   else:
    assert frame['event']=='result';result=frame['result'];break
 finally:
  process.join(5)
  if process.is_alive():process.terminate();process.join(5)
  parent.close()
 assert process.exitcode==0
 if result['status']!='worker_error':assert result['request_sha']==hashlib.sha256(payload).hexdigest() and result['received_bytes']==len(payload) and result['model_id']==job['model_id']
 if result['status']=='worker_error':
  write(str(stem)+'_response.json',result);raise RuntimeError(('worker_error',str(stem),result))
 if mode=='assemble':assert result['status']=='assembly_passed' and not result['solve_called']
 else:assert ready is not None and result['assembly']==ready['assembly']
 meta={'mode':mode,'model_id':job['model_id'],'request_sha':hashlib.sha256(payload).hexdigest(),'sent_bytes':len(payload),'received_bytes':len(response),'worker_exitcode':process.exitcode,'roundtrip_wall':time.time()-start}
 write(str(stem)+'_response.json',result);write(str(stem)+'_transport.json',meta)
 return result,meta
