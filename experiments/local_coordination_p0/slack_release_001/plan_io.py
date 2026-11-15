import gzip
from common import *
from physics import step

def load(n):return json.loads(gzip.decompress((HERE/n).read_bytes()))
def save(n,v):
 p=HERE/n;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(gzip.compress(json.dumps(v,sort_keys=True,default=str).encode(),mtime=0))
def record(s):return {'poses_Q':{c:pose_rows(s['poses'][c]) for c in CARS},'v':s['v'].copy(),'delta':s['delta'].copy(),'history':s['history'],'cycle':s['cycle']}
def restore(r):return {'poses':{c:tuple(m.I.raw(*q) for q in r['poses_Q'][c]) for c in CARS},'v':{c:F(q) for c,q in r['v'].items()},'delta':{c:F(q) for c,q in r['delta'].items()},'history':r['history'],'cycle':r['cycle']}
def source(plan,k):
 s=initial_state()
 for t in range(k):step(s,{c:plan['controls'][c][t] for c in CARS},plan['owners'][t],4)
 return s
