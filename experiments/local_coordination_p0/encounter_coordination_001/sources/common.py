"""Region-local identities and model; historical single-mover controllers stay frozen."""
import copy,gzip,hashlib,importlib.util,json,math,sys
from fractions import Fraction as F
from pathlib import Path
from types import SimpleNamespace
HERE=Path(__file__).resolve().parent
CARS=('A1','A2','B1','B2');H=F(1,5);RHO=F(1,1000);RB=F(1807,500);SAFE=F(1,2);EXTRA=F(1,10**8);TUBE=F(1,20)+EXTRA
MODEL_PATH=HERE/'inputs/interval_source.py'
spec=importlib.util.spec_from_file_location('region_interval',MODEL_PATH);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
INITIAL={'A1':(-35,3),'A2':(-39,-3),'B1':(37,3),'B2':(40,-3)}
WAIT={'A1':(-30,3),'A2':(-30,-3),'B1':(30,3),'B2':(30,-3)}
ROADS=((-52,52,F(-7,5),F(7,5)),(-52,-10,F(-11,2),F(11,2)),(10,52,F(-11,2),F(11,2)))

def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def content_id(v):return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),default=str).encode()).hexdigest()
def normal(v):return json.loads(json.dumps(v,default=str))
def write(p,v):Path(p).parent.mkdir(parents=True,exist_ok=True);Path(p).write_text(json.dumps(v,default=str,ensure_ascii=False,indent=2)+'\n')
def put(root,v):
    root=Path(root);root.mkdir(parents=True,exist_ok=True);key=content_id(v);f=root/(key+'.json.gz');raw=json.dumps(v,ensure_ascii=False,sort_keys=True,separators=(',',':'),default=str).encode()
    if f.exists():assert gzip.decompress(f.read_bytes())==raw
    else:f.write_bytes(gzip.compress(raw,mtime=0))
    return key
_OBJECT_CACHE={}
def get(root,key):
    f=Path(root)/(key+'.json.gz')
    if str(f) not in _OBJECT_CACHE:
        value=json.loads(gzip.decompress(f.read_bytes()));assert content_id(value)==key;_OBJECT_CACHE[str(f)]=value
    return _OBJECT_CACHE[str(f)]  # Objects are immutable; callers copy before proposing replacements.
def norm_upper(v,grid=10**24):
    z=sum(F(x)**2 for x in v);n=math.isqrt(z.numerator*grid*grid//z.denominator)
    if F(n,grid)**2<z:n+=1
    return F(n,grid)
def pose_rows(p):return [[q.l,q.u] for q in p]
MODEL_FRAMES={'A1':1,'A2':1,'B1':-1,'B2':-1}
def direction(c):return MODEL_FRAMES[c]

def initial_state():
    return dict(poses={c:(m.I(INITIAL[c][0]),m.I(INITIAL[c][1]),m.I(0)) for c in CARS},v={c:F(0) for c in CARS},delta={c:F(0) for c in CARS},cycle=0,history=content_id(dict(initial=INITIAL,model=sha(MODEL_PATH),h=H)),witness=None,owner=None,permission_version=0,served=[])
def state_record(state):return dict(poses_Q={c:pose_rows(state['poses'][c]) for c in CARS},v={c:str(x) for c,x in state['v'].items()},delta={c:str(x) for c,x in state['delta'].items()},history=state['history'],cycle=state['cycle'],witness=state['witness'],owner=state.get('owner'),permission_version=state.get('permission_version',0),served=state.get('served',[]))
def restore(model,record):return dict(poses={c:tuple(model.I.raw(*p) for p in record['poses_Q'][c]) for c in CARS},v={c:F(x) for c,x in record['v'].items()},delta={c:F(x) for c,x in record['delta'].items()},history=record['history'],cycle=record['cycle'],witness=record['witness'],owner=record.get('owner'),permission_version=record.get('permission_version',0),served=record.get('served',[]).copy())

def execute_one(model,state,actor,command,cfg,released):
    from physics import step
    witness=get(HERE/'objects',state['witness']);k=state['cycle'];commands={c:tuple(map(F,witness['controls'][c][k])) for c in CARS};commands[actor]=command
    return step(state,commands,witness['owners'][k],4)
legacy=SimpleNamespace(execute=execute_one)
