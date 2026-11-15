"""Frozen local dependencies and content-addressed B objects."""
import copy
from fractions import Fraction as F
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
P0=HERE/'dependencies/repo/experiments/local_coordination_p0'
sys.path.insert(0,str(P0/'stage_a'))
from region_certificates import Certificates,sha,write,read,content_id,CARS,H,pose_rows,reset_input,in_box,in_flow
import region_control as A
import pilot_control as legacy
from accept_interfaces import Replay
REBUILD=P0/'results/source_rebuild_005'
RHO=F(1,1000)


def setup():return Certificates(HERE/'inputs'),legacy.load_model(REBUILD)


def state_record(state):
    return dict(poses_Q={c:pose_rows(state['poses'][c]) for c in CARS},v={c:str(x) for c,x in state['v'].items()},delta={c:str(x) for c,x in state['delta'].items()},history=state['history'])


def restore(model,record):
    return dict(poses={c:tuple(model.I.raw(*p) for p in record['poses_Q'][c]) for c in CARS},v={c:F(x) for c,x in record['v'].items()},delta={c:F(x) for c,x in record['delta'].items()},history=record['history'])


def anchor_for(data,model,state,plan):
    value=dict(state=state_record(state),plan=plan,model_sha256=sha(REBUILD/'work/M2b_首段自由度与递归可行性/route_zoh_certificate.py'),physics_sha256=content_id(data.cfg),actor=data.cert['car'],coordinate_branch='A:psi=theta; B:psi=pi+theta',direction_sign=1 if data.cert['car'][0]=='A' else -1,m=32,rho=str(RHO),phase=plan['phase'],receiver_phase=plan['phase']+32,receiver_row_sha256=content_id(data.row(plan['phase']+32)),witness=data.witness_id,permission_owner=data.cert['car'],joint_source=data.edge['source'],joint_target=data.edge['target'],groups=A.groups(plan['phase'],data.cert['car']))
    return json.loads(json.dumps(value,default=str))


def initial_anchor(data,model,phase):
    state=legacy.initial_state(model,data.cfg);plan=A.initial_plan(data,state)
    for k in range(phase):
        legacy.execute(model,state,data.cert['car'],A.window(plan)[0],data.cfg,data.cert['source']>0)
        plan,_=A.shift(data,plan,state)
    return anchor_for(data,model,state,plan)


def put(root,value):
    key=content_id(value);root=Path(root);root.mkdir(parents=True,exist_ok=True);p=root/(key+'.json.gz')
    raw=json.dumps(value,ensure_ascii=False,default=str,sort_keys=True,separators=(',',':')).encode()
    if p.exists():assert gzip.decompress(p.read_bytes())==raw
    else:p.write_bytes(gzip.compress(raw,mtime=0))
    return key


def get(root,key):
    value=json.loads(gzip.decompress((Path(root)/(key+'.json.gz')).read_bytes()))
    assert content_id(value)==key,'B object content identity'
    return value


def numerical_plan(data,state,old,commands):
    phase=old['phase'];end=phase+32;row=data.row(end);c=data.cert['car'];new=copy.deepcopy(old)
    remaining=[tuple(map(F,x)) for x in old['remaining']];remaining[:32]=[tuple(map(F,x)) for x in commands]
    v=state['v'][c]+H*sum(a for a,w in remaining[:32]);d=state['delta'][c]+H*sum(w for a,w in remaining[:32])
    remaining[32]=reset_input(v,d,row)
    new.update(branch='box_candidate',remaining=remaining,parent=old['id'],receiver={**A.receiver(data.cert,phase),'component':'box','row_sha256':content_id(row),'domain':row['domain']},pending_reset=dict(phase=end,next_phase=end+1,row_sha256=content_id(row),input=remaining[32]),ancestry=old['ancestry']+[dict(parent=old['id'],interface=content_id(row),phase=end)])
    return A.seal(new)


def norm_upper(values,grid=10**24):
    import math
    square=sum(F(x)**2 for x in values);n=math.isqrt(square.numerator*grid*grid//square.denominator)
    if F(n,grid)**2<square:n+=1
    return F(n,grid)
