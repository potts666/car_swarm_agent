"""Frozen physical-only interface; never opens historical witness objects."""
import hashlib,importlib.util,json,math,sys
# Exact E/T proof rationals are locally generated and can exceed4300 decimal digits.
sys.set_int_max_str_digits(0)
from fractions import Fraction as F
from pathlib import Path
HERE=Path(__file__).resolve().parent
CARS=('A1','A2','B1','B2');H=F(1,5);RB=F(1807,500);SAFE=F(1,2);EXTRA=F(1,10**8);TUBE=F(1,20)+EXTRA
spec=importlib.util.spec_from_file_location('aut_interval',HERE/'sources/interval_source.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def content_id(v):return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),default=str).encode()).hexdigest()
def write(p,v):Path(p).parent.mkdir(parents=True,exist_ok=True);Path(p).write_text(json.dumps(v,indent=2,default=str)+'\n')
def pose_rows(p):return [[q.l,q.u] for q in p]
CFG=read(HERE/'vehicle_configs.json');TASKS=read(HERE/'local_tasks.json');ROADS=tuple(tuple(map(F,r)) for r in read(HERE/'map_config.json')['roads'])
def direction(c):return CFG['frames'][c]
def initial_state():return dict(poses={c:tuple(m.I(F(q)) for q in TASKS[c]['initial'][:3]) for c in CARS},v={c:F(0) for c in CARS},delta={c:F(0) for c in CARS},cycle=0,history=content_id(TASKS))
