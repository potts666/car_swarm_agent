"""Retimed single-car canonical phases, unchanged physical connection family."""
from functools import lru_cache
from common import *
from plan_io import load
BASE=None;STATES=None;REFERENCES={}
def base():
 global BASE
 if BASE is None:BASE=load('results/R_initial.json.gz')
 return BASE
def states():
 global STATES
 if STATES is None:STATES=load('results/canonical_states.json.gz')
 return STATES
def index(c,t,start):
 p=base()['per_car_policies'][c];prep=p['approach_end']
 if t<prep:return t
 return min(p['goal'],p['service_start']+max(0,t-start))
def point(c,t,start):
 r=states()[index(c,t,start)];return tuple(m.I.raw(*q) for q in r['poses_Q'][c]),F(r['v'][c]),F(r['delta'][c])
def reference(plan):
 key=(plan['B0'],tuple(plan['per_car_policies'][c]['service_start'] for c in CARS))
 if key not in REFERENCES:
  rows=[]
  for t in range(plan['B0']+1):
   ps={};v={};d={}
   for c in CARS:p,vv,dd=point(c,t,plan['per_car_policies'][c]['service_start']);ps[c]=pose_rows(p);v[c]=vv;d[c]=dd
   rows.append({'cycle':t,'poses_Q':ps,'v':v,'delta':d})
  REFERENCES[key]=rows
 return REFERENCES[key]
def exit_phase(plan,c):
 p=plan['per_car_policies'][c];return p['service_start']+plan['paths'][c]['ranges']['crossing'][1]-p['approach_end']
def pose_error(actual,nominal):return sum((actual[j]-nominal[j]).abs_upper() for j in (0,1))+RB*(actual[2]-nominal[2]).abs_upper()
