#!/usr/bin/env python3
"""Exact analytic time-priority calibration with a common executable brake suffix."""
import argparse
from fractions import Fraction as F
from pathlib import Path
import json

H=F(1,5)
def write(p,x):p.write_text(json.dumps(x,default=str,indent=2)+'\n')


def geometry(left,right):
    # All headings/steering are zero; v>=0 makes the exact sweep an x interval.
    centers=[(min(left,right),max(left,right),F(0)),(F(20),F(20),F(5)),(F(-20),F(-20),F(5)),(F(20),F(20),F(-5))]
    bodies=[(lo-1,hi+F(7,2),y-F(9,10),y+F(9,10)) for lo,hi,y in centers]
    road=min(min(b[0]+50,50-b[1],b[2]+15,15-b[3]) for b in bodies)
    pairs={}
    for i in range(4):
        for j in range(i+1,4):
            a,b=bodies[i],bodies[j];pairs[f'{i}-{j}']=max(b[0]-a[1],a[0]-b[1],b[2]-a[3],a[2]-b[3])
    assert road>=0 and min(pairs.values())>=F(1,2)
    return dict(continuous_road_lower=road,all_six_pair_lower=pairs,proof='exact monotone x sweep; fixed rectangular bodies and zero yaw')


def execute(prefix,suffix):
    x=F(0);v=F(1,2);budget=len(prefix)+len(suffix);rows=[];energy=F(0)
    for k,a in enumerate(prefix+suffix):
        nv=v+H*a;nx=x+H*v+H*H*a/2;assert max(abs(a),0)<=1 and 0<=v<=2 and 0<=nv<=2
        check=geometry(x,nx);energy+=H*a*a
        rows.append(dict(cycle=k,x0=x,v0=v,a=a,omega=0,x1=nx,v1=nv,y=0,psi=0,delta=0,budget_before=budget,budget_after=budget-1,geometry=check));budget-=1;x,v=nx,nv
    assert v==0 and budget==0
    return dict(steps=len(rows),prefix_steps=len(prefix),suffix_steps=len(suffix),initial_state=[0,0,0,'1/2',0],final_state=[x,0,0,v,0],energy=energy,rows=rows)


def independent(value):
    x=F(0);v=F(1,2);E=F(0);count=0;endpoint=None
    for row in value['rows']:
        a=F(row['a']);next_v=v+F(1,5)*a;next_x=x+F(1,5)*v+F(1,50)*a
        assert row['cycle']==count and F(row['x0'])==x and F(row['v0'])==v
        assert F(row['x1'])==next_x and F(row['v1'])==next_v and row['omega']==row['y']==row['psi']==row['delta']==0
        assert 0<=v<=2 and 0<=next_v<=2 and abs(a)<=1
        assert row['budget_before']==value['steps']-count and row['budget_after']==value['steps']-count-1
        geo=geometry(x,next_x);assert len(geo['all_six_pair_lower'])==6
        E+=F(1,5)*a*a;x,v=next_x,next_v;count+=1
        if count==value['prefix_steps']:endpoint=[x,F(0),F(0),v,F(0)]
    assert count==value['steps'] and [x,0,0,v,0]==[F(q) for q in value['final_state']] and E==F(value['energy'])
    assert endpoint==[F(2),0,0,F(1,2),0] and v==0
    return dict(passed=True,actual_steps=count,prefix_endpoint=endpoint,final_goal=value['final_state'],energy=E)


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():p.error('preserve analytic records')
    a.output.mkdir(parents=True);old=[F(0)]*20;new=[F(3,4)]*5+[F(0)]+[F(-3,4)]*5;suffix=[F(-5,6)]*3
    values={}
    for name,prefix in [('old_20',old),('new_11',new)]:
        value=execute(prefix,suffix);write(a.output/(name+'.json'),value);saved=json.loads((a.output/(name+'.json')).read_text());values[name]=independent(saved)
    assert H*sum(x*x for x in new)==F(9,8) and H*sum(x*x for x in old)==0
    assert max(F(r['v1']) for r in json.loads((a.output/'new_11.json').read_text())['rows'][:11])==F(5,4)
    assert len(old)-len(new)==9 and H*(len(old)-len(new))==F(9,5)
    assert (len(new)+3,H*sum(x*x for x in new))<(len(old)+3,F(0))
    write(a.output/'independent_acceptance.json',dict(passed=True,cases=values,continuous_sweep='exact analytic, all six pairs and road',same_executable_suffix=True))
    write(a.output/'summary.json',dict(passed=True,old_prefix_ticks=20,new_prefix_ticks=11,old_prefix_seconds='4',new_prefix_seconds='11/5',saved_ticks=9,saved_seconds='9/5',max_speed='5/4',old_prefix_energy='0',new_prefix_energy='9/8',suffix_ticks=3,old_completion_budget=23,new_completion_budget=14,initial_budget_before_acceptance=23,budget_after_acceptance=14,budget_after_new_first_step=13,goal_x='43/20',lexicographic_time_priority=True,map_experiment=False))
    print('C1 passed: exact same five-state endpoint, 20 -> 11 ticks; higher energy accepted for shorter time; independent replay passed')


if __name__=='__main__':main()
