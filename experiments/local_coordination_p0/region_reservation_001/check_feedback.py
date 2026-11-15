"""Nonzero boundary-domain ZOH feedback examples; separate from full-task metrics."""
from common import *
from physics import step
from feedback_controls import waiting,entering
from certify_domains import check_domain


def source(car,xy,theta,v,delta,owner):
    s=initial_state();s['poses']={c:(m.I(WAIT[c][0]),m.I(WAIT[c][1]),m.I(0)) for c in CARS};s['poses'][car]=(m.I(xy[0]),m.I(xy[1]),m.I(theta));s['v'][car]=F(v);s['delta'][car]=F(delta);s['owner']=owner;s['history']=content_id(dict(fixture=car,xy=xy,theta=theta,v=v,delta=delta));return s

def replay(s,car,commands,owner):
    road=pair=F(100);records=[]
    for u in commands:
        joint={c:(F(0),F(0)) for c in CARS};joint[car]=tuple(map(F,u));r=step(s,joint,owner,8);road=min(road,r['bounds']['road']);pair=min(pair,r['bounds']['pair']);records.append(r)
    return dict(actual_steps=len(records),minimum_road=str(road),minimum_pair=str(pair),source_input_commands=[list(map(str,u)) for u in commands],final=state_record(s))

def main():
    dom=read(HERE/'domain_checks.json')['domains'];routes=read(HERE/'rational_inputs.json')['routes'];results=[]
    for car,sgn in (('A1',1),('B1',-1)):
        s=source(car,(F(WAIT[car][0])+F(sgn,100),WAIT[car][1]),F(sgn,10000),F(1,50),F(sgn,100),None);check_domain(s,car,'W_in',dom);u=waiting(s['v'][car],s['delta'][car]);r=replay(s,car,u,None);r.update(case='W_in_to_W_hold',car=car,receiver=check_domain(s,car,'W_hold',dom));assert s['v'][car]==s['delta'][car]==0;results.append(r)
        d=direction(car);s=source(car,(-22*d+F(d,50),F(0)),F(sgn,5000),F(51,100) if sgn>0 else F(49,100),F(sgn,1000),car);check_domain(s,car,'G_in',dom);route=get(HERE/'objects',routes[car]);u=[tuple(map(F,x)) for x in route['controls'][101:663]];u[0]=entering(s['v'][car],s['delta'][car]);r=replay(s,car,u,car);r.update(case='G_in_to_G_out',car=car,receiver=check_domain(s,car,'G_out',dom));assert s['v'][car]==s['delta'][car]==0;results.append(r)
        s=source(car,(34*d+F(d*35,1000),F(0)),F(sgn*24,100000),0,0,None);check_domain(s,car,'G_out',dom);u=[tuple(map(F,x)) for x in route['controls'][663:]];r=replay(s,car,u,None);r.update(case='G_out_to_target',car=car,receiver=check_domain(s,car,'target',dom));results.append(r)
    write(HERE/'feedback_checks.json',dict(passed=True,cases=results,scope='auxiliary nonzero boundary-domain feedback executions, not counted as new full-task trials or solver calls'));print('Six nonzero boundary-domain ZOH feedback/continuation examples passed')
if __name__=='__main__':main()
