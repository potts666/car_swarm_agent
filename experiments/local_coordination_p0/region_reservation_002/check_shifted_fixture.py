"""Auxiliary nonzero shifted-domain K and fixed suffix, before any new solve."""
from common import *
from physics import step
from certify_domains import check_domain


def main():
    outputs=[];domains=read(HERE/'domain_checks.json')['domains'];r=read(HERE/'rational_inputs.json')
    for car in ('A1','B1'):
        sign=direction(car);state=initial_state();state['poses']={c:(m.I(WAIT[c][0]),m.I(WAIT[c][1]),m.I(0)) for c in CARS};state['poses'][car]=(m.I(sign*(F(-219,10)+F(21,1000))),m.I(0),m.I(sign*F(119,500000)));state['v'][car]=F(101,200);state['delta'][car]=sign*F(1,100000);check_domain(state,car,'G_in_shifted',domains)
        route=get(HERE/'objects',r['routes'][car]);u=[tuple(map(F,x)) for x in route['controls'][102:663]];u[0]=((F(1,2)-state['v'][car])/H,-state['delta'][car]/H)
        assert len(u)==561
        for j,command in enumerate(u):
            joint={c:(F(0),F(0)) for c in CARS};joint[car]=command;step(state,joint,car,8)
            if j==0:assert state['v'][car]==F(1,2) and state['delta'][car]==0
        outputs.append(dict(car=car,passed=True,steps=561,receiver=check_domain(state,car,'G_out',domains),final=state_record(state)))
    write(HERE/'shifted_fixture_checks.json',dict(passed=True,cases=outputs));print('Shifted nonzero-domain K and 561-step suffix passed for A/B directions')
if __name__=='__main__':main()
