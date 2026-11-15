"""A vehicle reaches its goal inside N; receiver and K extend into legal hold."""
from common import *
from execution import advance
from contracts import build,bind
from generation import generate
from receivers import feedback,check_generated

def main():
    assert read(HERE/'solver_calls.json')==[];r=read(HERE/'pre_registration.json');s=restore(m,r['state']);s['contract_id']=None;w=get(HERE/'objects',s['witness']);actor=min(CARS,key=lambda c:w['goal_times'][c]);target=w['goal_times'][actor];source=target-16
    while s['cycle']<source:advance(s,s['witness'])
    bind(s,build(s));g,gid=generate(s,actor);assert g['passed'],g;cert=get(HERE/'objects',g['receiver']['certificate_id']);trial=copy.deepcopy(s)
    while trial['cycle']<source+32:advance(trial,trial['witness'])
    check_generated(trial,actor,g['receiver']['domain']);assert feedback(trial,actor,cert)==(F(0),F(0)) and trial['v'][actor]==trial['delta'][actor]==0
    advance(trial,trial['witness']);write(HERE/'results/terminal_hold.json',dict(passed=True,source=state_record(s),goal=target,receiver=source+32,actor=actor,generation_id=gid,zero_K=True,new_calls=0));print(dict(passed=True,goal=target,receiver=source+32),flush=True)
if __name__=='__main__':main()
