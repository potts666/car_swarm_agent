"""Execute every rule-regression zero prefix, its K and the retained old suffix."""
from common import *
from local_update import certify_prefix
from receivers import replacement_events


def main():
    records=[]
    for case in read(HERE/'results/rule_checks.json')['cases']:
        generated=get(HERE/'objects',case['generation_id']);assert generated['passed'];anchor=get(HERE/'objects',generated['anchor_id']);source=restore(m,anchor['state']);old=get(HERE/'objects',source['witness']);candidate=copy.deepcopy(old);candidate['events']=replacement_events(old['events'],anchor['actor'],anchor['receiver']);proof=certify_prefix(source,candidate,anchor);pid=put(HERE/'objects',proof)
        records.append(dict(kind=case['kind'],case=case.get('case'),side=case.get('side'),generation_id=case['generation_id'],proof_id=pid,passed=True,source_phase=source['cycle'],K_phase=anchor['receiver_phase'],retained_phase=proof['retained_target']['cycle']))
    write(HERE/'results/zero_successor_checks.json',dict(passed=True,new_solver_calls=0,records=records));print(json.dumps(records))
if __name__=='__main__':main()
