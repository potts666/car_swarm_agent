"""Content-check full replay checkpoints; freshly verify actual numeric sources, no solve."""
import argparse
from common import *
from accept_case import Audit,accept

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--full-replay',action='store_true');args=parser.parse_args();manifest=read(HERE/'DELIVERY_MANIFEST.json')['files']
    for name,item in manifest.items():assert sha(HERE/name)==item['sha256'],name
    for f in (HERE/'objects').glob('*.json.gz'):get(HERE/'objects',f.name[:-8])
    checkpoint=read(HERE/'results/independent_checkpoints.json');assert checkpoint['full_replay_completed']
    for section in ('code','files'):
        for name,digest in checkpoint[section].items():assert sha(HERE/name)==digest,name
    outcomes={}
    for side in ('A','B'):
        folder=HERE/'results'/side;event=next(e for e in read(folder/'events.json') if e['kind']=='local_socp');anchor=get(HERE/'objects',event['anchor_id']);audit=Audit(read(folder/'config.json')['initial_witness']);audit.state=restore(m,anchor['state']);audit.p8=copy.deepcopy(audit.state['poses']);audit.validate_witness(audit.state['witness'])
        with gzip.open(folder/'execution.jsonl.gz','rt') as stream:row=next(r for r in map(json.loads,stream) if r['cycle']==event['cycle'])
        audit.numeric_update(event);audit.frame(row);rej=read(folder/'rejection/summary.json');assert rej['passed'] and len(rej['rejections'])==8 and rej['actual_steps']==99 and rej['invalid_commits']==0
        outcomes[side]=dict(passed=True,fresh_actual_source_and_prefix_checks=1,full_task_and_rejection_evidence_content_checked=True,full_task_replayed_this_call=False)
        if args.full_replay:outcomes[side]['full_replay']=accept(folder);outcomes[side]['full_task_replayed_this_call']=True
    assert read(HERE/'GATE.json')['passed'] and len(read(HERE/'results/solver_calls.json'))==2
    print(json.dumps(dict(passed=True,files_verified=len(manifest),new_solver_calls=0,cases=outcomes),indent=2))
if __name__=='__main__':main()
