"""Check saved full acceptance and freshly replay the two actual optimization sources."""
import argparse
from common import *
from accept_case import Audit,accept


def main():
    p=argparse.ArgumentParser();p.add_argument('--full-replay',action='store_true');args=p.parse_args();manifest=read(HERE/'DELIVERY_MANIFEST.json')['files']
    for name,item in manifest.items():assert sha(HERE/name)==item['sha256'],name
    for f in (HERE/'objects').glob('*.json.gz'):get(HERE/'objects',f.name[:-8])
    outcomes={}
    for case,checkpoint in read(HERE/'results/independent_checkpoints.json').items():
        assert checkpoint['passed'] and checkpoint['full_replay_completed']
        for section in ('files','code'):
            for name,digest in checkpoint[section].items():assert sha(HERE/name)==digest,name
        folder=HERE/'results'/case;events=[e for e in read(folder/'events.json') if e['kind']=='local_socp'];first_anchor=get(HERE/'objects',events[0]['anchor_id']);audit=Audit(read(folder/'config.json')['initial_witness']);audit.state=restore(m,first_anchor['state']);audit.p8=copy.deepcopy(audit.state['poses']);audit.validate_witness(audit.state['witness'])
        with gzip.open(folder/'execution.jsonl.gz','rt') as stream:rows={r['cycle']:r for r in map(json.loads,stream) if r['cycle'] in (787,788)}
        for event in events:audit.numeric_update(event);audit.frame(rows[event['cycle']])
        outcomes[case]=dict(passed=True,fresh_actual_source_and_prefix_checks=2,full_task_evidence_content_checked=True,full_task_replayed_this_call=False)
        if args.full_replay:outcomes[case]['full_replay']=accept(folder);outcomes[case]['full_task_replayed_this_call']=True
    assert read(HERE/'self_checks.json')['passed'] and read(HERE/'second_round_rejection_checks.json')['passed']
    print(json.dumps(dict(passed=True,files_verified=len(manifest),solver_calls=0,cases=outcomes),indent=2))
if __name__=='__main__':main()
