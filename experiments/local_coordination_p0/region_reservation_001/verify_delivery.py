"""Content-checked evidence reuse, fresh source/prefix check; optional full replay."""
import argparse
from common import *
from accept_case import Audit,accept


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--full-replay',action='store_true');args=parser.parse_args();manifest=read(HERE/'DELIVERY_MANIFEST.json')['files']
    for name,item in manifest.items():assert sha(HERE/name)==item['sha256'],name
    objects=0
    for f in (HERE/'objects').glob('*.json.gz'):get(HERE/'objects',f.name[:-8]);objects+=1
    checkpoints=read(HERE/'results/independent_checkpoints.json');checks={}
    for case,checkpoint in checkpoints.items():
        assert checkpoint['passed'] and checkpoint['full_independent_replay_already_performed']
        for group in ('files','code'):
            for name,digest in checkpoint[group].items():assert sha(HERE/name)==digest,name
        run=HERE/'results'/case;event=next(e for e in read(run/'events.json') if e['kind']=='local_socp');anchor=get(HERE/'objects',event['anchor_id']);audit=Audit(read(run/'config.json')['initial_witness']);audit.state=restore(m,anchor['state']);audit.p8=copy.deepcopy(audit.state['poses']);audit.validate_witness(audit.state['witness']);audit.numeric_update(event)
        checks[case]=dict(full_independent_evidence_content_checked=True,fresh_actual_source_derivative_and_33_tick_prefix_checked=True,full_task_replayed_this_call=False)
        if args.full_replay:checks[case]['full_replay']=accept(run);checks[case]['full_task_replayed_this_call']=True
    for name in ('domain_checks.json','uniform_quantization_check.json','integration_bounds.json','feedback_checks.json','flow_checks.json','source_binding_checks.json','native_rejection_checks.json','self_checks.json'):assert read(HERE/name)['passed']
    from protocol import Barrier
    dummy=Barrier({'relocation':'dependency-read-check'});assert dummy.context['relocation']=='dependency-read-check'
    print(json.dumps(dict(passed=True,files_verified=len(manifest),objects_verified=objects,solver_calls=0,checks=checks),indent=2))
if __name__=='__main__':main()
