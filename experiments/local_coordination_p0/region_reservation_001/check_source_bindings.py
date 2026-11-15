"""Check strengthened proposal-source conditions against every saved actual commit."""
from common import *
from protocol import validate_request,prepare,commit
from run_case import advance


def main():
    result={}
    for case in ('opposite','same_direction'):
        run=HERE/'results'/case;count=changes=0
        with gzip.open(run/'execution.jsonl.gz','rt') as stream:
            for row in map(json.loads,stream):
                source=restore(m,row['token']['context']['source_snapshot']);validate_request(source,row['token']['context']);count+=1;changes+=source['witness']!=row['witness_id']
        event=next(e for e in read(run/'events.json') if e['kind']=='local_socp');anchor=get(HERE/'objects',event['anchor_id']);source=restore(m,anchor['state']);old=source['witness'];advance(source,old)
        try:request=prepare(source,event['accepted_witness'],content_id(dict(witness=old,cycle=source['cycle'],history=source['history'],remaining=2802-source['cycle'])));commit(source,request)
        except (ValueError,AssertionError) as error:reason=str(error)
        else:raise AssertionError('stale numerical proposal adopted')
        result[case]=dict(passed=True,actual_commits_checked=count,adopted_policy_changes=changes,stale_next_cycle_proposal_rejected=True,rejection=reason)
    write(HERE/'source_binding_checks.json',dict(passed=True,cases=result));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
