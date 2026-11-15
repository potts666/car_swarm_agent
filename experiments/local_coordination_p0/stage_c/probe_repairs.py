#!/usr/bin/env python3
"""At most two registered representations, three scales each; no closed-loop run."""
import argparse
from c_common import *
from c_socp import solve


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if args.output.exists():p.error('preserve probes')
    args.output.mkdir(parents=True);data,m=setup();index=read(BASE/'results/enrichment_001/index.json');plan=read(HERE/'repair_plan.json');attempts=[];chosen=None
    for variant in plan['variants']:
        for config in plan['configs']:
            selection=dict(variant=variant['name'],**config);write(HERE/'selected_repair.json',selection);passed=True
            for name,item in index.items():
                anchor=get(HERE/'objects',item['anchor_id']);cert=get(HERE/'objects',item['derivative_id']);result=solve(data,m,anchor,cert,'B-JL');result['derivative_id']=item['derivative_id'];key=put(HERE/'objects',result)
                attempts.append(dict(interface=name,selection=selection,result_id=key,status=result['status'],accepted=result['accepted'],changed=result.get('changed',False)));passed &= result['accepted'] and result.get('changed',False)
                write(args.output/'attempts.json',attempts);print(name,selection,result['status'],result['accepted'],flush=True)
            if passed:chosen=selection;break
        if chosen:break
    if chosen:write(HERE/'selected_repair.json',chosen)
    write(args.output/'summary.json',dict(passed=chosen is not None,selected=chosen,attempts=attempts,variants_limit=2,configs_per_variant_limit=3));return 0


if __name__=='__main__':main()
