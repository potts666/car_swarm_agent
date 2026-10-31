#!/usr/bin/env python3
"""Re-read M0 logs; replay scalar dynamics independently of the execution integrator."""
import argparse
import json
from pathlib import Path
import numpy as np
from bicycle_core import CARS, audit, completed
from verify_entry_bicycle import load_rows, vectors, replay, verify_policy, read_jsonl
from run_entry_bicycle import write


def verify(directory, scene):
    original=verify_policy(directory,scene)
    rows=load_rows(directory/'actual.csv')
    actual={c:vectors(rows[c]) for c in CARS};replayed={};errors={}
    for c in CARS:
        commands=[[float(r['a']),float(r['omega'])] for r in rows[c][1:]]
        replayed[c]=replay(np.array(scene['initial'][c],float),commands,scene['sample_s'],scene['vehicle'])
        errors[c]=dict(zip(('x_m','y_m','heading_rad','speed_m_s','steering_rad'),map(float,np.max(abs(replayed[c]-actual[c]),axis=0))))
    replay_check=audit(replayed,scene)
    events=read_jsonl(directory/'events.jsonl');ownership=[e for e in events if e['kind'] in ('grant','release')]
    owner_consistent=True
    for c in CARS:
        index=0;owner=None
        for row in rows[c][1:]:
            # A row's control/owner describes the interval ending at its timestamp.
            while index<len(ownership) and ownership[index]['time_s']<float(row['time_s'])-1e-9:
                e=ownership[index];owner=e['car'] if e['kind']=='grant' else None;index+=1
            owner_consistent &= row['owner']==(owner or '')
    planning=json.loads((directory/'planning.json').read_text());solves=read_jsonl(directory/'solves.jsonl')
    summary=json.loads((directory/'summary.json').read_text())
    committed=sum(p['committed'] for p in planning)
    counts=all(len(rows[c])==1+committed*round(scene['h']/scene['sample_s']) for c in CARS)
    timed={p['cycle']:p for p in planning}
    timing_complete=len(timed)==len(planning) and all(s['cycle'] in timed for s in solves)
    timing_complete &= all(p['end_monotonic_s']>=p['start_monotonic_s'] and abs(p['total_cycle_wall_s']-(p['end_monotonic_s']-p['start_monotonic_s']))<1e-9 for p in planning)
    if summary['failure']:
        timing_complete &= bool(planning and not planning[-1]['committed'] and planning[-1]['outcome'] in ('rejected','exception')) or summary['failure']=='timeout'
    equation_ok=all(v<=scene['tolerance'] for e in errors.values() for v in e.values())
    prefix=bool(original['passed'] and replay_check['passed'] and equation_ok and owner_consistent and counts and timing_complete)
    goals={c:completed(replayed[c][-1],scene['goals'][c],scene) for c in CARS}
    result=dict(passed=bool(prefix and summary['both_completed'] and all(goals.values())),executed_prefix_passed=prefix,
        original_log_validation=original,accumulated_replay_errors=errors,accumulated_replay_physics=replay_check,
        independently_replayed_goals=goals,event_owner_matches_applied_intervals=bool(owner_consistent),
        committed_sample_counts_consistent=bool(counts),all_cycle_exits_timed=bool(timing_complete),
        scope='independent scalar RK4 with half steps, accumulated and interval replay; shared geometry audit; sampled 0.02 s only')
    write(directory/'independent_validation.json',result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True);args=parser.parse_args()
    scene=json.loads((args.run/'config.json').read_text())['scene']
    result=verify(args.run,scene)
    print(json.dumps({k:result[k] for k in ('passed','executed_prefix_passed','all_cycle_exits_timed')}))
