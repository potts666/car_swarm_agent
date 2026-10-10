#!/usr/bin/env python3
"""Export every rejection; distinguish seed feasibility, solver acceptance and execution."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
BASE = ROOT.parent / 'reverse_parking_mpc'


def read(path):
    return json.loads(path.read_text())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results', type=Path, default=ROOT / 'results')
    args = parser.parse_args()
    output = args.results
    config = read(ROOT / 'config.json')
    summaries = []
    rejections = []
    for seed in config['seeds']:
        source = output / f'two_{seed}.jsonl'
        rows = [json.loads(line) for line in source.read_text().splitlines()]
        summary = read(output / f'two_{seed}_summary.json')
        parent = [json.loads(line) for line in (BASE / f'two_baseline/two_{seed}.jsonl').read_text().splitlines()]
        # Original successful prefix must remain identical if only later retry
        # initializations have changed. Ignore telemetry and wall-clock duration.
        old_prefix = [row for row in parent if row['executed']]
        for before, after in zip(old_prefix, rows):
            for key in ['state', 'controls', 'actual_next_state', 'peer_input']:
                if before[key] != after[key]:
                    raise AssertionError('Original prefix changed: ' + key)
        accepted_extra = []
        for row in rows:
            for attempt in row['solver']['attempts']:
                if not attempt['accepted']:
                    record = dict(seed=seed, cycle=row['cycle'], time_s=row['time_s'], car=row['car'],
                                  state=row['state'], initialization=attempt['initialization'],
                                  optimizer_success=attempt['success'], violation=attempt['violation'],
                                  result_feasible_against_old_peer_prediction=attempt['result_residuals']['feasible_against_old_peer_prediction'],
                                  message=attempt['message'], initial_controls=attempt['initial_controls'],
                                  result_controls=attempt['result_controls'], executed_as_this_attempt=False)
                    rejections.append(record)
                elif attempt['initialization'].startswith(('max_brake', 'gentle_brake')):
                    accepted_extra.append(dict(time_s=row['time_s'], car=row['car'], initialization=attempt['initialization']))
            if row['executed']:
                if not row['solver']['accepted'] or row['solver']['fallback_executed']:
                    raise AssertionError('Rejected or fallback control was executed')
                np.testing.assert_array_equal(row['controls'],row['solver']['attempts'][-1]['result_controls'])
            else:
                snapshot = dict(seed=seed,cycle=row['cycle'],time_s=row['time_s'],car=row['car'],
                                state=row['state'],peer_input=row['peer_input'],warm_input=row['warm_input'],
                                reference_states=row['solver']['reference_states'],
                                reference_controls=row['solver']['reference_controls'],gear=row['solver']['gear'],
                                config=config,solver=row['solver'],executed=False,
                                fallback_executed=False)
                (output / f'rejected_problem_{seed}_car_{row["car"]}.json').write_text(json.dumps(snapshot,indent=2)+'\n')
        with (output / f'two_{seed}_metrics.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['t','car','goal_distance_m','yaw_error_rad','speed_m_s','direction','displacement_m',
                             'solver_accepted','successful_initialization','attempt_count','executed',
                             'min_sampled_pair_body_distance_m','solve_seconds','fallback_executed'])
            for row in rows:
                state = row.get('actual_next_state',row['state'])
                successful = next((a['initialization'] for a in row['solver']['attempts'] if a['accepted']),None)
                writer.writerow([row['time_s'],row['car'],row['goal_error']['position_m'],row['goal_error']['yaw_rad'],
                                 state[3],int(np.sign(state[3])) if abs(state[3])>1e-5 else 0,row.get('displacement_m'),
                                 row['solver']['accepted'],successful,len(row['solver']['attempts']),row['executed'],
                                 row.get('min_sampled_pair_body_distance_m'),row['solver']['solve_seconds'],False])
        summary.update(seed=seed,original_prefix_identical=True,
                       accepted_extra_initializations=accepted_extra,
                       first_unexecuted_time_s=rows[-1]['time_s'],
                       optimization_rejections=sum(not a['accepted'] for row in rows for a in row['solver']['attempts']),
                       legal_braking_candidates_at_stop={str(row['car']):[name for name,value in row['solver']['braking_initializations'].items() if value['feasible_against_old_peer_prediction']] for row in rows[-2:]},
                       fallback_controls_executed=0)
        summaries.append(summary)
    with (output / 'all_optimization_rejections.jsonl').open('w') as stream:
        for row in rejections:
            stream.write(json.dumps(row)+'\n')
    (output / 'comparison.json').write_text(json.dumps(dict(runs=summaries,
        conclusion='Initialization-only retries repair cycle 15 at 6 s but all three runs reject A at cycle 16 / 6.4 s; feasible braking candidates remain, no fallback executed, no sustained stagnation reproduced.'),indent=2)+'\n')
    table='| Seed | Accepted retry at 6 s | Next rejected round | Executed cycles | Rejected attempts | Min actual sampled pair distance (m) | Solve mean/max (s) | Stop |\n'
    table+='| --- | --- | --- | --- | --- | --- | --- | --- |\n'
    for row in summaries:
        accepted='/'.join(a['initialization'] for a in row['accepted_extra_initializations'])
        table+=f"| {row['seed']} | {accepted} | {row['first_unexecuted_time_s']:.1f} s | {row['cycles']} | {row['optimization_rejections']} | {row['min_sampled_pair_body_distance_m']:.6f} | {row['solve_seconds_mean']:.3f}/{row['solve_seconds_max']:.3f} | {row['stop']} |\n"
    (output / 'comparison.md').write_text(table)
    fig, axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
    for label,path in [('parent',BASE/'two_baseline/two_20261010.jsonl'),('brake initializations',output/'two_20261010.jsonl')]:
        rows=[json.loads(line) for line in path.read_text().splitlines()]
        for car in range(2):
            executed=[r for r in rows if r['car']==car and r['executed']]
            t=[r['time_s']+.4 for r in executed]
            axes[0].plot(t,[r['goal_error']['position_m'] for r in executed],label=label+' '+ 'AB'[car])
            axes[1].plot(t,[r['actual_next_state'][3] for r in executed])
        pair=[r for r in rows if r['car']==0 and r['executed']]
        axes[2].plot([r['time_s']+.4 for r in pair],[r['min_sampled_pair_body_distance_m'] for r in pair],label=label)
    axes[0].set_ylabel('Pre-control goal distance / m');axes[0].legend(fontsize=8)
    axes[1].set_ylabel('Post-control signed speed / m/s')
    axes[2].set_ylabel('Executed interval min body gap / m');axes[2].legend()
    axes[2].set_xlabel('End of executed interval / simulation s')
    for ax in axes:ax.axvline(6,color='gray',linestyle='--')
    fig.tight_layout();fig.savefig(output/'initialization_comparison.png',dpi=160);plt.close(fig)


if __name__=='__main__':main()
