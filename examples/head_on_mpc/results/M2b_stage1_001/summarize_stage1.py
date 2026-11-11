"""Small summaries only; stream actual/planning records and preserve raw evidence."""
import csv
import gzip
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[3]


def rows(path):
    compressed=path.with_name(path.name+'.gz')
    source=path if path.exists() else compressed
    with (gzip.open(source,'rt') if source.suffix=='.gz' else source.open()) as stream:
        for line in stream:yield json.loads(line)


def stats(values):
    a=np.array(values,float)
    return dict(count=len(a),mean=float(a.mean()),p50=float(np.percentile(a,50)),p95=float(np.percentile(a,95)),p99=float(np.percentile(a,99)),max=float(a.max()),over_200ms=int(np.sum(a>.2))) if len(a) else None


def signature(directory):
    h=hashlib.sha256();cost=0.;count=0
    for r in rows(directory/'actual.jsonl'):
        values={k:r[k] for k in ['cycle','owner','permission_version','states','controls']}
        h.update(json.dumps(values,sort_keys=True,separators=(',',':')).encode())
        cost+=.2*sum(u[0]**2+u[1]**2 for u in r['controls'].values());count+=1
    return dict(sha256=h.hexdigest(),cycles=count,input_square_integral=cost)


def summarize(directory):
    plans=list(rows(directory/'planning.jsonl'));committed=[p for p in plans if p['committed']]
    validation=json.loads((directory/'independent_validation.json').read_text())
    summary=json.loads((directory/'summary.json').read_text())
    labels={k for p in committed for k in p.get('timing_breakdown',{})}
    exclusive={k:float(np.mean([p.get('timing_breakdown',{}).get(k,{}).get('exclusive_s',0.) for p in committed])) for k in labels}
    phases={}
    for phase in ['before_last_final','after_last_final','commit_tail','diagnostics']:
        categories={k for p in committed for k in p.get('timing_phases',{}).get(phase,{})}
        if categories:
            phases[phase]={k:float(np.mean([p['timing_phases'].get(phase,{}).get(k,0.) for p in committed])) for k in categories}
    grey=None
    if committed and all('timing_candidate_phases' in p for p in committed):
        keys=set(phases.get('before_last_final',{}))
        grey={k:float(np.mean([p['timing_phases'].get('before_last_final',{}).get(k,0.)-p['timing_candidate_phases'].get('before_last_final',{}).get(k,0.) for p in committed])) for k in keys}
    for p in committed:
        if 'timing_breakdown' not in p:continue
        root=p['timing_breakdown']['cycle']['inclusive_s']
        assert abs(sum(r['exclusive_s'] for r in p['timing_breakdown'].values())-root)<1e-8
        assert abs(sum(sum(r.values()) for r in p['timing_phases'].values())-root)<1e-8
    solvers=list(rows(directory/'solves.jsonl'));local=[]
    for r in solvers:
        if r.get('local'):local.extend(v for v in r['local'].values() if v)
    return dict(summary=summary,actual_signature=signature(directory),
        validation={k:validation.get(k) for k in ['actual_full_acceptance','actual_prefix_acceptance','violations','original_sign_check_passed','local_information_boundary_passed','physical','dimensions']},
        cycle_s=stats([p['full_cycle_wall_s'] for p in committed]),
        consecutive_start_s=stats([p['start_to_next_start_wall_s'] for p in committed if 'start_to_next_start_wall_s' in p]),
        executable_s=stats([p['control_executable_wall_s'] for p in committed]),
        diagnostic_s=stats([p['diagnostic_and_recording_wall_s'] for p in committed]),
        local_predict_s=stats([p['solve_wall_s'] for p in local]),
        local_over_500ms=sum(bool(p['deadline_exceeded']) for p in local),
        phases_exclusive_mean_s=phases,grey_without_candidate_mean_s=grey,
        exclusive_mean_s=exclusive,application_bytes=sum(p['application_bytes'] for p in plans),
        cache_hits=(sum(p['timing_breakdown'].get('plan_cache_hit',{}).get('calls',0) for p in committed)
                    if committed and all('timing_breakdown' in p for p in committed) else None),
        cache_hits_measured=bool(committed) and all('timing_breakdown' in p for p in committed),
        stored_bytes=sum(p.stat().st_size for p in directory.rglob('*') if p.is_file()))


def write_csv(results):
    with (OUT/'comparison.csv').open('w',newline='') as stream:
        fields=['run','cycles','timing_sample_count','timing_scope','actual_full_acceptance','prefix_acceptance','completion_s','online_wall_s','p50_ms','p95_ms','p99_ms','max_ms','over_200ms','local_over_500ms','application_bytes','stored_bytes']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for name,m in results.items():
            s=m['summary'];use_intervals=s['all_completed'] and m['consecutive_start_s'] is not None;t=m['consecutive_start_s'] if use_intervals else m['cycle_s'];v=m['validation']
            writer.writerow(dict(run=name,cycles=m['cycle_s']['count'],timing_sample_count=t['count'],timing_scope='consecutive_starts' if use_intervals else 'through_diagnostic_end',actual_full_acceptance=v['actual_full_acceptance'],prefix_acceptance=v['actual_prefix_acceptance'],completion_s=s['completion_sim_s'],online_wall_s=s['online_wall_s'],**{q+'_ms':t[q]*1000 for q in ['p50','p95','p99','max']},over_200ms=t['over_200ms'],local_over_500ms=m['local_over_500ms'],application_bytes=m['application_bytes'],stored_bytes=m['stored_bytes']))


if __name__=='__main__':
    results={}
    for name in ['A_uncached','B_cached','B_no_profiler','full_L_FIFO','phase_probe_uncached','phase_probe_cached']:
        directory=OUT/name/'r1'
        if (directory/'independent_validation.json').exists():results[name]=summarize(directory)
    old=ROOT/'examples/head_on_mpc/results/M2b_fourcar_001/L-FIFO/r1'
    # Logical motion comparison excludes plan/source IDs, timing and new bookkeeping.
    historical=signature(old)
    comparison=dict(runs=results,historical_L_FIFO_r1_signature=historical,
                    motion_matches_historical=results.get('full_L_FIFO',{}).get('actual_signature')==historical,
                    scope='full_cycle ends after diagnostics; consecutive starts additionally cover timer finalization/progress; nested inclusive rows must not be summed')
    (OUT/'measurements.json').write_text(json.dumps(comparison,indent=2)+'\n')
    write_csv(results)
    print(json.dumps({'runs':list(results),'motion_matches_historical':comparison['motion_matches_historical']}))
