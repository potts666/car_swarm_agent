#!/usr/bin/env python3
"""Read-only audit of actual logs, handoff, clearance and release events."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from head_on_core import Settings, body_gap, inside_road, write_json


def verify(run):
    cases=json.loads((run/'summary.json').read_text())
    study=json.loads((run/'study_config.json').read_text())
    options=study['options']
    total_intervals=0
    findings=[]
    for case in cases:
        selection=json.loads((run/f'case_{case["case"]:03d}'/'selection.json').read_text())
        assert selection['used_pass_results'] is False
        eligible=[c for c in ('A','B') if selection['candidates'][c]['feasible']]
        candidate_scene=json.loads((run/f'case_{case["case"]:03d}'/'candidates'/'A_retreat'/'config.json').read_text())
        chosen=min(eligible,key=lambda c:(abs(candidate_scene['initial_states_x_v'][c][0]-candidate_scene['retreat_x'][c]),c)) if eligible else None
        assert selection['selected']['screened']['role']==chosen
        for policy,summary in case['branches'].items():
            directory=run/f'case_{case["case"]:03d}'/policy
            config=json.loads((directory/'config.json').read_text())
            scene=config['scene'];cfg=Settings(**scene['settings'])
            with (directory/'actual.csv').open() as stream:rows=list(csv.DictReader(stream))
            events=[json.loads(line) for line in (directory/'events.jsonl').read_text().splitlines()]
            groups={}
            for r in rows:
                assert all(np.isfinite(float(r[k])) for k in ('absolute_time_s','x','vx','y','acceleration'))
                groups.setdefault(float(r['absolute_time_s']),{})[r['car']]=r
                assert (r['role'] or None)==summary['role']
                s=np.array([[float(r['x']),float(r['vx']),float(r['y'])]])
                assert inside_road(s,scene['road'],cfg)[0]
                assert abs(float(r['vx']))<=cfg.max_speed+cfg.tolerance
                assert abs(float(r['acceleration']))<=cfg.max_acceleration+cfg.tolerance
            min_gap=float('inf')
            for pair in groups.values():
                a,b=(np.array([[float(pair[c][k]) for k in ('x','vx','y')]]) for c in ('A','B'))
                min_gap=min(min_gap,float(body_gap(a,b,cfg)[0]))
            assert min_gap>=cfg.margin-cfg.tolerance
            assert abs(min_gap-summary['minimum_executed_gap_m'])<1e-8
            for c in ('A','B'):
                samples=[r for r in rows if r['car']==c]
                for a,b in zip(samples,samples[1:]):
                    dt=float(b['absolute_time_s'])-float(a['absolute_time_s'])
                    assert dt>=-1e-9
                    x,v,u=float(a['x']),float(a['vx']),float(a['acceleration'])
                    assert abs(float(b['x'])-(x+dt*v+.5*u*dt*dt))<1e-8
                    assert abs(float(b['vx'])-(v+u*dt))<1e-8
                    if dt<=1e-9:assert abs(float(b['y'])-float(a['y']))<1e-12
                    else:
                        assert abs(dt-cfg.dt/cfg.substeps)<1e-8
                        total_intervals+=1
            grants=[e for e in events if e['kind']=='grant']
            pass_started=bool(grants)
            if pass_started:
                assert summary['exit_success'] and len(grants)==1
            else:
                assert not summary['pass_success'] and not summary['released']
                assert summary['failure_phase'] in ('EXIT', 'PARKED_READY')
                assert summary['exit_success']==(summary['failure_phase']=='PARKED_READY')
                assert not any(r['stage'] in ('PASS', 'CLEARED_HOLD', 'RELEASED') for r in rows)
            if pass_started:
                grant=grants[0]
                assert grant['before']==grant['after']
                assert grant['holder']==summary['passer']
                assert all(v==0. for v in grant['continuity']['residuals'].values())
                for c in ('A','B'):
                    last_exit=next(r for r in reversed(rows) if r['car']==c and r['stage']=='EXIT')
                    assert abs(float(last_exit['absolute_time_s'])-grant['absolute_time_s'])<1e-8
                    for i,k in enumerate(('x','vx','y')):
                        assert abs(float(last_exit[k])-grant['before']['states'][c][i])<1e-12
                    assert float(last_exit['acceleration'])==grant['before']['previous_acceleration'][c]
                role=summary['role'];reference=summary['parked_reference_actual']
                suffix=[r for r in rows if r['car']==role and r['stage']!='EXIT']
                assert all(abs(float(r['x'])-reference[0])<=options['parked_position_tolerance_m'] and
                           abs(float(r['vx']))<=options['parked_speed_tolerance_m_s'] and
                           abs(float(r['y'])-reference[2])<=options['parked_lateral_tolerance_m'] for r in suffix)
            # Prediction minima are measured independently, never merged into actual minima.
            prediction_gaps=[]
            for line in (directory/'predictions.jsonl').read_text().splitlines():
                entry=json.loads(line)
                a,b=(np.array(entry['states'][c],float) for c in ('A','B'))
                prediction_gaps.append(float(np.min(body_gap(a,b,cfg))))
            if prediction_gaps:
                assert abs(min(prediction_gaps)-summary['minimum_predicted_gap_m'])<1e-8
            records=json.loads((directory/'solves.json').read_text())
            for record in records:
                if record['accepted']:
                    assert record['independent_acceptance']['passed']
                    assert all(v<=cfg.tolerance for v in record['independent_acceptance']['residuals'].values())
            if pass_started:
                for c in ('A','B'):
                    last_solve=next(r for r in reversed(records) if r['stage']=='EXIT' and r['car']==c)
                    expected=np.r_[last_solve['controls'][1:],last_solve['controls'][-1]]
                    np.testing.assert_array_equal(expected,grant['before']['warm_controls'][c])
                c=summary['passer'];region=summary['geometry']['conflict_region']
                actual_cleared=[]
                for row in rows:
                    if row['car']!=c or row['stage']=='EXIT':continue
                    x=float(row['x'])
                    clear=(x-cfg.length/2>=region['xmax']+options['release_margin_m']) if c=='A' else (x+cfg.length/2<=region['xmin']-options['release_margin_m'])
                    if clear:actual_cleared.append(float(row['absolute_time_s'])-cfg.epoch)
                if actual_cleared:
                    assert abs(min(actual_cleared)-summary['first_actual_body_cleared_time_s'])<1e-8
            releases=[e for e in events if e['kind']=='release']
            assert len(releases)==int(summary['released'])
            assert summary['both_tasks_completed'] is False
            if releases:
                release=releases[0];c=summary['passer'];x,v,y=release['states'][c]
                region=summary['geometry']['conflict_region'];margin=options['release_margin_m']
                assert (x-cfg.length/2>=region['xmax']+margin) if c=='A' else (x+cfg.length/2<=region['xmin']-margin)
                assert abs(x-summary['geometry']['passing_stop_targets_x'][c])<=scene['arrival_position_tolerance_m']
                assert abs(v)<=scene['arrival_speed_tolerance_m_s']
                assert release['holder']==c
                assert all(release['check']['checks'].values())
                assert summary['release_time_s']-summary['first_stop_candidate_time_s']>=options['cleared_hold_s']-1e-8
                assert summary['first_actual_body_cleared_time_s']<summary['release_time_s']
                assert release['both_tasks_completed'] is False
                assert summary['failure'] is None
            else:
                assert summary['overall_success'] is False
                assert not any(e.get('to')=='RELEASED' for e in events)
            checks=json.loads((directory/'path_checks.json').read_text())
            for check in checks:
                if check.get('stage')!='EXIT' and check.get('kind')=='executed_segment' and check.get('applied_to_simulation'):
                    assert check['passed']
                    assert all(v<=cfg.tolerance for entry in check['dynamics_residuals'].values() for v in entry.values())
            if study.get('hold_constraints', False) and pass_started:
                prechecks=json.loads((directory/'hold_prechecks.json').read_text())
                if summary['released']:
                    assert len(prechecks)==1 and prechecks[0]['passed']
                    assert prechecks[0]['before']==prechecks[0]['after']
                    assert prechecks[0]['hold']['passed'] and prechecks[0]['pair']['passed']
                    assert prechecks[0]['stop_candidate']['applied'] is False
                holding_rows=[r for r in rows if r['stage']=='CLEARED_HOLD' and r['car']==summary['passer']]
                target=summary['geometry']['passing_stop_targets_x'][summary['passer']]
                assert all(abs(float(r['vx']))<=scene['arrival_speed_tolerance_m_s'] and
                           abs(float(r['x'])-target)<=scene['arrival_position_tolerance_m'] for r in holding_rows)
                for record in records:
                    if record['stage']=='CLEARED_HOLD' and record['car']==summary['passer']:
                        assert record['hold_bounds'] is not None
                diag=[r for r in records if r.get('diagnostic')]
                assert len(diag)==summary['diagnostic_calls']
            finding=dict(case=case['case'],policy=policy,actual_intervals_verified=True,minimum_executed_gap_m=min_gap,
                         released=summary['released'],exit_handoff_verified=pass_started,role_unchanged=True)
            findings.append(finding)
        if case['same_role_check']:
            assert case['same_role_check']['identical']
            hashes=[hashlib.sha256((run/f'case_{case["case"]:03d}'/p/'actual.csv').read_bytes()).hexdigest() for p in ('fixed_A','screened')]
            assert hashes[0]==hashes[1]
    for name,digest in json.loads((run/'provenance.json').read_text())['source_sha256'].items():
        assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest,name
    return dict(passed=True,branches=len(findings),executed_substep_intervals=total_intervals,
                source_hashes_match=True,selection_exit_only=True,findings=findings)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run',type=Path,required=True)
    args=parser.parse_args();result=verify(args.run)
    write_json(args.run/'validation.json',result)
    print(json.dumps({k:v for k,v in result.items() if k!='findings'}))


if __name__=='__main__':main()
