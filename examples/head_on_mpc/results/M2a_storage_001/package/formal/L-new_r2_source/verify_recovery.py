#!/usr/bin/env python3
"""Independent global-log audit of detection, continuity and original goals."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from head_on_core import Settings, body_gap, inside_road, write_json


def verify(run):
    study=json.loads((run/'study_config.json').read_text())
    config=study['recovery'];results=json.loads((run/'summary.json').read_text());findings=[]
    for result in results:
        path=run/f'case_{result["case"]:03d}'/result['variant']
        scene=json.loads((path/'config.json').read_text())['scene'];cfg=Settings(**scene['settings'])
        with (path/'actual.csv').open() as stream:rows=list(csv.DictReader(stream))
        by_time={};intervals=0
        for c in ('A','B'):
            samples=[r for r in rows if r['car']==c]
            assert float(samples[0]['absolute_time_s'])==cfg.epoch
            np.testing.assert_allclose([float(samples[0]['x']),float(samples[0]['vx'])],scene['initial_states_x_v'][c],atol=0.,rtol=0.)
            values=np.array([[float(r[k]) for k in ('x','vx','y')] for r in samples])
            assert np.all(np.isfinite(values)) and np.all(inside_road(values,scene['road'],cfg))
            assert np.max(np.abs(values[:,1]))<=cfg.max_speed+cfg.tolerance
            assert all(abs(float(r['acceleration']))<=cfg.max_acceleration+cfg.tolerance for r in samples)
            for a,b in zip(samples,samples[1:]):
                dt=float(b['absolute_time_s'])-float(a['absolute_time_s']);assert dt>=-1e-9
                x,v,u=float(a['x']),float(a['vx']),float(a['acceleration'])
                assert abs(float(b['x'])-(x+dt*v+.5*u*dt*dt))<1e-8
                assert abs(float(b['vx'])-(v+dt*u))<1e-8
                if dt<=1e-9:assert abs(float(b['y'])-float(a['y']))<1e-12
                else:
                    assert abs(dt-cfg.dt/cfg.substeps)<1e-8;intervals+=1
            for r in samples:by_time.setdefault(float(r['absolute_time_s']),{})[c]=[float(r[k]) for k in ('x','vx','y')]
        min_actual=min(float(body_gap(np.array([pair['A']]),np.array([pair['B']]),cfg)[0]) for pair in by_time.values())
        assert min_actual>=cfg.margin-cfg.tolerance
        assert abs(min_actual-result['minimum_executed_gap_m'])<1e-8
        prediction_gaps=[]
        for line in (path/'predictions.jsonl').read_text().splitlines():
            p=json.loads(line)
            prediction_gaps.append(float(np.min(body_gap(np.array(p['states']['A']),np.array(p['states']['B']),cfg))))
        if prediction_gaps:assert abs(min(prediction_gaps)-result['minimum_predicted_gap_m'])<1e-8
        events=[json.loads(line) for line in (path/'events.jsonl').read_text().splitlines()]
        detection=[e for e in events if e['kind']=='mutual_blocking_detected']
        assert len(detection)==int(result['detected'])
        if detection:
            e=detection[0]['evidence'];threshold=e['thresholds']
            assert not e['strict_deadlock_proof'] and e['window_observed_s']>=threshold['window_s']-1e-8
            assert all(e['unfinished'].values())
            assert all(v<=threshold['actual_progress_m'] for v in e['actual_position_span_m'].values())
            assert all(v<=threshold['prediction_endpoint_change_m'] for v in e['endpoint_span_m'].values())
            assert all(v<=threshold['prediction_tail_displacement_m'] for v in e['maximum_prediction_tail_displacement_m'].values())
            assert all(v>=threshold['minimum_blocked_fraction'] for v in e['blocked_fraction'].values())
        if (path/'selection.json').exists():
            selected=json.loads((path/'selection.json').read_text())
            assert selected['actual_checkpoint_before']==selected['actual_checkpoint_after']
            assert selected['used_pass_results'] is False
            eligible=[c for c,v in selected['candidates'].items() if v['feasible']]
            expected=min(eligible,key=lambda c:(round(selected['candidates'][c]['exit_time_s'],9),round(selected['candidates'][c]['retreat_distance_m'],6),c)) if eligible else None
            assert expected==result['role']
            assert all(not r['role'] or r['role']==result['role'] for r in rows)
            for c in selected['candidates']:
                source=json.loads((path/'candidates'/f'{c}_retreat'/'config.json').read_text())
                assert source['initial_checkpoint']==selected['actual_checkpoint_before']
        releases=[e for e in events if e['kind']=='release'];completed=[e for e in events if e['kind']=='both_tasks_completed']
        assert bool(releases)==result['released'] and bool(completed)==result['both_tasks_completed']
        assert len(releases)<=1 and len(completed)<=1 and result['role_switch_count']==0
        if releases:
            e=releases[0];passer=e['holder'];x,v,y=e['states'][passer];region=e['scope']
            assert all(e['check']['checks'].values())
            assert (x-cfg.length/2>=region['xmax']+study['options']['release_margin_m']) if passer=='A' else (x+cfg.length/2<=region['xmin']-study['options']['release_margin_m'])
            assert abs(x-scene['goals_x'][passer])<=scene['arrival_position_tolerance_m']
            assert abs(v)<=scene['arrival_speed_tolerance_m_s']
            assert e['both_tasks_completed'] is False
        if completed:
            assert releases and completed[0]['absolute_time_s']>releases[0]['absolute_time_s']
            for c,s in completed[0]['states'].items():
                assert abs(s[0]-scene['goals_x'][c])<=scene['arrival_position_tolerance_m']
                assert abs(s[1])<=scene['arrival_speed_tolerance_m_s'] and abs(s[2])<=cfg.tolerance
        for r in json.loads((path/'solves.json').read_text()):
            if r['accepted']:
                assert r['independent_acceptance']['finite']
                assert all(v<=cfg.tolerance for v in r['independent_acceptance']['residuals'].values())
        if result.get('switch_guard_enabled'):
            switches=[json.loads(line) for line in (path/'switch_checks.jsonl').read_text().splitlines()]
            if result['both_tasks_completed']:
                assert {'PASS','CLEARED_HOLD','RETURN_MERGE','RETURN'} <= {s['to_mode'] for s in switches}
                assert all(s['allowed'] for s in switches)
            for s in switches:
                if s['allowed']:
                    assert s['witness'] in ('shifted_zero_tail','existing_hold_precheck','fresh_joint_plan')
                    accepted=s['shifted_candidate'] if s['witness']=='shifted_zero_tail' else s['fresh_candidate']
                    assert accepted['passed'] and accepted['pair']['passed']
                    assert all(a['passed'] for a in accepted['local'].values())
                assert s['not_a_recursive_feasibility_proof']
        findings.append(dict(case=result['case'],variant=result['variant'],passed=True,executed_substep_intervals=intervals,
                             both_tasks_completed=result['both_tasks_completed'],minimum_executed_gap_m=min_actual))
    for name,digest in json.loads((run/'provenance.json').read_text())['source_sha256'].items():
        assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest,name
    return dict(passed=True,findings=findings,scope='sampled dynamics and geometry; no continuous-time or deadlock theorem')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);args=p.parse_args()
    result=verify(args.run);write_json(args.run/'validation.json',result)
    print(json.dumps(dict(passed=result['passed'],runs=len(result['findings']),executed_substep_intervals=sum(r['executed_substep_intervals'] for r in result['findings']))))


if __name__=='__main__':main()
