"""Small cross-file checks, coverage and provenance; never re-read large plan logs."""
import csv
import hashlib
import json
import platform
from pathlib import Path
import sys
from analyze import OUT,ROOT,RESULTS,SOURCE,load,write
from analysis_io import file_sha


if __name__=='__main__':
    inv=load(OUT/'input_identity.json');results=load(OUT/'analysis_results.json');detail=load(OUT/'detailed_results.json')
    assert len(inv)==17 and not load(OUT/'status.json')['errors']
    assert len({r['physical_scenario'] for r in inv})==1
    assert all(v['passed'] for v in load(OUT/'alias_protocol_validation.json').values())
    shared=load(OUT/'shared_diagnostic_integrity.json')
    assert len(shared['runs'])==6 and all(v['passed'] for v in shared['runs'].values())
    guards=load(OUT/'task_events.json')['L-FIFO/r1']
    assert all(r['ready_guard_passed'] for r in guards['handoff_guards'] if 'ready_guard_passed' in r)
    assert all(r['actual_release_passed'] for r in guards['handoff_guards'] if 'actual_release_passed' in r)
    assert all(r['original_progress_speed_guard_passed'] for r in guards['segment_transition_guards'])
    with (OUT/'candidate_coverage.csv').open('w',newline='') as f:
        fields=['run','metrics_source','seed_sets','seed_car_candidates','initial_car_candidates','shifted_car_candidates','seed_physical_pass_ratio','seed_dynamics_checked','seed_dynamics_pass_ratio','partial_phase_checked','partial_phase_pass_ratio','partial_terminal_checked','partial_terminal_pass_ratio','full_ocp_checked','full_ocp_pass_ratio']
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
        for row in inv:
            name=row['run'];src=name
            while 'reused_from' in results[src]:src=results[src]['reused_from']
            n=results[src]['counts'];x=detail[src]['dynamics_checks'];total=n['seed_replays'];checked=x['seed_replay_error:checked']
            w.writerow(dict(run=name,metrics_source=src,seed_sets=n['seed_complete'],seed_car_candidates=total,initial_car_candidates=total-n.get('shift_candidates',0),shifted_car_candidates=n.get('shift_candidates',0),seed_physical_pass_ratio=n['seed_physical_protocol_passed']/n['seed_complete'] if n['seed_complete'] else None,seed_dynamics_checked=checked,seed_dynamics_pass_ratio=x.get('seed_replay_error:within_1e-12_per_reported_unit',0)/checked if checked else None,partial_phase_checked=n.get('seed_phase_checked',0),partial_phase_pass_ratio=n.get('seed_phase_passed',0)/n['seed_phase_checked'] if n.get('seed_phase_checked') else None,partial_terminal_checked=n.get('seed_terminal_rows_checked',0),partial_terminal_pass_ratio=n.get('seed_terminal_rows_passed',0)/n['seed_terminal_rows_checked'] if n.get('seed_terminal_rows_checked') else None,full_ocp_checked=0,full_ocp_pass_ratio=None))
    expected=load(SOURCE/'manifest.json');checks={name:file_sha(ROOT/'examples/head_on_mpc/scripts'/name)==digest for name,digest in expected.items()}
    assert all(checks.values()),'maintained runtime differs from frozen baseline'
    snapshots={}
    for row in inv:
        if not row['source_snapshot_available']:continue
        source=Path(row['directory']).parent.parent/'sources'/row['source_version']
        if str(source) in snapshots:continue
        hashes=load(source/'manifest.json');bad=[n for n,v in hashes.items() if file_sha(source/n)!=v]
        assert not bad
        snapshots[str(source)]=dict(files=len(hashes),mismatches=bad)
    write(OUT/'source_snapshot_checks.json',snapshots)
    write(OUT/'scenario_grouping.json',dict(physical_scenarios=1,excluded_metadata=['source_version','log_format'],
        reason='the second raw config fingerprint was solely a log_format difference; raw input byte SHA retained',
        raw_motion_groups=len({r['motion_group'] for r in inv}),motion_groups_are_not_independent_scenarios=True))
    audit_path=RESULTS/'M2b_token_audit_001/actor_totals.json';actors=load(audit_path)
    write(OUT/'prior_token_reference.json',dict(main_record_tokens=sum(a['totals']['total_tokens'] for a in actors if a['role']=='main'),
        guardian_record_tokens=sum(a['totals']['total_tokens'] for a in actors if a['role']!='main'),cached_input_included_in_input=sum(a['totals']['cached_input_tokens'] for a in actors),
        source=str(audit_path),source_sha256=file_sha(audit_path),current_analysis_tokens=None,fee_or_quota_conversion=None))
    import numpy,scipy,osqp,joblib,matplotlib
    prefix='OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/tmp/m2b_stage1_repro_deps python3 '
    write(OUT/'execution.json',dict(working_directory=str(ROOT),host_platform=platform.platform(),python=platform.python_version(),
        numerical_versions={m.__name__:m.__version__ for m in [numpy,scipy,osqp,joblib,matplotlib]},
        commands=[prefix+str(OUT/n) for n in ['test_analysis.py','analyze.py','summarize.py','counterexamples.py','make_dictionary.py','plots.py','verify_shared_diagnostics.py','verify_finish.py','make_report.py']],
        frozen_source_version=SOURCE.name,maintained_runtime_matches_frozen_baseline=checks,
        control_simulation_rerun=False,optimizer_called=False,settings_or_permissions_changed=False,
        analysis_wall_s=None,analysis_wall_reason='multi-script analysis wall not instrumented; no model-time conversion'))
    print('Coverage and provenance checks passed; maintained controller sources unchanged.')
