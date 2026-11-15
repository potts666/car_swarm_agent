#!/usr/bin/env python3
"""Bounded P0 driver. Exit 2 means the full P0 gate is not yet satisfied."""
import argparse
from fractions import Fraction
import hashlib
import importlib.util
from importlib.metadata import version as package_version
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT/'examples/head_on_mpc/scripts'))
from bicycle_core import rollout
from checks import shortest_straight, straight_source_certificate, independent_rollout
from theory_checks import curved_fixture_certificate


def write(path, obj):
    def encode(value):
        if isinstance(value, Fraction):
            return str(value)
        raise TypeError(type(value).__name__)
    path.write_text(json.dumps(obj, default=encode, indent=2, allow_nan=False)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    if out.exists():
        parser.error('output already exists; use a new directory to preserve evidence')
    out.mkdir(parents=True)
    (out/'source').mkdir()
    relative = ['experiments/local_coordination_p0/'+name for name in
                ('checks.py', 'test_checks.py', 'theory_checks.py', 'test_theory_checks.py',
                 'run.py', 'README.md', 'pending_parameters.json', 'design_source.txt',
                 'theorem_premises.md', 'theorem_premises.json')]
    relative += [str(p.relative_to(ROOT)) for p in sorted((HERE/'theory').rglob('*')) if p.is_file()]
    relative += ['examples/head_on_mpc/config/m2b_fourcar.json',
                 'examples/head_on_mpc/scripts/'+ 'bicycle_core.py',
                 'examples/head_on_mpc/scripts/m2b_protocol.py',
                 'examples/head_on_mpc/scripts/m2b_scene.py',
                 'examples/head_on_mpc/scripts/m2b_candidates.py',
                 'examples/head_on_mpc/scripts/m2b_control.py']
    manifest = {}
    for name in relative:
        path = ROOT/name
        target = out/'source'/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
        manifest[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    config = ROOT/'examples/head_on_mpc/config/m2b_fourcar.json'
    scene = json.loads(config.read_text())
    version = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    tracked_changes = subprocess.check_output(['git', 'diff', 'HEAD', '--name-only'], cwd=ROOT, text=True).splitlines()
    write(out/'provenance.json', dict(base_commit=version, source_sha256=manifest,
          python=sys.version, platform=platform.platform(), execution='development preflight only',
          dependency_versions={name:package_version(name) for name in ('numpy', 'scipy')},
          design_sha256=hashlib.sha256((HERE/'design_source.txt').read_bytes()).hexdigest(),
          tracked_worktree_changes=tracked_changes,
          historical_files_modified=any(not name.startswith('experiments/local_coordination_p0/') for name in tracked_changes)))
    stages = []

    def stage(name, operation):
        start = time.monotonic()
        try:
            result = operation()
        except Exception as error:
            stages.append(dict(stage=name, status='failed', exit_code=1,
                               error=repr(error), evidence=str(out/(name+'.json'))))
            write(out/(name+'.json'), dict(error=repr(error)))
            write(out/'stages.json', stages)
            raise
        stages.append(dict(stage=name, status='passed', exit_code=0,
                           wall_s=time.monotonic()-start, evidence=str(out/(name+'.json'))))
        write(out/(name+'.json'), result)
        write(out/'stages.json', stages)
        print(name+': passed', flush=True)
        return result

    def tests():
        codes = {}
        with (out/'tests.log').open('w') as log:
            for name in ('test_checks.py', 'test_theory_checks.py'):
                result = subprocess.run([sys.executable, str(HERE/name), '-v'],
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                log.write(name+'\n'+result.stdout)
                codes[name] = result.returncode
                if result.returncode:
                    raise RuntimeError('test exit '+str(result.returncode)+'; see '+str(out/'tests.log'))
        return dict(exit_codes=codes, log='tests.log')
    stage('small_checks', tests)

    def straight():
        whole = shortest_straight(4, scene); half = shortest_straight(2, scene)
        commands = [(float(a), 0., 'STRAIGHT') for a in whole['accelerations']]
        actual = rollout([0, 0, 0, 0, 0], commands, scene)
        write(out/'straight_controls.json', commands)
        write(out/'straight_actual_states.json', actual.tolist())
        return dict(motion_handoff=whole, stop_half=half, stop_total_s=2*half['seconds'],
                    moving_join_state=[2, 0, 0, 2, 0], stopping_join_state=[2, 0, 0, 0, 0],
                    theory_scope='exact uniform-grid ZOH longitudinal model, rest-to-rest, no obstacles',
                    continuous_time_stop_infimum_s=4*np.sqrt(2),
                    observation_final_state=actual[-1].tolist(), continuous_fourcar_certified=False)
    stage('straight_benchmark', straight)

    def source_box():
        source = [[-40.01, -39.99], [2.99, 3.01], [0, 0], [.5, .51], [0, 0]]
        road = [-52, -10, -5.5, 5.5]
        certificate = straight_source_certificate(source, [.1]*32, road, scene)
        if not certificate['passed']:
            raise RuntimeError('restricted straight certificate failed')
        return dict(source_box=source, road=road, acceleration_sequence=[.1]*32,
                    dt_s=scene['h'], certificate=certificate,
                    certificate_version='p0-restricted-straight-v1',
                    scope='isolated fixture; not the full four-car route certificate')
    stage('straight_source_box', source_box)

    def curve():
        start = [-40, 3, 0, 1, 0]
        commands = [(0., .05, 'RAMP')]*10+[(0., 0., 'CONTINUE')]*5
        independent = independent_rollout(start, commands, scene)
        n = round(scene['h']/scene['sample_s'])
        actual = rollout(start, commands, scene)[::n]
        error = float(np.max(abs(actual-independent)))
        if error > 1e-9:
            raise RuntimeError('independent integration mismatch')
        return dict(start=start, commands=commands, rk4_states=actual.tolist(),
                    dop853_states=independent.tolist(), max_state_error=error,
                    numerical_comparison_tolerance=1e-9, rigorous_integral_enclosure=False,
                    theory_scope='numerical calibration only; no source-set or road/six-pair certificate')
    stage('curve_calibration', curve)

    def curved_domain():
        certificate = curved_fixture_certificate()
        if not certificate['passed']:
            raise RuntimeError('analytic curved-domain fixture certificate failed')
        commands = [(float(a),float(w),'THEORY_REF') for a,w in certificate['reference_inputs']]
        centers = independent_rollout([0,0,0,.5,0], commands, scene)
        return dict(certificate=certificate, auxiliary_receiver_centers=centers.tolist(),
                    numerical_centers_certified=False,
                    operational_receiver_membership_validator_implemented=False,
                    distinction='analytic source-set bound, numerical centers only for debugging')
    stage('theory_curved_domain', curved_domain)

    dependency = dict(osqp_available=importlib.util.find_spec('osqp') is not None,
                      note='No solver was substituted for the historical OSQP dependency.')
    if dependency['osqp_available']:
        dependency['version'] = package_version('osqp')
        result = subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-s',
                str(ROOT/'examples/head_on_mpc/tests'), '-p', 'test_m2b_protocol.py', '-v'],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        (out/'historical_protocol.log').write_text(result.stdout)
        dependency.update(exit_code=result.returncode, passed=result.returncode == 0)
    else:
        dependency.update(exit_code=None, passed=False, status='not_run_missing_osqp')
    write(out/'historical_protocol.json', dependency)

    missing = ['Full straight/turn/exit/wait/park source-set connector library is absent.',
               'Continuous full-body road and six-pair enclosure is absent.',
               'SOCP, explicit inherited anchor and complete OCP revalidation are absent.',
               'Three-car merge/split/permission transition is not implemented or certified.',
               'Provisional seam guards are not integrated with the historical controller.',
               'Historical reverse-speed/goal tolerances and abort behavior differ from theorem assumptions.',
               'The archived 141-node/402-edge graph and its certificate dependencies are not available here.',
               'Small analytic receiver sets still lack operational certified numeric membership checks.']
    if not dependency['passed']:
        missing.append('Historical protocol regression did not pass or could not run; see historical_protocol.json.')
    gate = dict(theory_read=True, premise_matrix='theorem_premises.json',
                p0_passed=False, algorithm_frozen=False, pilot_allowed=False,
                formal_batch_allowed=False, blockers=missing, exit_code=2,
                note='Passing preflight fixtures is insufficient to satisfy the design P0 gate.')
    write(out/'gate.json', gate)
    methods = dict(**{'S-L':dict(stop=True, time_objective='m'),
                      'S-T':dict(stop=True, time_objective='m+continuation'),
                      'M-L':dict(stop=False, time_objective='m'),
                      'M-T':dict(stop=False, time_objective='m+continuation'),
                      'M-R':dict(stop=False, time_objective='m+continuation', local_socp=False)})
    write(out/'pilot_plan.json', dict(status='not_started_P0_not_passed', development_only=True,
          methods=methods, cases=['asymmetric_baseline', 'tight_curve_handoff', 'exit_merge_dependency'],
          runs=[dict(case=case, method=method, status='not_run') for case in
                ['asymmetric_baseline', 'tight_curve_handoff', 'exit_merge_dependency'] for method in methods],
          executable_method_implementations=False, formal_batch_enabled=False))
    stages.append(dict(stage='full_P0_gate', status='not_passed', exit_code=2, evidence=str(out/'gate.json')))
    write(out/'stages.json', stages)
    (out/'HANDOFF.md').write_text(
        'Goal: certify P0 before the five-method, three-case development pilot.\n'
        'Constraints: no formal batch, no paper edits, preserve historical results.\n'
        'Base commit: '+version+'; exact sources: provenance.json and source/.\n'
        'Evidence: stages.json, tests.log, gate.json, pilot_plan.json.\n'
        'Theory read: source/theory/ via the saved original README_UBUNTU.md index.\n'
        'Unresolved: see gate.json and theorem_premises.json; full graph/certificate objects required.\n'
        'Resume with a NEW output directory:\n'
        'PYTHONPATH=/tmp/dmpc-p0-deps python3 experiments/local_coordination_p0/run.py --output /tmp/dmpc-p0-next\n')
    print('full_P0_gate: NOT PASSED; exit 2; evidence '+str(out/'gate.json'), flush=True)
    return 2


if __name__ == '__main__':
    raise SystemExit(main())
