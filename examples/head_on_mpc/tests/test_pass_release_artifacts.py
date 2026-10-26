"""Artifact contracts and reports must also work for aborted experiments."""
import csv
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from head_on_core import Settings, write_json
from pass_release_core import geometry
from run_comparison import make_scene
from run_exit_screening import generate_exit
from run_pass_release import failed_exit_branch, report
from verify_pass_release import verify


class PassReleaseArtifacts(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.run = Path(self.folder.name)
        self.base = json.loads((ROOT/'config/comparison.json').read_text())
        self.options = json.loads((ROOT/'config/pass_release.json').read_text())
        self.scene = make_scene(self.base, -3., 0., 20., .5)
        self.scene['duration_s'] = .4
        case = self.run/'case_000'
        source = case/'candidates'/'A_retreat'
        candidate = generate_exit(source, self.scene, 'A')
        self.assertFalse(candidate['feasible'])
        branches = {
            policy: failed_exit_branch(case/policy, source, self.scene, policy, role, .1)
            for policy, role in (('fixed_A', 'A'), ('screened', None))
        }
        self.result = dict(case=0, factors=self.scene['factors'],
                           geometry=geometry(self.scene, Settings(**self.scene['settings']), self.options),
                           branches=branches, same_role_check=None,
                           selection_wait_wall_s=.1, candidate_generation_wall_s=.1)
        write_json(case/'selection.json', dict(used_pass_results=False,
            candidates={'A': candidate, 'B': candidate}, selected={'screened': {'role': None}}, primary_calls=2))
        self.save_study()
        write_json(self.run/'provenance.json', {'source_sha256': {
            str(ROOT/'scripts/verify_pass_release.py'):
                hashlib.sha256((ROOT/'scripts/verify_pass_release.py').read_bytes()).hexdigest()}})

    def save_study(self):
        write_json(self.run/'study_config.json', dict(base=self.base, options=self.options, hold_constraints=True))
        write_json(self.run/'summary.json', [self.result])

    def test_exit_failures_have_complete_logs_and_verify_without_holding(self):
        result = verify(self.run)
        self.assertTrue(result['passed'])
        self.assertEqual(result['branches'], 2)
        self.assertTrue(all(not f['exit_handoff_verified'] for f in result['findings']))
        for policy in ('fixed_A', 'screened'):
            directory = self.run/'case_000'/policy
            self.assertEqual(json.loads((directory/'hold_prechecks.json').read_text()), [])
            events = [json.loads(s) for s in (directory/'events.jsonl').read_text().splitlines()]
            self.assertEqual(events[-1]['failure_phase'], 'EXIT')
            self.assertTrue(events[-1]['simulation_terminated'])
            self.assertFalse(any(e['kind'] == 'release' for e in events))
        report(self.run, [self.result])
        self.assertIn('exit_candidate_failed:exit_timeout', (self.run/'report.md').read_text())

    def test_exit_failure_cannot_be_relabeled_as_passed(self):
        self.result['branches']['fixed_A']['pass_success'] = True
        self.save_study()
        with self.assertRaises(AssertionError):
            verify(self.run)

    def test_pass_rows_without_a_grant_are_rejected(self):
        path = self.run/'case_000'/'fixed_A'/'actual.csv'
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        rows[0]['stage'] = 'PASS'
        with path.open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        with self.assertRaises(AssertionError):
            verify(self.run)

    def test_ready_failure_has_no_handoff_to_verify(self):
        summary = self.result['branches']['fixed_A']
        summary.update(exit_success=True, failure_phase='PARKED_READY', failure='parked_ready_or_continuity_failed')
        directory = self.run/'case_000'/'fixed_A'
        write_json(directory/'summary.json', summary)
        event = json.loads((directory/'events.jsonl').read_text())
        event.update(failure_phase=summary['failure_phase'], failure=summary['failure'])
        (directory/'events.jsonl').write_text(json.dumps(event)+'\n')
        self.save_study()
        self.assertTrue(verify(self.run)['passed'])

    def test_aborted_branches_still_check_actual_body_safety(self):
        path = self.run/'case_000'/'fixed_A'/'actual.csv'
        with path.open() as stream:
            rows = list(csv.DictReader(stream))
        rows[0]['x'] = self.scene['road']['xmax'] + 10
        with path.open('w') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        with self.assertRaises(AssertionError):
            verify(self.run)

    def test_report_uses_saved_geometry_and_nondefault_thresholds(self):
        self.base['settings'].update(length=5., dt=.5, horizon=10, substeps=5)
        self.options.update(release_margin_m=.7, cleared_hold_s=1.2)
        scene = make_scene(self.base, 0., 0., 24., 1.)
        scene.update(arrival_position_tolerance_m=.12, arrival_speed_tolerance_m_s=.03)
        self.result['geometry'] = geometry(scene, Settings(**scene['settings']), self.options)
        self.result['factors'] = scene['factors']
        self.save_study()
        report(self.run, [self.result])
        text = (self.run/'report.md').read_text()
        for fragment in ('车长 5 m', '释放裕度配置为 0.7 m', '连续保持 1.2 s',
                         'x−2.5≥R.xmax+0.7', '[-12, 12]', '24.5 / -24.5', '0.12 / 0.03',
                         '控制周期0.5 s、预测5 s、检查间隔0.1 s'):
            self.assertIn(fragment, text)
        self.assertNotIn('释放裕度明确为0.5 m', text)
        self.assertNotIn('并连续保持0.8 s', text)


if __name__ == '__main__':
    unittest.main()
