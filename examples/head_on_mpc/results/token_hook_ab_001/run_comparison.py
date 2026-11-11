"""Two bounded Codex measurements of the same existing tiny numerical checks."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
CLI = Path('/home/sunflower/.vscode-server/extensions/openai.chatgpt-26.1021.22027-linux-x64/bin/linux-x86_64/codex')
TESTS = [
    'test_entry_bicycle.BicycleTests.test_no_motion_from_steering_at_rest',
    'test_entry_bicycle.BicycleTests.test_arc_geometry_and_return_continuity',
    'test_entry_bicycle.BicycleTests.test_reverse_crosses_zero_and_curves_with_heading',
]
COMMAND = ('OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 '
           'PYTHONPATH=examples/head_on_mpc/tests python3 -m unittest ' + ' '.join(TESTS))
PROMPT = '''This is a small controlled measurement, not a development task.
Run these three existing bicycle numerical regression checks once, from the given repository cwd:
''' + COMMAND + '''
Do not edit code, read other sources/logs, install dependencies, add agents, or run full simulations.
Use the existing command as given. Keep tool output small. Report only test count, exit code and pass/fail.
Stop on failure; do not attempt repairs. At most four tool calls and 120 seconds.
Normal sandbox/approval rules still apply.
Host preflight has established that the default bwrap sandbox cannot start.
For this exact read-only numerical check, request sandbox_permissions=require_escalated
through the normal automatic approval flow; do not disable approvals or the sandbox globally.
An infrastructure permission request is allowed; a rejection or test failure must stop the run.
'''


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


if __name__ == '__main__':
    assert (ROOT/'examples/head_on_mpc').is_dir(), ROOT
    if (OUT/'manifest.json').exists():
        raise SystemExit('Refuse to overwrite an existing measurement')
    sources = ['examples/head_on_mpc/tests/test_entry_bicycle.py',
               'examples/head_on_mpc/scripts/bicycle_core.py',
               'examples/head_on_mpc/scripts/run_entry_bicycle.py',
               'examples/head_on_mpc/config/entry_bicycle.json',
               'tools/codex_token_guard/guard.py']
    manifest = dict(cwd=str(ROOT), cli=str(CLI), model='gpt-6.1-sol', reasoning='medium',
                    design='identical prompt / new sessions / hooks off then on / one pair only',
                    command=COMMAND, tests=TESTS, limit_seconds_per_arm=120,
                    scope='existing small numerical checks; no full task simulation',
                    source_sha256={s:digest(ROOT/s) for s in sources},
                    prompt_sha256=hashlib.sha256(PROMPT.encode()).hexdigest(), arms=[])
    (OUT/'prompt.txt').write_text(PROMPT)
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', PYTHONDONTWRITEBYTECODE='1')
    for arm, enabled in [('before', False), ('after', True)]:
        args = [str(CLI), '--no-daemon', 'exec', '--json', '--color', 'never',
                '--approve-for-me', '-m', manifest['model'],
                '-c', 'model_reasoning_effort="medium"',
                '--enable' if enabled else '--disable', 'hooks',
                '-C', str(ROOT), '-o', str(OUT/(arm+'.answer.txt')), '-']
        record = dict(arm=arm, hooks_enabled=enabled, argv=args, started_unix=time.time())
        print(arm+' started', flush=True)
        with (OUT/(arm+'.events.jsonl')).open('w') as stdout, (OUT/(arm+'.stderr.log')).open('w') as stderr:
            try:
                result = subprocess.run(args,input=PROMPT,text=True,stdout=stdout,stderr=stderr,
                                        env=env,timeout=120,cwd=ROOT)
                record['exit_code'] = result.returncode
                record['timed_out'] = False
            except subprocess.TimeoutExpired:
                record['exit_code'] = None
                record['timed_out'] = True
        record['ended_unix'] = time.time()
        record['wall_seconds'] = record['ended_unix']-record['started_unix']
        manifest['arms'].append(record)
        (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        print(arm+' finished '+json.dumps({k:record[k] for k in ['exit_code','timed_out','wall_seconds']}),flush=True)
        if record['timed_out'] or record['exit_code'] != 0:
            print('Stop comparison on unsuccessful model run; preserve evidence.',flush=True)
            break
    manifest['sources_unchanged'] = all(digest(ROOT/p)==h for p,h in manifest['source_sha256'].items())
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
