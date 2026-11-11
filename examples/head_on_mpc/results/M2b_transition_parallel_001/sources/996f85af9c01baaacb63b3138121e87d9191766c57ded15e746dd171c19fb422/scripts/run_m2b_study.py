#!/usr/bin/env python3
"""Sequential formal execution: no online run overlaps another run or an audit."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import threading

SCRIPTS=Path(__file__).resolve().parent


def write(path,data):path.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def run(root):
    if (root/'formal_freeze.json').exists():raise FileExistsError('formal freeze already exists; do not overwrite a study')
    names=sorted({p.name for p in SCRIPTS.glob('*m2b*.py')}|{'m2a_logs.py','m2a_local.py','m2a_protocol.py','bicycle_core.py','linearized_joint.py','longitudinal_joint.py','verify_entry_bicycle.py','run_entry_bicycle.py','recovery_core.py'})
    hashes={name:sha(SCRIPTS/name) for name in names};scene=SCRIPTS.parent/'config/m2b_fourcar.json'
    freeze=dict(source_sha256=hashes,scene_sha256=sha(scene),scene=json.loads(scene.read_text()),control_watchdog_s=1800,group_process_timeout_s=4200,order=['C-FIFO','L-FIFO','L-ALT'],repetitions=2,thread_env='OPENBLAS_NUM_THREADS=1',online_and_audits_overlap=False,formal_started_wall_ns=time.time_ns())
    write(root/'formal_freeze.json',freeze);status=dict(groups={},completed=False);write(root/'execution_status.json',status)
    log=(root/'formal_execution.log').open('x')
    def execute(command,env):
        text=' '.join(map(str,command));print('START',text,flush=True);log.write('START '+text+'\n');log.flush()
        p=subprocess.Popen(list(map(str,command)),stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,env=env)
        timer=threading.Timer(4200,p.kill);timer.daemon=True;timer.start()
        try:
            for line in p.stdout:print(line,end='',flush=True);log.write(line);log.flush()
            result=p.wait()
        finally:timer.cancel()
        if result:raise RuntimeError(f'process exit {result}: {text}')
    try:
        for group,arm,policy,domain in [('C-FIFO','central','fifo-ready','121'),('L-FIFO','local','fifo-ready','122'),('L-ALT','local','alternating-ready','123')]:
            for name,digest in hashes.items():
                if sha(SCRIPTS/name)!=digest:raise RuntimeError('frozen source changed '+name)
            if sha(scene)!=freeze['scene_sha256']:raise RuntimeError('frozen scene changed')
            if group=='L-ALT':
                for i in (1,2):
                    v=json.loads((root/'L-FIFO'/f'r{i}/independent_validation.json').read_text())
                    if not v['actual_full_acceptance'] or not v['local_information_boundary_passed']:raise RuntimeError('L-FIFO gate failed: L-ALT not run')
            env={**os.environ,'OPENBLAS_NUM_THREADS':'1','ROS_DOMAIN_ID':domain}
            execute([sys.executable,SCRIPTS/'run_m2b.py','--arm',arm,'--policy',policy,'--repeat','2','--output',root/group],env)
            status['groups'][group]={'control_runs_finished':True,'audits':{}};write(root/'execution_status.json',status)
            for i in (1,2):
                execute([sys.executable,SCRIPTS/'verify_m2b.py','--run',root/group/f'r{i}'],env)
                v=json.loads((root/group/f'r{i}/independent_validation.json').read_text());status['groups'][group]['audits'][f'r{i}']=dict(actual_full_acceptance=v['actual_full_acceptance'],prefix=v['actual_prefix_acceptance'],violations=v['violations']);write(root/'execution_status.json',status)
        status['completed']=True
    except Exception as exc:status['error']=type(exc).__name__+': '+str(exc);raise
    finally:write(root/'execution_status.json',status);log.close()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();run(a.run)
