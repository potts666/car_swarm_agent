"""Bounded real-DDS paired prefixes; no whole-experiment rerun or model polling."""
import contextlib
import hashlib
import json
import os
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[4]
SCRIPTS=ROOT/'examples/head_on_mpc/scripts'
sys.path.insert(0,str(SCRIPTS))
from run_m2b import online,snapshot,build_scene,build_routes,write
from verify_m2b import verify_run
from m2b_recording import lines

OUT=Path(__file__).resolve().parent


if __name__=='__main__':
    scene=build_scene();scene['source_version']=snapshot(OUT);routes=build_routes(scene)
    assert os.environ.get('OPENBLAS_NUM_THREADS')=='1'
    result={}
    for name,cache,profile in [('A_uncached',False,True),('B_cached',True,True),('B_no_profiler',True,False)]:
        directory=OUT/name
        directory.mkdir(exist_ok=False)
        write(directory/'routes.json',routes)
        print(name,'started',flush=True)
        with (directory/'execution.log').open('w') as log,contextlib.redirect_stdout(log):
            summary=online(scene,routes,directory/'r1','local','fifo-ready',cache,profile,6)
            validation=verify_run(directory/'r1')
        result[name]=dict(summary=summary,prefix_validation=validation,
                          actual_sha256=hashlib.sha256(json.dumps(lines(directory/'r1/actual.jsonl'),sort_keys=True).encode()).hexdigest())
        write(OUT/'prefix_results.json',result)
        print(name,'committed',summary['committed_cycles'],'prefix',validation['actual_prefix_acceptance'],flush=True)
        if summary['failure_layer']!='prefix_limit' or summary['committed_cycles']!=6 or not validation['actual_prefix_acceptance']:
            raise SystemExit('Preserved failed prefix; no later arm run')
    assert len({r['actual_sha256'] for r in result.values()})==1,'actual traces differ; no equivalence claim'
    print('All three actual prefixes identical; independent prefix audits passed.',flush=True)
