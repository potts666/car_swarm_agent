"""Small timed probes after the complete run; count OSQP setup/solve separately."""
import contextlib
from pathlib import Path
import sys
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'examples/head_on_mpc/scripts'))
import osqp
from run_m2b import online,snapshot,build_scene,build_routes,write
from m2b_timing import timed
from verify_m2b import verify_run
OUT=Path(__file__).resolve().parent


if __name__=='__main__':
    scene=build_scene();scene['source_version']=snapshot(OUT);routes=build_routes(scene)
    for name,cache in [('phase_probe_uncached',False),('phase_probe_cached',True)]:
        directory=OUT/name;directory.mkdir(exist_ok=False);write(directory/'routes.json',routes)
        print(name,'started',flush=True)
        with (directory/'execution.log').open('w') as log,contextlib.redirect_stdout(log),contextlib.ExitStack() as stack:
            stack.enter_context(patch.object(osqp.OSQP,'setup',timed('osqp_setup')(osqp.OSQP.setup)))
            stack.enter_context(patch.object(osqp.OSQP,'solve',timed('osqp_solve')(osqp.OSQP.solve)))
            s=online(scene,routes,directory/'r1','local','fifo-ready',cache,True,2)
            v=verify_run(directory/'r1')
        if s['failure_layer']!='prefix_limit' or s['committed_cycles']!=2 or not v['actual_prefix_acceptance']:
            raise SystemExit('Failed phase probe retained')
        print(name,'two cycles passed independent prefix audit',flush=True)
