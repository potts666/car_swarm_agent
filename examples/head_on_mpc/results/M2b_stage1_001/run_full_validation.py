"""One optimized FIFO run, bounded watchdog, then independent audit and resource summary."""
import contextlib
import json
import os
from pathlib import Path
import resource
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'examples/head_on_mpc/scripts'))
from run_m2b import online,snapshot,build_scene,build_routes,write
from verify_m2b import verify_run
OUT=Path(__file__).resolve().parent


if __name__=='__main__':
    directory=OUT/'full_L_FIFO';directory.mkdir(exist_ok=False)
    scene=build_scene();scene['source_version']=snapshot(OUT);routes=build_routes(scene)
    write(directory/'routes.json',routes)
    status=dict(stage='running',started_unix=time.time(),pid=os.getpid(),detailed_timing=False,validation_cache=True)
    write(OUT/'full_status.json',status)
    stopped=threading.Event();samples=[]
    def sample():
        while not stopped.is_set():
            values={}
            for line in Path('/proc/self/status').read_text().splitlines():
                k,_,v=line.partition(':')
                if k in ('VmRSS','Threads'):values[k]=int(v.split()[0])
            samples.append(dict(time=time.time(),loadavg=os.getloadavg(),**values))
            stopped.wait(1)
    monitor=threading.Thread(target=sample,daemon=True);monitor.start()
    before=resource.getrusage(resource.RUSAGE_SELF)
    try:
        print('Full optimized L-FIFO started; automatic independent audit follows.',flush=True)
        summary=online(scene,routes,directory/'r1','local','fifo-ready',True,False)
        status['stage']='independent_audit';write(OUT/'full_status.json',status)
        audit=verify_run(directory/'r1')
        status.update(stage='complete',summary=summary,actual_full_acceptance=audit['actual_full_acceptance'],violations=audit['violations'])
    except Exception as exc:
        status.update(stage='failed',error=type(exc).__name__+': '+str(exc))
        raise
    finally:
        stopped.set();monitor.join(timeout=2)
        after=resource.getrusage(resource.RUSAGE_SELF)
        status.update(ended_unix=time.time(),resource=dict(user_cpu_s=after.ru_utime-before.ru_utime,system_cpu_s=after.ru_stime-before.ru_stime,
            max_rss_kib=after.ru_maxrss,max_observed_threads=max((r['Threads'] for r in samples),default=None),
            scope='whole driver incl online/export/independent audit; sampler thread included',samples=samples))
        write(OUT/'full_status.json',status)
