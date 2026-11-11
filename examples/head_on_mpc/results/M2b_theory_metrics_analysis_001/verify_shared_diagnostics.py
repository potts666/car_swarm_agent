"""Restore existing shared details and check original logical-byte digests only."""
import sys
from pathlib import Path
from analyze import OUT,ROOT,RESULTS,load,write
sys.path.append(str(ROOT/'examples/head_on_mpc/scripts'))
from m2b_diagnostic_archive import object_loader,logical_digest


if __name__=='__main__':
    root=RESULTS/'M2b_fourcar_001';loader=object_loader(root);seen=set();checks={}
    def read(key):
        seen.add(key)
        return loader(key)
    for group in ['C-FIFO','L-FIFO','L-ALT']:
        for repeat in ['r1','r2']:
            name=group+'/'+repeat;directory=root/group/repeat
            try:
                descriptor=load(directory/'diagnostic_archive.json')
                digest,count=logical_digest(directory/'diagnostics.jsonl.gz',read)
                if digest!=descriptor['original_logical_sha256'] or count!=descriptor['rows']:
                    raise ValueError('restored logical diagnostic bytes differ')
                checks[name]=dict(passed=True,rows=count,restored_sha256=digest,full_ocp_rows_rebuilt=False)
            except Exception as e:
                checks[name]=dict(passed=False,error=type(e).__name__+': '+str(e))
            print(name,'diagnostic integrity',checks[name]['passed'],flush=True)
    loader.cache_clear()
    write(OUT/'shared_diagnostic_integrity.json',dict(runs=checks,distinct_referenced_objects=len(seen),
          scope='all archived formal diagnostic rows restored/hashed; no solver or full OCP candidate check'))
