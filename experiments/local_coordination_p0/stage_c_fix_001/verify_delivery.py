"""Read-only manifest and independent replay; no solver calls or sibling dependency."""
from fix_common import *
from accept_fix import accept
import sys

def main():
    manifest=read(HERE/'DELIVERY_MANIFEST.json');bad=[]
    for name,item in manifest['files'].items():
        f=HERE/name
        if not f.is_file() or sha(f)!=item['sha256']:bad.append(name)
    assert not bad,bad
    reports={name:accept(HERE/'results'/name) for name in ('curve','straight_acceleration')}
    assert all(v['passed'] and v['accepted_numeric_candidates']==1 and v['actual_K_executions']==1 for v in reports.values())
    print(json.dumps(dict(passed=True,files_verified=len(manifest['files']),solver_calls=0,independent=reports),indent=2))
if __name__=='__main__':main()
