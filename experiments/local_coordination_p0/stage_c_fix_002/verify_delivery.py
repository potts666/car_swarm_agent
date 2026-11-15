"""Read-only independent verification from an extracted distribution; zero solver calls."""
from fix_common import *
from accept_fix import accept


def main():
    manifest=read(HERE/'DELIVERY_MANIFEST.json')
    for name,item in manifest['files'].items():assert sha(HERE/name)==item['sha256'],name
    for call in read(HERE/'results/solver_calls.json'):
        raw=get(HERE/'objects',call['raw_result_id']);assert raw['source_code_sha256']==sha(HERE/'fix_socp.py')
    audits={name:accept(HERE/'results'/name) for name in ('curve','straight_acceleration')}
    assert all(a['passed'] and a['accepted_numeric_candidates']==2 and a['damage_forks']['passed'] for a in audits.values())
    print(json.dumps(dict(passed=True,files_verified=len(manifest['files']),solver_calls=0,independent=audits),indent=2))
if __name__=='__main__':main()
