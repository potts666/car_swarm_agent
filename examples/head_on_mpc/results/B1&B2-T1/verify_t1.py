"""Artifact and regression self-checks; no optimization or timing rerun."""
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent

def check():
    provenance=json.loads((ROOT/'provenance.json').read_text())
    for relative,digest in provenance['hashes'].items():
        assert hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()==digest,relative
    for relative,digest in provenance['original_hashes'].items():
        assert hashlib.sha256((ROOT.parent/'bicycle_e3_002'/relative).read_bytes()).hexdigest()==digest,relative
    derivative=json.loads((ROOT/'derivative_results.json').read_text())
    assert len(derivative)==8
    for instance in derivative:
        assert next(x for x in instance['finite_difference'] if x['derivative']=='composed')['max_abs_fd_error']<2e-8
        for run in instance['runs']:
            assert run['committed'] is False
            for relative in run['qp_files']:
                with np.load(ROOT/relative) as qp:
                    assert qp['P'].shape[0]==len(qp['q'])
                    assert qp['A'].shape==(len(qp['l']),len(qp['q']))
                    assert np.all(qp['l']<=qp['u']+1e-12)
        if instance['instance'].endswith('_56'):
            for run in instance['runs']:
                assert run['record']['accepted']==(run['update_limit']==3)
                assert run['record']['qp_updates']==(2 if run['update_limit']==3 else 1)
    parking=json.loads((ROOT/'parking_results.json').read_text())
    for cycle in range(117,122):
        for mode in ('cold','shift_primal','shift_primal_dual','no_row_scaling'):
            runs=[x for x in parking if x['cycle']==cycle and x['mode']==mode]
            if not runs:assert cycle==117 and mode.startswith('shift');continue
            assert len(runs)==3
            assert len({(x['status'],x['iterations']) for x in runs})==1
            for x in runs:
                assert x['committed'] is False
                assert x['end_monotonic_s']>=x['start_monotonic_s']
                if x['accepted']:
                    assert x['physical_constraint_violation']<=1e-5
                    assert x['terminal_equation_error']<=1e-5
    failure=[x for x in parking if x['cycle']==121]
    assert all(not x['accepted'] and x['iterations']==20000 for x in failure if x['mode']=='cold')
    for mode in ('shift_primal','shift_primal_dual'):
        run=next(x for x in failure if x['mode']==mode and x['repeat']==0)
        assert run['accepted']
        assert run['replay']['downstream_bicycle_original_3_updates']['record']['accepted']
    result=dict(passed=True,derivative_instances=len(derivative),derivative_candidate_runs=sum(len(x['runs']) for x in derivative),
        parking_runs=len(parking),frozen_state_snapshots=len(list((ROOT/'frozen').glob('B2_*.json'))),
        checks=['original/frozen/runner SHA256','composed derivative finite differences','frozen QP dimensions/bounds',
                'cycle56 preserved rejection and two-update recovery','deterministic parking repetitions',
                'cycle121 cold rejection and warm-start physical constraints/downstream nonlinear replay'])
    (ROOT/'verification.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))

if __name__=='__main__':check()
