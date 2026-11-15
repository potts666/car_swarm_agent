"""One actual source/contract -> rows, derivatives and receiving continuation."""
import time
from common import *
from rules import contracts_for
from receivers import synthesize
from derivatives import build,verify
from socp_rows import rows_for,evaluate
CFG=dict(roads=ROADS,h=H,L=F(27,10),body=[[-1,F(7,2)],[-F(9,10),F(9,10)]],v_bounds=[-1,2],delta_abs=F(1,2),a_abs=1,omega_abs=1)

def anchor_for(state,actor,row=None):
    row=synthesize(state,actor) if row is None else row
    w=get(HERE/'objects',state['witness']);k=state['cycle']
    return normal(dict(contract_id=state['contract_id'],dependencies=get(HERE/'objects',state['contract_id'])['edge_ids'],state=state_record(state),model_sha256=sha(MODEL_PATH),physics_sha256=content_id(CFG),actor=actor,coordinate_branch='explicit fixed model frame; signed longitudinal velocity',direction_sign=direction(actor),m=32,rho=str(RHO),phase=k,receiver_phase=k+32,receiver_row_sha256=content_id(row),receiver=row,contracts=contracts_for(state,actor),witness=state['witness'],permission_owner=state['owner'],peer_horizon_id=content_id({c:w['controls'][c][k:k+33] for c in CARS if c!=actor}),plan=dict(phase=k,remaining=w['controls'][actor][k:],budget=w['ticks']-k,witness=state['witness'])))

def data_for(anchor):
    row=anchor['receiver'];return SimpleNamespace(cfg=CFG,cert={'car':anchor['actor'],'source':0},row=lambda k:row if k==row['phase'] else (_ for _ in ()).throw(ValueError('wrong receiving phase')))

def generate(state,actor,subdivisions=32):
    began=time.perf_counter()
    try:
        row=synthesize(state,actor);anchor=anchor_for(state,actor,row);aid=put(HERE/'objects',anchor);data=data_for(anchor);cert=build(data,m,anchor,subdivisions);verify(data,m,cert,anchor,False);did=put(HERE/'objects',cert);rows=rows_for(data,m,anchor,cert,'B-JL');zero=evaluate(rows,[F(0)]*64);rows_id=put(HERE/'objects',rows)
        if not zero['passed']:raise ValueError('zero_anchor: '+','.join(zero['violations']))
        value=dict(passed=True,actor=actor,phase=state['cycle'],source_history=state['history'],old_witness=state['witness'],anchor_id=aid,derivative_id=did,physical_rows_id=rows_id,receiver=row,zero_anchor=zero,budget=anchor['plan']['budget'],wall_s=time.perf_counter()-began)
    except (AssertionError,ValueError,KeyError) as error:
        value=dict(passed=False,actor=actor,phase=state['cycle'],source=state_record(state),old_witness=state['witness'],failure=repr(error),wall_s=time.perf_counter()-began)
    key=put(HERE/'objects',value);return value,key
