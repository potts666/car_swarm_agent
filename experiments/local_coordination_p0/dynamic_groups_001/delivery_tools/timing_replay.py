"""Zero-call protocol component timings, explicitly separated from original cycles."""
import sys,time,csv
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(ROOT))
from common import *
from protocol import prepare,commit,select_for_execution,edge_cover
from execution import advance
rows=[]
for arm in ('F-S','F-P','D-P'):
    for e in read(ROOT/'results'/arm/'summary.json')['events']:
        s=restore(m,e['source']);versions={c:0 for c in CARS};candidates={}
        for t in e['tokens']:versions.update(t['request']['versions']);candidates.update(t['request']['plans'])
        contract=get(ROOT/'objects',s['contract_id']);start=time.perf_counter();edge_cover(contract);cross=time.perf_counter()-start;requests=[];start=time.perf_counter()
        for t in e['tokens']:requests.append(prepare(s,candidates,t['request']['participants'],s['contract_id'],versions))
        prepare_s=time.perf_counter()-start;start=time.perf_counter();tokens=[commit(s,q,versions) for q in requests];commit_s=time.perf_counter()-start;start=time.perf_counter();wid,chosen=select_for_execution(s,tokens,versions);select_s=time.perf_counter()-start;assert wid==e['selected_witness'];s['contract_id']=None;start=time.perf_counter();advance(s,wid);execute_s=time.perf_counter()-start
        rows.append(dict(arm=arm,round=e['round'],measurement='zero-call replay; not original online cycle',group_prepare_s=prepare_s,cross_group_interface_identity_s=cross,commit_s=commit_s,execution_selection_s=select_s,physical_execution_s=execute_s,original_contract_construction_s=contract['wall_s'],original_derivatives_s=sum(get(ROOT/'objects',j['derivative_id'])['build_wall_s'] for j in e['jobs']),original_cone_assembly_s=sum(j['assembly_s'] for j in e['jobs']),original_local_certification_s=sum(a['admission_s'] for a in e['admissions']),original_full_combination_audit_s=e['timing']['full_audit_s']))
with (ROOT/'timing_components.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
print(dict(passed=True,component_replays=len(rows),new_calls=0),flush=True)
