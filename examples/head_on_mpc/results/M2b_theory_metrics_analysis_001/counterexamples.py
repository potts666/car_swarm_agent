"""Identify the existing natural failures and inspect only their final diagnostic rows."""
from pathlib import Path
import sys
from analyze import OUT,ROOT,RESULTS,load,write
from analysis_io import stream,physical,file_sha

sys.path.append(str(ROOT/'examples/head_on_mpc/scripts'))
from m2b_diagnostic_archive import object_loader,expand


if __name__=='__main__':
    records={}
    for name in ['debug_v1','debug_v2','debug_c1']:
        directory=RESULTS/'M2b_fourcar_001'/name/'r1';summary=load(directory/'summary.json');target=summary['timed_cycles']-1
        solve=next(r for r in stream(directory/'solves.jsonl') if r['cycle']==target)
        local=solve.get('local') or {};q=solve.get('central') or next(r for r in local.values() if r and not r['accepted'])
        reference=q['reference_record']
        item=dict(cycle=target,summary_failure=summary['failure'],failure_layer=q.get('failure_layer'),accepted=q['accepted'],
                  total_predict_s=q['solve_wall_s'],deadline_s=q.get('deadline_s'),deadline_exceeded=q.get('deadline_exceeded'),
                  reference_status=reference['message'],reference_accepted=reference['accepted'],reference_iterations=reference['iterations'],
                  bicycle_statuses=[p['message'] for p in q['subproblems']],bicycle_accepted=[p['accepted'] for p in q['subproblems']],
                  last_recorded_speed_m_s={c:z[3] for c,z in summary['last_states'].items()},source_version=load(directory/'config.json').get('source_version'),solve_log_sha256=file_sha(physical(directory/'solves.jsonl')))
        item['all_stopped_at_original_terminal_threshold']=all(abs(z[3])<=load(directory/'config.json')['scene']['terminal']['speed_m_s'] for z in summary['last_states'].values())
        path=directory/'diagnostics.jsonl'
        try:
            selected=next(r for r in stream(path) if r['cycle']==target)
            restored=expand(selected,object_loader(RESULTS/'M2b_fourcar_001'))
            item['diagnostic_body_restored']=True
            item['diagnostic_log_sha256']=file_sha(physical(path))
        except (FileNotFoundError,StopIteration) as e:
            item['diagnostic_body_restored']=False;item['diagnostic_missing']=str(e) or 'failure cycle not retained'
        records[name]=item
    write(OUT/'natural_counterexamples.json',records)
    print('Three natural failure identities/records checked; missing diagnostics explicit.')
