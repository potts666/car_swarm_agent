"""Two actual sources, overlapping K replacement, inherited-prefix and source integrity."""
from common import *
from local_update import data_for
from socp_rows import rows_for,evaluate


def main():
    records=[]
    for case in ('opposite','same_direction'):
        folder=HERE/'results'/case;summary=read(folder/'summary.json');audit=read(folder/'independent_acceptance.json');events=[e for e in read(folder/'events.json') if e['kind']=='local_socp'];assert len(events)==2 and all(e['accepted'] for e in events)
        with gzip.open(folder/'execution.jsonl.gz','rt') as stream:rows=list(map(json.loads,stream))
        first,second=events;anchor1=get(HERE/'objects',first['anchor_id']);anchor2=get(HERE/'objects',second['anchor_id']);actor=second['actor'];source=anchor2['state'];assert source['cycle']==788 and source['witness']==first['accepted_witness'] and source['history']==rows[787]['actual']['history']
        assert source['poses_Q']==rows[787]['actual']['poses_Q'] and source['v']==rows[787]['actual']['v'] and source['delta']==rows[787]['actual']['delta']
        assert anchor1['state']['poses_Q']['A1']!=source['poses_Q']['A1'] and anchor1['state']['poses_Q'][actor]!=source['poses_Q'][actor]
        assert anchor1['peer_horizon_id']!=anchor2['peer_horizon_id'] and first['assembly_id']!=second['assembly_id'] and first['derivative_id']!=second['derivative_id']
        source_movers=[c for c in CARS if F(source['v'][c])!=0];assert actor in source_movers and 'A1' in source_movers
        zero=[]
        for e in events:
            a=get(HERE/'objects',e['anchor_id']);cert=get(HERE/'objects',e['derivative_id']);result=evaluate(rows_for(data_for(actor,e['cycle']+32),m,a,cert,'B-JL'),[F(0)]*64);assert result['passed'];zero.append(dict(phase=e['cycle'],inherited_prefix_in_new_rows=True))
        old=get(HERE/'objects',first['accepted_witness']);new=get(HERE/'objects',second['accepted_witness']);assert [actor,'G_in'] not in new['events'].get('819',[]) and [actor,'G_in_shifted'] in new['events']['820']
        assert new['controls'][actor][:788]==old['controls'][actor][:788] and new['controls'][actor][821:]==old['controls'][actor][821:]
        for c in CARS:
            if c!=actor:assert new['controls'][c]==old['controls'][c]
        assert rows[820]['actual']['v'][actor]=='1/2' and rows[820]['actual']['delta'][actor]=='0'
        assert [actor,'G_in_shifted'] in rows[1381]['token']['context']['receiver_chain']['820']
        assert all(row['budget_after']==row['budget_before']-1 for row in rows) and summary['remaining_budget']==0 and audit['continuous_two_updates_accepted']
        damages=read(folder/'second_round_rejections/summary.json');assert damages['passed'] and len(damages['cases'])==6 and all(x['invalid_commits']==0 and x['actual_steps']==33 and x['actual_old_K_executed'] for x in damages['cases'])
        records.append(dict(case=case,passed=True,two_actual_sources=True,both_source_bodies_moving=source_movers,source_state_matches_actual_first_step=True,zero_anchor_checks=zero,second_receiver_phase=820,second_K_phase=820,first_K_phase=819,actual_second_K_exact_reset=True,subsequent_choices_keep_shifted_receiver=True,rejected_then_same_old_tail_cases=6))
    for name,digest in read(HERE/'executed_source_versions.json').items():assert sha(HERE/name)==digest,name
    hist=read(HERE/'historical_hashes.json');assert all(sha(HERE.parent/name)==digest for name,digest in hist.items())
    calls=read(HERE/'results/solver_calls.json');assert len(calls)==4 and all(c['completed'] for c in calls)
    write(HERE/'self_checks.json',dict(passed=True,cases=records,solver_calls=4,historical_files_unchanged=len(hist),frozen_source_versions_unchanged=True));print('Both actual two-moving sources, zero anchors, K replacement and all integrity checks passed')
if __name__=='__main__':main()
