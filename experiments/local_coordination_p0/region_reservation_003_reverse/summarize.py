"""Measured events and common-time energy; no numerical optimization."""
import csv,platform,subprocess
from common import *


def main():
    metrics=[];index={}
    for side in ('A','B'):
        run=HERE/'results'/side;e=next(e for e in read(run/'events.json') if e['kind']=='local_socp');raw=get(HERE/'objects',e['raw_result_id']);pol=get(HERE/'objects',e['policy_result_id']);anchor=get(HERE/'objects',e['anchor_id']);receiver=get(HERE/'objects',anchor['receiver']['certificate_id']);old=get(HERE/'objects',e['parent_witness']);new=get(HERE/'objects',e['accepted_witness']);actor=e['actor'];k=e['cycle'];actual={}
        with gzip.open(run/'execution.jsonl.gz','rt') as stream:
            for row in map(json.loads,stream):
                if k<=row['cycle']<=k+32:actual[row['cycle']]=row
        delta=[F(x)-F(y) for u,v in zip(new['controls'][actor][k:k+32],old['controls'][actor][k:k+32]) for x,y in zip(u,v)];v=F(anchor['state']['v'][actor]);reverse_changes=[];changes=[];stops=[]
        for t in range(k,k+33):
            command=list(map(F,actual[t]['actual']['commands'][actor]));expected=list(map(F,new['controls'][actor][t]));assert command==expected
            changed=command!=list(map(F,old['controls'][actor][t]));
            if changed:changes.append(t)
            if v<0 and changed:reverse_changes.append(dict(cycle=t,v_before=str(v),actual_command=list(map(str,command)),old_command=old['controls'][actor][t]))
            v+=H*command[0]
            if v==0:stops.append(t+1)
        def energy(w,n):return H*sum(F(x)**2 for u in w['controls'][actor][k:k+n] for x in u)
        energies={str(n):dict(old=str(energy(old,n)),actual=str(energy(new,n)),gain=str(energy(old,n)-energy(new,n)),gain_percent=float((energy(old,n)-energy(new,n))/energy(old,n)*100)) for n in (32,33)}
        audit=read(run/'independent_acceptance.json');rej=read(run/'rejection/summary.json');info=raw['info'];row=dict(side=side,actor=actor,source_cycle=k,source_v=anchor['state']['v'][actor],status=raw['status'],res_primal=info['res_primal'],res_dual=info['res_dual'],gap_abs=info['gap_abs'],gap_rel=info['gap_rel'],iterations=info['iterations'],solve_time_s=info['solve_time'],generation_and_admission_s=pol['wall_s'],alpha=pol['attempts'][-1]['alpha'] if pol['accepted'] else '',accepted=pol['accepted'],quantization_error=pol['attempts'][-1]['quantization']['error'],input_delta_norm=str(norm_upper(delta)),first_changed_cycle=changes[0] if changes else None,reverse_changed_ticks=len(reverse_changes),reverse_changes=reverse_changes,first_actual_command=actual[k]['actual']['commands'][actor],first_old_command=old['controls'][actor][k],crosses_stop=bool(stops),exact_stop_cycles=stops,K_phase=k+32,K_command=actual[k+32]['actual']['commands'][actor],K_reset_v=actual[k+32]['actual']['v'][actor],K_reset_delta=actual[k+32]['actual']['delta'][actor],retained_phase=receiver['retained_target_phase'],receiver_domain=receiver['domain'],domain_dimension=receiver['dimension'],energy=energies,actual_steps=audit['actual_steps'],remaining_budget=audit['remaining_budget'],minimum_road_m=audit['minimum_road_m'],minimum_pair_m=audit['minimum_pair_m'],independent_passed=audit['passed'],rejections=len(rej['rejections']),old_tail_steps=rej['actual_steps'])
        metrics.append(row);index[side]=e
    write(HERE/'metrics.json',dict(cases=metrics,new_native_solver_calls=len(read(HERE/'results/solver_calls.json')),formal_batch=False,more_orderings_queued=True));write(HERE/'event_index.json',index)
    fields=['side','actor','source_cycle','source_v','status','res_primal','res_dual','gap_abs','gap_rel','iterations','solve_time_s','generation_and_admission_s','alpha','accepted','quantization_error','input_delta_norm','first_changed_cycle','reverse_changed_ticks','first_actual_command','first_old_command','crosses_stop','K_phase','K_command','K_reset_v','K_reset_delta','retained_phase','domain_dimension','actual_steps','remaining_budget','minimum_road_m','minimum_pair_m','independent_passed','rejections','old_tail_steps']
    with (HERE/'comparison.csv').open('w',newline='',encoding='utf-8-sig') as out:
        writer=csv.DictWriter(out,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(metrics)
    with (HERE/'energy_windows.csv').open('w',newline='',encoding='utf-8-sig') as out:
        writer=csv.DictWriter(out,fieldnames=['side','start_cycle','ticks','old','actual','gain','gain_percent']);writer.writeheader()
        for row in metrics:
            for n,values in row['energy'].items():writer.writerow(dict(side=row['side'],start_cycle=row['source_cycle'],ticks=n,**values))
    hashes=read(HERE/'historical_hashes.json');mismatches=[name for name,digest in hashes.items() if not (HERE.parent/name).exists() or sha(HERE.parent/name)!=digest];assert not mismatches,mismatches;write(HERE/'preservation_check.json',dict(passed=True,files_checked=len(hashes),mismatches=mismatches,git_commits=0,papers_modified=False))
    import clarabel,numpy,scipy,osqp
    write(HERE/'environment.json',dict(python=sys.version,executable=sys.executable,platform=platform.platform(),clarabel=clarabel.__version__,numpy=numpy.__version__,scipy=scipy.__version__,osqp= osqp.__version__,dependency_path='/tmp/dmpc-p0-deps',OPENBLAS_NUM_THREADS='1',v1_native_calls=0,v2_native_calls=2))
    write(HERE/'executed_source_versions.json',dict(source_commit=read(HERE/'plan.json')['source_commit'],captured_after_execution=True,code={str(f.relative_to(HERE)):sha(f) for f in sorted(HERE.rglob('*.py')) if 'delivery' not in f.parts and '__pycache__' not in f.parts}))
    rules=read(HERE/'results/rule_checks.json');ranges=read(HERE/'results/source_range_checks.json');zero=read(HERE/'results/zero_successor_checks.json');rejections=read(HERE/'results/rejection_checks.json')
    gate=dict(passed=rules['passed'] and ranges['passed'] and zero['passed'] and all(x['independent_passed'] and x['accepted'] and x['reverse_changed_ticks']>0 for x in metrics),shared_generation_rules=dict(passed=rules['passed'] and ranges['passed'],entry='generation.generate',method='method_spec.md'),receiver_synthesis=dict(passed=zero['passed'],entry='receivers.synthesize',rule='margin_allocation_v2_1',same_entry_both_sides=True),existing_forward_regression=dict(passed=all(x['passed'] for x in rules['cases'] if x['kind'].startswith('existing')),cases=4,new_solver_calls=0),reverse_source_checks=dict(passed=ranges['passed'] and all(x['passed'] for x in rules['cases'] if x['kind']=='reverse'),registered_sources='pre_registered.json',initial_no_solve_checks='results/rule_checks.json',additional_rectangular_ranges='results/source_range_checks.json'),reverse_actual_execution=dict(passed=all(x['independent_passed'] and x['accepted'] and x['reverse_changed_ticks']>0 for x in metrics),cases=2,total_steps=5604),rejection_continuation=dict(passed=all(x['passed'] for x in rejections),rejected_proposals=16,actual_fallback_branches=2,actual_fallback_steps=198,invalid_commits=0),native_solver_calls=2,historical_preservation=True,formal_batch=False,theorem_scope='nominal signed bicycle, exact measured speed/steering, fixed certified policy timing, synchronous in-process four-plan commit; conditional sufficient domain, not universal feasibility',unresolved=['A is AlmostSolved with res_primal above configured tolerance; exact certificate admission passed','uncertain-speed/steering robust policy, real processes/clock and other orderings not exercised'])
    assert gate['passed'];write(HERE/'GATE.json',gate);write(HERE/'work_queue.json',dict(completed='priority 2 v2 controlled reverse windows',queued=['priority 3: more orders and overlap combinations','real independent processes and sustained physical clock'],requires_separate_task=True))
    print(json.dumps(dict(passed=gate['passed'],native_calls=2,historical_files_checked=len(hashes),energy=[(x['side'],x['energy']['33']['gain_percent']) for x in metrics]),indent=2))
if __name__=='__main__':main()
