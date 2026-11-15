"""Report two real reoptimizations without double-counting overlapping predictions."""
from common import *
import csv,difflib,sys,platform
import numpy,scipy,clarabel,osqp


def main():
    assert read(HERE/'self_checks.json')['passed'] and read(HERE/'second_round_rejection_checks.json')['passed'] and read(HERE/'shifted_fixture_checks.json')['passed']
    table=[];metrics={};index=[];checkpoints={}
    for case in ('opposite','same_direction'):
        folder=HERE/'results'/case;s=read(folder/'summary.json');a=read(folder/'independent_acceptance.json');assert s['passed'] and a['passed'] and a['continuous_two_updates_accepted']
        with gzip.open(folder/'execution.jsonl.gz','rt') as stream:logs=list(map(json.loads,stream))
        events=[e for e in read(folder/'events.json') if e['kind']=='local_socp'];assert len(events)==2
        case_rows=[]
        for e in events:
            raw=get(HERE/'objects',e['raw_result_id']);p=get(HERE/'objects',e['policy_result_id']);old=get(HERE/'objects',e['parent_witness']);new=get(HERE/'objects',e['accepted_witness']);actor=e['actor'];k=e['cycle'];info=raw['info'];attempt=p['attempts'][-1]
            Eold=F(p['old_energy']);Enew=F(attempt['energy']);delta=list(map(F,attempt['delta']));first_changed=logs[k]['actual']['commands'][actor]!=old['controls'][actor][k]
            row=dict(case=case,round=e['round'],cycle=k,actor=actor,status=raw['status'],primal=info['res_primal'],dual=info['res_dual'],gap_abs=info['gap_abs'],gap_rel=info['gap_rel'],iterations=info['iterations'],solver_s=info['solve_time'],pipeline_s=p['wall_s'],accepted=p['accepted'],actual_first_changed=first_changed,old_first=';'.join(old['controls'][actor][k]),actual_first=';'.join(logs[k]['actual']['commands'][actor]),delta_norm_upper=str(norm_upper(delta)),alpha=attempt['alpha'],old_prefix_energy=str(Eold),new_prefix_energy=str(Enew),predicted_improvement_percent=float(100*(Eold-Enew)/Eold),source_history=raw['history'],source_witness=raw['source_witness'],anchor_id=e['anchor_id'],derivative_id=e['derivative_id'],assembly_id=e['assembly_id'],raw_result_id=e['raw_result_id'],policy_result_id=e['policy_result_id'],commit_id=e['commit_id'],receiver_phase=k+32,K_phase=k+32,receiver_kind=new['receiver_contract']['kind'],budget=2802-k,budget_after=2801-k,selected_prefix_ticks=1 if e['round']==1 else 32)
            assert e['commit_id']==logs[k]['token']['commit_id'];table.append(row);case_rows.append(row);index.append(e)
        actor=events[0]['actor'];baseline=get(HERE/'objects',events[0]['parent_witness']);E0=H*sum(F(x)**2 for u in baseline['controls'][actor][787:821] for x in u);Ea=H*sum(F(x)**2 for r in logs[787:821] for x in r['actual']['commands'][actor]);changed=[r['cycle'] for r in logs[787:821] if r['actual']['commands'][actor]!=baseline['controls'][actor][r['cycle']]]
        metrics[case]=dict(passed=True,actual_steps=s['actual_steps'],logical_task_time_s=float(F(s['actual_task_time_s'])),initial_budget=2802,remaining_budget=s['remaining_budget'],numerical_proposals=2,accepted_numerical_proposals=2,rounds=case_rows,actual_union_energy=dict(start=787,stop_exclusive=821,ticks=34,old_energy=str(E0),actual_energy=str(Ea),improvement_percent=float(100*(E0-Ea)/E0),changed_cycles=changed),actual_K_phase=820,old_K_superseded_phase=819,independent_road_lower_m=a['minimum_road_m'],independent_pair_lower_m=a['minimum_pair_m'],overlap_ticks=a['actual_overlap_ticks'],controller_wall_s=s['wall_s'],independent_wall_s=a['wall_s'],damaged_second_proposals_rejected=6,actual_old_tail_steps_per_rejection=33,independent_passed=True)
        names=('config.json','events.json','summary.json','execution.jsonl.gz','independent_acceptance.json','second_round_rejections/summary.json')
        checkpoints[case]=dict(passed=True,full_replay_completed=True,files={str((folder/name).relative_to(HERE)):sha(folder/name) for name in names},code={name:sha(HERE/name) for name in ('common.py','physics.py','accept_case.py','derivatives.py','conic.py','socp_rows.py','receivers.py','inputs/interval_source.py','sources/certify_interfaces.py')})
    write(HERE/'event_index.json',index);write(HERE/'metrics.json',metrics);write(HERE/'results/independent_checkpoints.json',checkpoints)
    with (HERE/'comparison.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(table[0]));writer.writeheader();writer.writerows(table)
    (HERE/'comparison_windows.csv').write_bytes(b'\xef\xbb\xbf'+(HERE/'comparison.csv').read_bytes())
    proof=read(HERE/'shifted_receiver_certificate.json');preserved=read(HERE/'historical_hashes.json');assert all(sha(HERE.parent/name)==digest for name,digest in preserved.items());write(HERE/'preservation_check.json',dict(passed=True,historical_files_unchanged=len(preserved)))
    calls=read(HERE/'results/solver_calls.json');assert len(calls)==4 and all(c['completed'] for c in calls)
    write(HERE/'versions.json',dict(python=sys.version,platform=platform.platform(),clarabel=clarabel.__version__,numpy=numpy.__version__,scipy=scipy.__version__,osqp_legacy_import=osqp.__version__))
    (HERE/'requirements.txt').write_text(f'clarabel=={clarabel.__version__}\nnumpy=={numpy.__version__}\nscipy=={scipy.__version__}\nosqp=={osqp.__version__}\n')
    gate=dict(version='region_reservation_002',passed=True,actual_source_update=True,continuous_two_round_acceptance=True,continuous_safety_and_completion=True,rejection_and_same_old_tail=True,solver_calls=4,extra_solver_calls=0,solver_statuses=[r['status'] for r in table],strict_Solved_all=False,policy='certificate_suboptimal_v1',cases=metrics,shifted_receiver_proof=proof['certificate_id'],initial_budget=2802,final_budget=0,history_preserved=True,papers_changed=False,formal_batch=False,priority_queue=dict(item1='completed: two actual overlapping-car reoptimizations',item2='pending: reverse SOCP',item3='pending: more order/overlap combinations'),limits=['Two actual anchors per case; not a guarantee that every future SOCP is feasible','One local 64-input vehicle, other moving plans frozen and time aligned','Nominal synchronous ZOH model, not hard real-time/async/disturbance validation','Three AlmostSolved proposals admitted by complete certificate; no strict numerical optimality claim'])
    write(HERE/'GATE.json',gate)
    rt='\n'.join(f"|{r['case']}|{r['round']} / {r['cycle']}|{r['status']}|{r['primal']:.3g}|{r['gap_abs']:.3g}|{r['alpha']}|{r['predicted_improvement_percent']:.6f}%|{r['actual_first_changed']}|" for r in table)
    mt='\n'.join(f"|{case}|{v['actual_steps']}|{v['remaining_budget']}|{v['independent_road_lower_m']:.6f}|{v['independent_pair_lower_m']:.6f}|{v['actual_union_energy']['improvement_percent']:.6f}%|" for case,v in metrics.items())
    report=f'''# 两车重叠期间连续两拍实际重新优化

用户当前最关键的问题已在两个用例通过：两车同时运动时，cycle787求解并执行首拍后，cycle788从实际联合状态再求解，第二份新候选共同受理并继续合法执行。两条完整任务均2802拍、四车到达目标、预算归零。后移接收域已先做数学与非零状态认证，未把原Gin中心直接复用到下一拍。

这是优先级1的隔离验证，保存于region_reservation_002。用户指定队列为1→2→3；倒车SOCP与更多通行/重叠组合在本轮仍排队，不作已完成声明。历史与论文保留，未启动正式批量或Git提交。

## 理论条件和新增接收域

依据原region_reservation_001的安全组合、可执行后续与预算归纳，以及certificate_suboptimal_v1。本轮新增Gin_shifted中心为局部(-21.9,0)、航向中心0：位置L1半径0.021m，航向半径0.000238rad，速度[0.495,0.505]m/s，轮角±1e-5rad。K为a=(0.5-v)/h、omega=-delta/h，分别满足|a|≤0.025、|omega|≤5e-5。

K后航向界{float(F(proof['after_K_yaw'])):.12f}rad，保存的直行/四拍制动后位置L1界{float(F(proof['G_out_position_l1_upper'])):.12f}m，小于原Gout的0.035m；位置余量约{float(F(7,200)-F(proof['G_out_position_l1_upper'])):.12f}m，航向小于0.00024rad。561拍剩余控制按真实对象核对，K后的v=0.5、delta=0精确复位。A/B两个非零边界状态的辅助561拍8子段回放也通过。

新域并不宣称覆盖原Gin所有最坏误差经过K后的全部流像。本次两个实际第二轮源点的继承32拍前缀都通过新原行的零增量检查；连续SOCP可行性的结论限于已验证锚点。一般情况下即使新域问题不可行，仍保留已认可真实尾段，递归控制可行性不以任意未来SOCP必可解为前提。

数学对象见shifted_receiver_certificate.json、domain_checks.json、shifted_fixture_checks.json；原地图、|x|≤16资源、0.05m+量化余量车身管和既有量化/积分证明不变。

## 实现与共同提交

两用例都保留原初态、55/101/562/95拍备份时序、选择函数、目标权重、h=.2、N=32、64输入/65变量、rho=.001、Q预留、scale=1e-4、Clarabel0.11.1、三个1e-10容差、300迭代、5秒预算。

每个用例的新调用链为实际源状态/history→fresh32拍导数和同步邻车预测→原行和组装对象→一次新求解→量化/完整前缀/K/后续认证→四份计划ACK→首拍实际输出→从该后继再次走同一链。第一轮也重新求解，未复用旧probe数值结果；两用例共4次新调用。

receivers.py登记两处接收相位：第一轮819，第二轮820。第二轮32拍包含旧K819，受理时整体替换旧Gin819及其K，把接收/K一起更新到820，保留821之后的真实尾段。拒绝时保留第一轮Gin819、旧K及全部旧控制。witness.py同时保留后续换权时已发生的接收链，防止未来选择重新写回旧Gin义务。

source绑定包括全四车实际位姿/v/delta、history、当前见证、实际移位父ID、邻车预测hash、anchor/derivative/assembly/raw/policy与提交编号。第二轮旧车A1和优化车的实际位姿均已推进，邻车预测也按下一时间片重建。资源权限仍同版更新，旧车出口动作继续执行。

## 逐轮数值和实际输出

|用例|轮 / cycle|状态|原始残差|绝对gap|alpha|本轮32拍预测输入代价改善|实际首拍改变|
|---|---|---|---:|---:|---:|---:|---|
{rt}

相反方向第一轮Solved，其余三次AlmostSolved；未放宽严格收敛标准。所有采用均由独立版本的完整证书规则决定。精确输入、范数、对偶残差/相对gap、完整设置与原始x/s/z、日志、source/模型对象都已保存。实际首拍相同的情况照实列出，方案后续修改仍按实际命令核对。

第一轮只执行首拍，其余预测窗被第二轮替换。因此逐轮32拍收益不相加；实际输入代价按同一[787,821)的34拍（首轮首拍+次轮32拍+新K）对原备份重算：

|用例|实际拍数|剩余预算|独立道路下界m|独立六对净空下界m|34拍实际输入代价改善|
|---|---:|---:|---:|---:|---:|
{mt}

固定任务时长仍560.4s，无缩时结论。各用例194拍存在释放后的两车实际位姿运动重叠。数值求解时间、构造/认证流水线和完整控制/独立重放墙钟分别见comparison.csv和metrics.json；逻辑h=.2不是硬实时保证。

## 独立验收和拒绝续行

两个完整2802拍8子段独立验收均通过。入口逐事件读取内容ID、重建实际第二轮源状态和导数，独立复算终端Taylor行、raw→有理量化、四份公开计划/权限、连续道路和全部六对净空、域/K、预算与停车目标；可共享区间算术、传播与变分生成器，未把控制器提交标志作为安全证据。

第二轮复制分支每用例6类：陈旧anchor、陈旧derivative、旧phase、陈旧邻车位姿、损坏K、混合权限版本。12份非法提案均零错误提交；随后从同一实际源状态执行第一轮旧见证33拍，包含旧Gin819与旧K819，v/delta精确复位并由独立8子段核对。分支不计入主任务时长/求解数。

self_checks.json还核对第二轮源状态等于首拍实际后继、两车实际运动、零增量继承前缀、新K820、821后旧尾段、后续选择保留后移接收链和每拍预算下降。首次启动缺source_commit元数据在0次求解前修正，原失败日志保留于results/development_failures；没有追加数值求解。

## 交付与复核

报告、GATE.json、comparison.csv、event_index.json、metrics.json、完整最小补丁、版本与对象及每个case的原始控制和独立报告都保存在本目录。冻结源码身份见executed_source_versions.json，历史哈希见preservation_check.json。默认verify_delivery复用内容核对的完整独立验收checkpoint，并新复算两轮source/导数/33拍前缀；--full-replay才重跑两条完整独立轨迹。

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 run_stage.py
OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py
# 完整独立重算（不调用优化器）：
OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py --full-replay
```

尚未完成队列2/3；也未声明任意未来拍持续SOCP可行、两车同时独立求解、持续物理时钟、扰动或异步进程保证。一般优先/公平规则保持待设计。本轮明确回答的是两个实际重叠用例的“执行首拍以后，能否再次优化并合法接续”。
'''
    (HERE/'report.md').write_text(report)
    write(HERE/'work_queue.json',dict(order=[1,2,3],item1=dict(status='complete',gate='GATE.json'),item2=dict(status='pending',task='certify signed-speed derivative/physical rows and reverse SOCP actual window'),item3=dict(status='pending',task='bounded additional certified order/overlap cases'),formal_batch=False))
    previous=HERE.parent/'region_reservation_001';patch=[]
    for f in sorted(HERE.glob('*.py')):
        old=previous/f.name;before=old.read_text().splitlines(True) if old.exists() else []
        if before==f.read_text().splitlines(True):continue
        patch+=list(difflib.unified_diff(before,f.read_text().splitlines(True),fromfile='region_reservation_001/'+f.name if before else '/dev/null',tofile='region_reservation_002/'+f.name))
    (HERE/'changes.patch').write_text(''.join(patch))
    (HERE/'HANDOFF.md').write_text('目标：用户队列1→2→3，本轮先完成最关键的两车重叠连续两拍优化。\n状态：2用例×2新求解全部采用；新接收820/K820，两个2802拍独立验收、12个拒绝旧尾续行通过。\n限制：名义同步模型；历史和论文保留；不做批量。\n源码与证据：executed_source_versions.json、shifted_receiver_certificate.json、GATE.json、results/*/independent_acceptance.json。\n恢复：OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 run_stage.py\n复核：OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py\n后续：work_queue.json，优先2倒车SOCP，再3更多组合。\n')
    print(json.dumps(dict(passed=True,metrics=metrics),indent=2))
if __name__=='__main__':main()
