"""Five-layer measured report; separate uniform proof, logical time and wall time."""
from common import *
import csv,difflib,platform
import clarabel,numpy,scipy,osqp


def main():
    assert read(HERE/'self_checks.json')['passed'] and read(HERE/'feedback_checks.json')['passed'] and read(HERE/'native_rejection_checks.json')['passed']
    quant=read(HERE/'uniform_quantization_check.json');domains=read(HERE/'domain_checks.json');assert quant['passed'] and read(HERE/'integration_bounds.json')['passed'] and read(HERE/'flow_checks.json')['passed']
    bundle=dict(model_sha256=sha(MODEL_PATH),analytic_geometry_sha256=sha(HERE/'inputs/geometry_certificate.json'),domain_certificate_id=domains['certificate_id'],uniform_quantization_certificate_id=quant['certificate_id'],rational_inputs_id=content_id(read(HERE/'rational_inputs.json')),flow_images_sha256=sha(HERE/'flow_images.json'),integration_bounds_sha256=sha(HERE/'integration_bounds.json'))
    write(HERE/'CERTIFICATE_BUNDLE.json',dict(certificate_id=put(HERE/'objects',bundle),**bundle))
    comparison=[];metrics={};events=[];checkpoints={}
    for case in ('opposite','same_direction'):
        run=HERE/'results'/case;s=read(run/'summary.json');a=read(run/'independent_acceptance.json');assert s['passed'] and a['passed'];ev=read(run/'events.json');event=next(e for e in ev if e['kind']=='local_socp');raw=get(HERE/'objects',event['raw_result_id']);pol=get(HERE/'objects',event['policy_result_id']);old=get(HERE/'objects',event['parent_witness']);new=get(HERE/'objects',event['accepted_witness']);actor=event['actor'];k=event['cycle'];info=raw['info'];firstpair=['A1','B1' if case=='opposite' else 'A2']
        with gzip.open(run/'execution.jsonl.gz','rt') as stream:records=list(map(json.loads,stream))
        target_overlap=[r['cycle'] for r in records if 718<=r['cycle']<813 and all(c in r['actual']['bounds']['active'] for c in firstpair)]
        assert target_overlap and all(c in records[k]['actual']['bounds']['active'] for c in firstpair)
        Eold=H*sum(F(x)**2 for u in old['controls'][actor][k:k+33] for x in u);Eactual=H*sum(F(x)**2 for r in records[k:k+33] for x in r['actual']['commands'][actor]);assert Eactual==H*sum(F(x)**2 for u in new['controls'][actor][k:k+33] for x in u)
        actual_changed_phases=[j for j in range(k,k+33) if records[j]['actual']['commands'][actor]!=old['controls'][actor][j]];assert actual_changed_phases
        E32old=F(pol['old_energy']);E32new=F(pol['attempts'][-1]['energy']);delta=list(map(F,pol['attempts'][-1]['delta']));firstold=old['controls'][actor][k];firstnew=records[k]['actual']['commands'][actor]
        native=dict(case=case,actor=actor,cycle=k,status=raw['status'],primal=info['res_primal'],dual=info['res_dual'],gap_abs=info['gap_abs'],gap_rel=info['gap_rel'],iterations=info['iterations'],solver_s=info['solve_time'],pipeline_s=pol['wall_s'],accepted=pol['accepted'],changed=pol['changed'],actual_first_changed=firstnew!=firstold,first_actual_changed_cycle=min(actual_changed_phases),actual_changed_steps=len(actual_changed_phases),strict_failed=';'.join(key for key,ok in dict(primal=info['res_primal']<1e-10,dual=info['res_dual']<1e-10,gap=info['gap_abs']<1e-10 or info['gap_rel']<1e-10,ktratio=info['ktratio']<=1).items() if not ok),delta_norm_upper=str(norm_upper(delta)),alpha=pol['attempts'][-1]['alpha'],predicted_32_energy_improvement_percent=float(100*(E32old-E32new)/E32old),actual_33_energy_improvement_percent=float(100*(Eold-Eactual)/Eold),actual_energy_interval=f'[{k},{k+33})',raw_result_id=event['raw_result_id'],policy_result_id=event['policy_result_id'])
        comparison.append(native);events.extend(ev)
        with (run/'trajectory_display.csv').open('w',encoding='utf-8-sig',newline='') as f:
            fields=['cycle_after','logical_time_s','car','x_mid_m','y_mid_m','physical_yaw_mid_rad','v_rational','delta_rational','a_rational','omega_rational','owner','permission_version','action','x_lo_Q','x_hi_Q','y_lo_Q','y_hi_Q','theta_lo_Q','theta_hi_Q'];writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
            for r in records:
                for c in CARS:
                    pp=r['actual']['poses_Q'][c];ctx=r['token']['context'];writer.writerow(dict(cycle_after=r['cycle']+1,logical_time_s=float(H*(r['cycle']+1)),car=c,x_mid_m=float(F(sum(pp[0]),2*m.Q)),y_mid_m=float(F(sum(pp[1]),2*m.Q)),physical_yaw_mid_rad=float(F(sum(pp[2]),2*m.Q))+(math.pi if c.startswith('B') else 0),v_rational=r['actual']['v'][c],delta_rational=r['actual']['delta'][c],a_rational=r['actual']['commands'][c][0],omega_rational=r['actual']['commands'][c][1],owner=ctx['owner'],permission_version=ctx['permission_version'],action=ctx['actions'][c],x_lo_Q=pp[0][0],x_hi_Q=pp[0][1],y_lo_Q=pp[1][0],y_hi_Q=pp[1][1],theta_lo_Q=pp[2][0],theta_hi_Q=pp[2][1]))
        metrics[case]=dict(passed=True,actual_steps=s['actual_steps'],actual_logical_time_s=float(F(s['actual_task_time_s'])),initial_budget=2802,final_budget=s['remaining_budget'],controller_wall_s=s['wall_s'],independent_wall_s=a['wall_s'],selection=a['selection'],target_overlap_pair=firstpair,target_overlap_ticks=len(target_overlap),all_after_release_two_body_motion_ticks=a['actual_overlap_ticks'],initial_preparation_ticks=55,initial_max_simultaneous_bodies=max(len(r['actual']['bounds']['active']) for r in records[:55]),initial_four_body_motion_ticks=sum(len(r['actual']['bounds']['active'])==4 for r in records[:55]),independent_minimum_road_m=a['minimum_road_m'],independent_minimum_pair_m=a['minimum_pair_m'],controller_minimum_road_m=float(F(s['minimum_road_m'])),controller_minimum_pair_m=float(F(s['minimum_pair_m'])),local_update=native,version_rejection_actual_old_tail_steps=32,damaged_new_K_actual_old_tail_steps=33,all_four_goals=True)
        evidence=[run/'config.json',run/'execution.jsonl.gz',run/'summary.json',run/'events.json',run/'independent_acceptance.json',run/'version_rejection/summary.json',run/'version_rejection/execution.jsonl.gz']
        checkpoints[case]=dict(passed=True,full_independent_replay_already_performed=True,files={str(f.relative_to(HERE)):sha(f) for f in evidence},code={name:sha(HERE/name) for name in ('accept_case.py','common.py','physics.py','derivatives.py','socp_rows.py','conic.py','inputs/interval_source.py','sources/certify_interfaces.py')})
    write(HERE/'metrics.json',metrics);write(HERE/'event_index.json',events);write(HERE/'results/independent_checkpoints.json',checkpoints)
    with (HERE/'comparison.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(comparison[0]));writer.writeheader();writer.writerows(comparison)
    (HERE/'comparison_windows.csv').write_bytes(b'\xef\xbb\xbf'+(HERE/'comparison.csv').read_bytes())
    preserved=read(HERE/'historical_hashes.json');assert all(sha(HERE.parent/name)==digest for name,digest in preserved.items());write(HERE/'preservation_check.json',dict(passed=True,historical_files_unchanged=len(preserved),original_inputs_unchanged=all(sha(HERE/'inputs'/name)==digest for name,digest in read(HERE/'inputs/SHA256SUMS.json').items())))
    calls=read(HERE/'results/solver_calls.json');assert len(calls)==2 and all(c['completed'] for c in calls)
    write(HERE/'versions.json',dict(python=sys.version,platform=platform.platform(),clarabel=clarabel.__version__,numpy=numpy.__version__,scipy=scipy.__version__,osqp_legacy_protocol_import=osqp.__version__))
    (HERE/'requirements.txt').write_text(f'clarabel=={clarabel.__version__}\nnumpy=={numpy.__version__}\nscipy=={scipy.__version__}\nosqp=={osqp.__version__}\n')
    gate=dict(version='region_reservation_001',passed=True,layers=dict(analytic_checks=dict(passed=True,segments=32,leaves=64),rational_inputs_and_domains=dict(passed=True,uniform_all_time_body_error=float(F(quant['body_error'])),extra_reserve=1e-8,domains='L1 polytope subsets, independent feedback and signed integration checks passed'),reservation_protocol=dict(passed=True,all_four_ACKs=True,resource_and_outside_exit_permissions_separate=True,damaged_version_and_K_rejected=True),control_replays=dict(passed=True,cases=metrics),local_socp_updates=dict(passed=True,actual_anchors=2,solver_calls=2,statuses=[r['status'] for r in comparison],policy='certificate_suboptimal_v1',scope='one forward 32-tick update per two-moving-car case, fixed peer predictions')),
        original_experiments_preserved=True,papers_changed=False,formal_batch=False,vehicle_priority_rule='experimental selector only; not frozen',limits=['Nominal model and ideal original four-car start','Synchronous in-process four-car common barrier; no physical clock or async guarantee','Two online selection traces, not enumeration of all 24 orders','One 64-input/65-variable forward local update per case; reverse is certified backup, not a reverse SOCP experiment','Numerical status AlmostSolved for same-direction case; no global optimality claim','Target reachable flow image retained, wider goal tolerance is not a new restart domain'])
    write(HERE/'GATE.json',gate)
    table='\n'.join(f"|{case}|{'→'.join(v['selection'])}|{v['actual_steps']}|{v['actual_logical_time_s']:.1f}|{v['target_overlap_ticks']}|{v['independent_minimum_road_m']:.6f}|{v['independent_minimum_pair_m']:.6f}|" for case,v in metrics.items())
    numtable='\n'.join(f"|{r['case']}|{r['actor']}|{r['status']}|{r['primal']:.3g}|{r['gap_abs']:.3g}|{r['delta_norm_upper']}|{r['predicted_32_energy_improvement_percent']:.6f}%|{r['actual_33_energy_improvement_percent']:.6f}%|" for r in comparison)
    timing='\n'.join(f"|{case}|{v['controller_wall_s']:.3f}|{v['independent_wall_s']:.3f}|{v['local_update']['solver_s']:.6f}|{v['local_update']['pipeline_s']:.3f}|" for case,v in metrics.items())
    report=f'''# 区域预约：有理输入、域衔接及两车重叠最小接入

已完成五层最小验收：原解析证书复现、有理输入与域衔接认证、预约共同提交、两种方向控制回放及各一次局部SOCP更新。新工作仅在region_reservation_001；历史实验、原资料和论文保留。

## 五层验收

|层|结果|主要证据|
|---|---|---|
|解析检查|通过：32段、64叶；原始道路/车对下界0.109973188539/0.528223178768m|results/analytic_rebuild、analytic_geometry.log、analytic_domains.log|
|有理输入与域|通过：四车实际有理ZOH、全时域量化界、带倒车的积分误差界、L1域包含和6个非零边界辅助回放|rational_inputs.json、uniform_quantization_check.json、integration_bounds.json、domain_checks.json、feedback_checks.json|
|预约协议|通过：资源与出口动作权限分开；四份计划同版ACK；损坏版本/新K拒绝后执行真实旧尾段|protocol.py、strategy.py、每个case的version_rejection和native_K_rejection|
|控制回放|通过：两条4车完整任务均2802拍、预算2802→0、两车实际重叠、独立8子段验收|results/*/execution.jsonl.gz及independent_acceptance.json|
|局部SOCP更新|通过（限定范围）：重叠期间B1/A2各一次32拍64输入/65变量新求解和实际采用|comparison.csv、event_index.json、局部source/derivative/assembly/raw/policy对象|

## 原资料与复现

原材料位置：/home/sunflower/Downloads/DMPC_region_construction_001。完整副本与原SHA清单保存在inputs；先在results/analytic_rebuild运行construct_certificate.py与check_domains.py，避免改写原证书。重建几何与原JSON语义逐项相同，域边界/拍数逐项相同；重建JSON换行格式引起的文件SHA差异单独保留，原文件未覆盖。

冻结四车协议的导入链包含已有OSQP（版本见versions.json和requirements.txt），本轮没有调用OSQP求解；新数值提案仅有两次Clarabel调用。基础模型沿用inputs/interval_source.py，SHA {sha(MODEL_PATH)}。原解析32段实为每车8个空间段；静态等待/完成组合24项也在追加量化余量后复核。原失败布局counterexample_old_wait_parking.json保留，未把原失败构造替换成成功记录。

## 有理输入、连续误差与适用域

alpha取已验证根区间中点，delta_q=0.422853926133，D_q为3sqrt(7)区间中点的有理近似。每段以实际有理速度爬升/制动保证v精确归零，停车时按实际轮角差转轮，保证delta精确达到新目标。基准备份允许v∈[-1,2]；倒车限1m/s，前进限2m/s。h固定0.2s，输入及轮角界保持。

恒定轮角段使用真实曲率tan(delta_q)/2.7的区间闭式ZOH积分，静止转轮保持位姿，变化轮角的ZOH用原带余项积分。新推导以绝对速度重证84/256的Simpson上界和15m/s²车角曲率弦误差上界，覆盖倒车。4子段控制器与8子段独立包络都计入每车0.05000001m管，并检查所有6对车身，而非单活动车与静态邻车假设。

必须区分两项量化结果：节点参考差最大9.48885644e-12m；最终连续全时域整车上界为{float(F(quant['body_error'])):.12g}m，预留1e-8m。rational_inputs.json里的maximum_body_quantization_error是初始导出的节点诊断字段，完整全时域证明以uniform_quantization_check.json为准。证明比较同一固定时序下的累计弧长与曲率误差，利用全路径<80m，形成对任意连续时刻有效的上界，未用节点采样替代连续证明。

加入额外管后，解析全参数证书道路下界为{float(F(domains['minimum_road'])):.12f}m，车对下界为{float(F(domains['minimum_pair'])):.12f}m，均保持0.5m要求；同侧出口/入口纵向分离仍大于0.962746m。四车初始准备最多55拍，入场准备101拍、通行562拍、出口尾段95拍不变。

实际采用的五状态接收域为“位置L1多面域×航向/速度/轮角区间”，是原L2位置域的收紧子集。重新推导了W_in→W_hold→G_in→G_out→目标的L1包含，而非直接套用原L2数值。Win非零v/delta的两拍反馈、Gin非零v/delta的一拍反馈均在feedback_controls.py导出精确有理命令，辅助边界回放真实执行并通过。主任务从给定理想静止初态开始，等待反馈恰为零；优化后的Gin反馈为实际非零K并执行，未声明任意外部扰动下都保持这份已物化时间表。

flow_images.json给出3036个相位流像索引，flow_receivers.jsonl.gz逐拍导出32拍接收索引。源域、实际输入对象和相位共同定义精确流像；每步保留真实尾段。优化前缀使用本次实际源快照的精确流像，进入Gin/K后接回新认证域链。目标后的保持使用更紧的可达终端流像，未把0.08m目标容差整体重新作为0.05m静态管的源域。

## 预约、选择与共同提交

资源为D∩{{|x|≤16}}，owner在入场准备前取得预约。实际达到Gout、完整矩形清空资源、旧出口尾段与下一准备策略相容后才释放；本轮没有优化更早释放。前车95拍出口尾段在同一联合版本中继续有效，下一车101拍准备与其重叠。资源占用权与区域外出口动作权限分别记录，后者不因owner切换消失。

复用冻结的四车Barrier；未复用旧Channel的±18/26.5m几何规则。新context绑定resource_id、owner、资源版本、全源快照/history、域证书、完整剩余见证、接收链与动作权限。四个公开计划及ACK指向同一context；执行端再次核对四份内容ID、提交编号及源版本，才输出四车实际首拍。

选择函数从当前未服务且在合法等待域的车中提出候选，先保留一份具体完整备份，再共同认可新选择。两种方向偏好仅用于本次受控轨迹，ID破同序，不是本方法冻结的车辆优先规则；用户所说“先来后到”只表示研究顺序。

|用例|实际在线选择轨迹|拍数|逻辑任务时间s|首个指定交接的两车实际运动重叠拍数|独立道路下界m|独立六对净空下界m|
|---|---|---:|---:|---:|---:|---:|
{table}

各用例在资源释放后累计194拍出现两车实际位姿运动；这些不是把静止转轮计为运动。初始55拍也独立检查四车同时准备。两条完整任务均到四个停车目标、v/delta归零，预算逐拍减1至0。

版本损坏分支分别在首个交接时拒绝错误版本，实际执行同一旧见证32拍；相反方向主分支选择B1，而被拒后的复制分支仍沿旧备份A2准备，明确没有偷偷采用被拒的新选择。另各拒绝一份损坏新K提案，再从本次数值源状态实际执行旧联合尾段33拍，进入Gin并执行其旧K，独立8子段核对。所有辅助分支与主任务时间/求解次数分开。

## 重叠期间的局部SOCP

两用例都在cycle787，旧车出口、下一车合流实际同时运动时，从实际状态/history新建锚点。逐步更新32拍其他车辆的固定预测，并生成相同时间片上的联合几何余量；不把运动中的旧车当作静态障碍。生成5×7单步实际流Jacobian区间、64列复合Jacobian、本地二阶界、物理行与Gin终端行。

求解设置沿用Clarabel0.11.1、rho=.001、scale=1e-4、Q量化预留、居中等价锥、三个容差1e-10、300迭代、5秒。每个用例一次求解，总共2次，无扫描。使用certificate_suboptimal_v1完整受理：精确有理最近量化与有限alpha回退、原行/真实连续联合前缀、Gin、实际K、保存后续、固定前缀能量非增与预算共同验证。

|用例|优化车|状态|原始残差|绝对gap|改动范数上界|32拍预测代价改善|同一33拍实际代价改善（含K）|
|---|---|---|---:|---:|---|---:|---:|
{numtable}

完整状态/gap/时间、raw→有理→实际输入绑定均在comparison.csv及对象。相反方向候选的首拍与旧首拍相同，首次实际修改拍及实际改变拍数在CSV单列；后续修改已实际执行，未把将来提案当成首拍改变。非优化车辆的计划保持冻结，但实际继续运动；本轮不是128维联合优化，也不是每车每拍在线求解。倒车由有理备份认证，本轮两处局部SOCP窗口均为前进；扩大到倒车SOCP或持续多轮优化需另立覆盖检查。同方向AlmostSolved的采用依据是完整物理与后续证书，不将它报告为严格Solved或严格最优性证明。

## 时间与开销

h=0.2s是逻辑ZOH步长。两条本轮实际执行记录均为2802拍，即560.4s，与解析上界数值相同是因为采用同一固定时序；不是拿解析上界充当测得轨迹。完全串行的3087拍/617.4s仅是重新构造得到的参考策略，本轮未运行串行实验，因此57s只作为这组构造的重叠调度差值，不作新的实验方法性能结论。

|用例|控制器墙钟s（含复制版本分支）|独立重放墙钟s|数值求解s|局部构造/受理流水线s|
|---|---:|---:|---:|---:|
{timing}

这些开销不是0.2s硬实时达标声明。通信为同步进程内可靠序列化协议；没有持续物理时钟、实际独立ROS进程异步或外部扰动实验。

## 实现版本、保留与复核

源码及原始依据均单独保存；源提交见plan.json。执行时源码快照在results/runtime_sources与executed_source_versions.json。完成回放后，strategy.py显式递归认证优化见证父策略，protocol.py补上不可改写已执行输入/权限及新数值提案源相位/history检查；5604份实际提交逐项满足这些加强条件，陈旧下一拍提案被拒绝。原运行源码快照保留，没有重跑或替换数值结果。独立验收入口不依赖该控制器缓存。

{len(preserved)}份历史非缓存文件SHA核对保持不变，原始理论包清单也逐项通过。新材料不做Git提交，不修改论文，不启动正式批量。

复核已存控制与独立验收：

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 run_validation.py
OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py
# 需要完整重算两个2802拍独立验收时：
OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py --full-replay
```

默认分发复核校验完整内容清单、独立验收的源码/输入checkpoint、重新复算两次数值来源/导数与33拍前缀；明确复用已通过的完整独立轨迹验收。full-replay才重新执行两条完整8子段独立重放。此区分写入迁移报告，避免把内容复核称为重新跑完任务。

尚未冻结：通用车辆选择/公平规则、更早释放策略、更多并行组合、任意域源状态的在线策略物化、倒车局部SOCP与连续多轮优化、扰动/持续物理时钟/异步部署。当前通过的是用户指定的最小两方向接入及对应证书范围。
'''
    (HERE/'report.md').write_text(report)
    write(HERE/'pending_parameters.json',dict(frozen_this_run=read(HERE/'plan.json'),not_frozen=['general priority/fairness selector','earlier release optimization','additional parallel configurations','arbitrary-domain-source online policy materialization','reverse SOCP anchors','persistent wall clock and asynchronous process deployment','disturbance model'],unchanged_theory_reference_bounds=dict(overlapped_ticks=2802,serial_ticks=3087),no_formal_batch=True))
    patch=[]
    for f in sorted(HERE.glob('*.py')):
        if f.name=='certify_interfaces.py':continue
        before=(HERE/'sources/derivatives.py').read_text().splitlines(True) if f.name=='derivatives.py' else []
        patch+=list(difflib.unified_diff(before,f.read_text().splitlines(True),fromfile='frozen_derivatives.py' if before else '/dev/null',tofile=f.name))
    (HERE/'changes.patch').write_text(''.join(patch))
    (HERE/'HANDOFF.md').write_text('目标：区域预约001五层最小接入。\n状态：原两证书复现、有理输入全时域量化/域/倒车积分、两种方向2802拍四车任务、两次局部求解采用、版本和K拒绝续行及独立验收均通过。\n约束：保留历史和论文，不做正式批量；一般优先规则、反向SOCP、持续物理时钟/异步仍待另立任务。\n源提交：'+read(HERE/'plan.json')['source_commit']+'\n证据：GATE.json、CERTIFICATE_BUNDLE.json、metrics.json、comparison.csv、各independent_acceptance.json。\n恢复：OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 run_validation.py\n分发校验：OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py\n')
    print(json.dumps(dict(passed=True,metrics=metrics),indent=2))
if __name__=='__main__':main()
