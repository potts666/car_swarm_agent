"""Build measured comparison and layered gate from saved evidence, no solves."""
from fix_common import *
import csv,math,difflib

def main():
    probes=read(HERE/'results/probe/summary.json');comparison=[];runs={};counts={};sc=read(HERE/'results/self_checks.json');assert sc['passed']
    for tag,item in probes.items():
        raw=get(HERE/'objects',item['result_id']);chosen=get(HERE/'objects',item['chosen_id']);info=raw['info'];exact=read(HERE/f'results/probe/{tag}_exact_audit.json');name=tag.rsplit('_',1)[0]
        checks=dict(primal=info['res_primal']<1e-10,dual=info['res_dual']<1e-10,gap=info['gap_abs']<1e-10 or info['gap_rel']<1e-10,ktratio=info['ktratio']<=1)
        comparison.append(dict(interface=name,representation=raw['representation'],status=raw['status'],pre_postprocess_status='unknown',primal=info['res_primal'],dual=info['res_dual'],gap_abs=info['gap_abs'],gap_rel=info['gap_rel'],ktratio=info['ktratio'],strict_failed=';'.join(k for k,v in checks.items() if not v),iterations=raw['iterations'],solver_s=raw['solver_wall_s'],cancellation_ratio=exact['cancellation_ratio'],strict_accepted=item['strict_accepted'],certificate_accepted=chosen['accepted'],policy=chosen['admission_policy'],alpha=chosen.get('accepted_alpha'),assembly_id=raw['assembly_id'],raw_result_id=item['result_id']))
    for name in ('curve','straight_acceleration'):
        dest=HERE/'results'/name;summary=read(dest/'summary.json');audit=read(dest/'independent_acceptance.json');assert summary['passed'] and audit['passed']
        raw=get(HERE/'objects',probes[name+'_centered']['chosen_id']);anchor=get(HERE/'objects',raw['anchor_id']);delta=list(map(F,raw['accepted_delta']));old=anchor['plan'];base=[tuple(map(F,u)) for u in old['remaining'][:32]];new=[tuple(map(F,u)) for u in raw['accepted_proposal']['remaining'][:32]]
        E0=H*sum(x*x for u in base for x in u);E1=H*sum(x*x for u in new for x in u);assert E1<E0
        with gzip.open(dest/'execution.jsonl.gz','rt') as stream:logs=list(map(json.loads,stream))
        ref=get(HERE/'objects',old['object']) if (HERE/'objects'/(old['object']+'.json.gz')).exists() else None
        data,m=setup();commands=[tuple(map(F,u)) for u in data.cert['inputs']];diverged=[r['phase'] for r in logs if r['kind']=='B' and tuple(map(F,r['actual']['executed_input']))!=commands[r['phase']]]
        prefix=logs[anchor['phase']:anchor['phase']+32];assert len(prefix)==32
        executed_energy=H*sum(F(x)**2 for r in prefix for x in r['actual']['executed_input']);assert executed_energy==E1
        shifted=summary['solver_events'][1];assert shifted['kind']=='actual_shift_recertification'
        counts[name]=dict(numerical_proposals=1,full_certificate_passed=1,joint_accepted=summary['accepted_numeric_candidates'],actual_nonzero_prefix_changes=sum(r['phase'] in diverged for r in prefix),actual_nonzero_all_changes=len(diverged),first_divergence_phase=min(diverged),shift_passed=audit['actual_steps']-1,actual_source_derivative_recertifications=len(audit['shift_recertifications']),K_executions=audit['actual_K_executions'],parked_handoffs=1,rejected_then_old_K_executed=audit['damaged_K_proposals_rejected_with_actual_old_K'],remaining_budget=audit['remaining_budget'])
        runs[name]=dict(counts=counts[name],input_norm_upper=str(norm_upper(delta)),first_delta=list(map(str,delta[:2])),old_prefix_energy=str(E0),actual_prefix_energy=str(E1),energy_improvement=float(E0-E1),energy_improvement_percent=float((E0-E1)/E0*100),controller_wall_s=summary['wall_s'],independent_wall_s=audit['wall_s'],minimum_road_m=audit['minimum_road_lower_m'],minimum_pair_m=audit['minimum_pair_lower_m'],new_anchor_id=shifted['anchor_id'],new_derivative_id=shifted['derivative_id'],shift_free_box_passed=shifted['free_box_zero_check']['passed'],shift_new_optimization=False,duration_ticks=240,duration_change_ticks=0,initial_budget=1279,final_budget=1039)
    write(HERE/'results/metrics.json',runs)
    with (HERE/'comparison.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(comparison[0]));writer.writeheader();writer.writerows(comparison)
    gate=dict(version='C0_fix_001',cause_diagnosis=dict(known_strict_failures=True,archive_444_reproduced=True,pre_postprocess_source='unknown via public interface',linear_system_root_cause='not established'),original_rule_numerical_repair=dict(passed=False,statuses=[r['status'] for r in comparison],tolerances_unchanged=True),new_admission_rule=dict(passed=True,policy='certificate_suboptimal_v1',conditional_theorem='instructions/theory.md §8',strict_optimality_claim=False),control_succession_closed=dict(passed=True,interfaces=counts,independent_acceptance=True),initial_solver_calls=4,additional_solver_calls=0,original_C0_gate_unchanged=True,original_history_preserved=True,formal_batch=False,C2=False,reservations=False,multiple_movers=False,papers_changed=False,git_commit=False,unresolved=['Strict Solved at 1e-10 remains unmet','Public callback exposes Unsolved before final AlmostSolved, not guaranteed pre-postprocess cause','Per-factorization regularization/refinement activity unavailable','Next actual anchor recertified and inherited; no second optimization or full-task performance claim'])
    write(HERE/'GATE.json',gate)
    changes=[]
    for original,new in [('base_c/base_b/execute_case.py','execute_fix.py'),('base_c/base_b/accept_case.py','accept_fix.py'),('base_c/base_b/validate_numeric.py','validate_fix.py')]:
        changes.extend(difflib.unified_diff((HERE/original).read_text().splitlines(True),(HERE/new).read_text().splitlines(True),fromfile=original,tofile=new))
    for new in ('fix_common.py','fix_socp.py','probe.py','self_check.py','run_validation.py'):
        changes.extend(difflib.unified_diff([],(HERE/new).read_text().splitlines(True),fromfile='/dev/null',tofile=new))
    (HERE/'changes.patch').write_text(''.join(changes))
    table='\n'.join(f"|{n}|{v['counts']['actual_nonzero_prefix_changes']}|{v['energy_improvement_percent']:.6f}%|{v['minimum_road_m']:.6f}|{v['minimum_pair_m']:.6f}|" for n,v in runs.items())
    report=f'''# C0 诊断与修复验证（2026-11-13）

本轮“数值新候选未受理”的控制接续阻塞已在两个固定接口关闭，采用独立规则 `certificate_suboptimal_v1`。严格 `Solved` 数值修复未通过；原 C0 失败门禁保持原样。本结论仅覆盖本轮两个局部接口，不推广为全程性能或扰动稳定性结论。

## 四层验收

|层级|结果|证据|
|---|---|---|
|原因定位|归档12例严格失败4/4/4复现；12份原模型和数值锥内点通过。停止前具体来源保留未知|archive_audit.json；results/probe/*_trace.json|
|原规则数值修复|未通过：4次均 AlmostSolved；居中表达两例均原始残差不达1e-10|comparison.csv|
|新受理规则|通过：同一原始结果，S拒绝，C完整证书通过；没有追加求解|results/probe/summary.json；validate_fix.py|
|控制接续|通过：各240拍、1次数值共同受理、实际修改32拍前缀、实际移位、K、拒绝后同一旧K续行、停车换权及下一车首拍|results/curve、results/straight_acceleration 的执行日志和 independent_acceptance.json|

## 数值诊断与局限

原表达曲线主要失败于gap，直线主要失败于原始残差；居中表达曲线原始残差约7.92e-9，直线约1.63e-7，均超过1e-10。二者gap和对偶残差已满足严格条件，kappa/tau也满足外围条件。原表达的抵消指标约11.39/12.14，居中后均为1；这是几何改写的观察，不能据此断言KKT误差根因。

完整保存65维x、440维s/z、原矩阵、所有公开设置、逐轮callback/verbose记录、原始及对偶目标和精确二进有理数残差向量。高精度点积先用Fraction精确求和，再以浮点显示范数；独立重算的归一化值基于返回的x/s/z和L2范数，单独保存，未冒称与内部在缩放/后处理前的指标完全相同。官方 v0.11.1 的 info.rs/settings.rs 为停止条件来源。公开callback最后仍为Unsolved，最终为AlmostSolved，故后处理前的InsufficientProgress/NumericalError具体来源未知；未插桩、未改变求解器版本。线性求解器为qdldl，1线程；默认正则化及迭代改进设置已保存，逐次触发及次数未公开。

## 理论条件、实现、实际观察

理论：instructions/theory.md §7 的等价锥恒等式、§5严格内点和§8的A_cert是条件性依据。安全/递归可行/有限完成依赖名义模型、合法历史策略和保存后续、一致提交及预算下降；次优受理不提供严格最优性或扰动稳定性证明。

实现：fix_socp.py仅重写H>0、m>0的锥块；H=0线性行及m<=0旧有效分支保留。原Q行、量化预留、尺度1e-4、64维输入、32拍、rho=.001、时长/分组/停车换权/目标权重均固定。状态白名单只开启检查，不直接受理；原行、范数重建、精确最近量化、真实前缀能量、实际K/后继流像、后续及预算再由A验收和四车协议提交。独立检查器按65维和精确有理量化核对数值来源，重新生成导数和终端参数，并以更细8子段重放连续几何。

观察：

|接口|实际改变前缀拍数|真实前缀能量改善|道路净空下界m|六对车身净空下界m|
|---|---:|---:|---:|---:|
{table}

各1个数值提案、1个完整认证、1次共同受理、1次实际K、1次损坏K拒绝后续行、1次停车换权。首次输入分歧均发生在提案首拍。239拍连接加下一车首拍共240拍，预算1279→1039逐拍减1，无增补。已执行所有32拍数值前缀，能量由原始实际命令独立重算；固定时长变化为0。结果不使用A人工构造候选替代数值提案。

下一实际锚点均重建32子段导数、更新history/phase/信赖中心、检查原行及连续真实尾段、接收K和后续；下一免费接收盒检查均通过。本轮继承已受理尾段完成接续，没有追加第二次数值求解；这不构成持续在线优化验收。预算绑定原8连接已认可策略及同版见证，不从文档常数重建其它策略。

## 代码与保存

新增代码和结果仅在stage_c_fix_001。changes.patch包含共享锥表达、受理和候选/共同提交调用链、实际移位日志、65维独立验收改动及自检。源提交与固定配置见plan.json。base_c保存冻结旧C及其B/A依赖，objects按内容ID核对；指令及身份清单也保留。

自检覆盖精确恒等式/正负tau/内外边界、线性和非正margin分支、量化误差、状态白名单/NaN、raw来源/anchor/导数损坏，以及K、phase、对象、域、期限、见证、预算损坏拒绝。历史文件逐项SHA保持不变。独立验收初版Fraction与JSON字符串直接比较导致断言失败，原失败证据保留为failed_001；改用相同规范化内容身份比较后通过，未重跑求解与控制日志。

## 复核与恢复

在本目录：

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 run_validation.py
OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py
```

run_validation.py以180秒分层超时、完整日志及内容校验checkpoint复用已通过证据。verify_delivery.py在分发目录核验清单及两份独立重放，无新求解。复现实验需在新目录运行probe.py（4次）及执行器，禁止覆盖旧证据；依赖Python3、Clarabel0.11.1、NumPy/SciPy，实测版本见results/probe/environment.json。分发清单与SHA另附，重复内容以tar硬链接去重。

未开展C2、资源预约、多活动车、异步通信、正式批量或论文修改；无Git提交。后续扩展须另立任务。待解决：严格数值收敛、停止前状态可观测性、持续在线多次优化与全程评价。
'''
    (HERE/'report.md').write_text(report)
    (HERE/'HANDOFF.md').write_text('目标：C0两接口诊断及实际接续；四次求解完成，证书策略接续已通过，严格Solved仍失败。\n约束：A/B/C/P0及论文保留；不推进后续阶段。\n源版本：'+read(HERE/'plan.json')['source_commit']+'\n证据：GATE.json、results/metrics.json、两份independent_acceptance.json、comparison.csv。\n恢复：OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 run_validation.py\n交付复核：OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py\n')
    print(json.dumps(dict(gate=gate,metrics=runs),ensure_ascii=False,indent=2))
if __name__=='__main__':main()
