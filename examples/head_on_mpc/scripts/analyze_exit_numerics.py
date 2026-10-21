#!/usr/bin/env python3
"""Summarize complete reruns without editing or relabeling previous results."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from head_on_core import write_json


def summarize(root):
    rows = json.loads((root/'summary.json').read_text())
    reports = [r for p in root.glob('case_*/*/solves.json') for r in json.loads(p.read_text())]
    checks = [a for p in root.glob('case_*/*/path_checks.json') for a in json.loads(p.read_text())]
    residual_max = {}
    accepted_max = {}
    for r in reports:
        for key, value in r['independent_acceptance']['residuals'].items():
            residual_max[key] = max(value, residual_max.get(key, 0.))
            if r['accepted']:
                accepted_max[key] = max(value, accepted_max.get(key, 0.))
    path_max = {}
    for a in checks:
        if not a['passed']:
            continue
        for k, v in a['residuals'].items():
            path_max[k] = max(v, path_max.get(k, 0.))
    policies = {}
    for policy in ('fixed_A', 'screened'):
        group = [r['baselines'][policy] for r in rows]
        good = [b for b in group if b['feasible']]
        policies[policy] = dict(success_count=len(good), total=len(group),
            mean_retreat_distance_m=float(np.mean([b['retreat_distance_m'] for b in good])),
            mean_exit_simulation_s=float(np.mean([b['exit_time_s'] for b in good])))
    qp = [r['qp_crosscheck'] for r in reports if 'qp_crosscheck' in r]
    accepted_pairs = [(r, r['qp_crosscheck']) for r in reports if r['accepted'] and
                      'qp_crosscheck' in r and r['qp_crosscheck']['independent_acceptance']['passed']]
    hessian = [r['hessian_min_eigenvalue'] for r in reports+qp if 'hessian_min_eigenvalue' in r]
    delays = [r['decision_wait_wall_seconds'] for r in rows]
    result = dict(run=str(root), policies=policies,
        role_success={c: sum(r['candidates'][c]['feasible'] for r in rows) for c in ('A', 'B')},
        primary_solve_calls=len(reports), qp_diagnostic_calls=len(qp), retry_count=sum(r['retry_count'] for r in rows),
        nonconverged_accepted_solves=sum(not r['solver_success'] and r['accepted'] for r in reports),
        rejected_returned_solves=sum(not r['accepted'] for r in reports),
        qp_diagnostic_passed=sum(r['independent_acceptance']['passed'] for r in qp),
        primary_rejected_but_qp_passed=sum(not r['accepted'] and r.get('qp_crosscheck', {}).get('independent_acceptance', {}).get('passed', False) for r in reports),
        minimum_hessian_eigenvalue=min(hessian) if hessian else None,
        max_objective_gap_when_both_accepted=max((abs(q['objective_gap']) for r, q in accepted_pairs), default=None),
        max_control_difference_when_both_accepted=max((q['max_control_difference'] for r, q in accepted_pairs), default=None),
        max_returned_residuals=residual_max, max_accepted_local_residuals=accepted_max,
        max_passed_full_path_residuals=path_max,
        candidate_generation_wall_s=[r['candidate_generation_total_wall_seconds'] for r in rows],
        decision_wait_wall_s=delays, decision_wait_min_s=min(delays), decision_wait_max_s=max(delays),
        decision_within_0_4_s_count=sum(r['decision_within_control_period'] for r in rows))
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runs', nargs='+', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    results = [summarize(root) for root in args.runs]
    write_json(args.output/'analysis.json', results)
    lines = ['# 数值接受规则、等价 QP 与完整预演定位', '',
             '所有结果均为新规则下从同一组安全静止初值完整重跑；没有改写旧表。成功要求局部独立分项验收、配对完整预测、实际路径和退出终态全部通过。', '',
             '|主求解器/诊断|A退成功|B退成功|固定A|筛选|主调用|QP诊断调用|重试|选角前等待 s|',
             '|---|---|---|---|---|---|---|---|---|']
    for r in results:
        lines.append(f'|{Path(r["run"]).name}|{r["role_success"]["A"]}/8|{r["role_success"]["B"]}/8|'
                     f'{r["policies"]["fixed_A"]["success_count"]}/8|{r["policies"]["screened"]["success_count"]}/8|'
                     f'{r["primary_solve_calls"]}|{r["qp_diagnostic_calls"]}|{r["retry_count"]}|'
                     f'{r["decision_wait_min_s"]:.4f}–{r["decision_wait_max_s"]:.4f}|')
    for r in results:
        lines += ['', f'## {Path(r["run"]).name}', '',
                  f'接受未收敛但局部检查通过的返回解 {r["nonconverged_accepted_solves"]} 次；仍拒绝 {r["rejected_returned_solves"]} 次。',
                  f'QP诊断通过 {r["qp_diagnostic_passed"]}/{r["qp_diagnostic_calls"]}；其中主返回解不通过而等价QP通过 {r["primary_rejected_but_qp_passed"]} 次。',
                  f'Hessian最小特征值 {r["minimum_hessian_eigenvalue"]}。两个返回解均通过时，最大绝对目标差 {r["max_objective_gap_when_both_accepted"]}，最大控制分量差 {r["max_control_difference_when_both_accepted"]}。这只是数值交叉验证，不以目标差代替可行性检查。', '',
                  '|基线|成功|平均退让距离 m|平均退出模拟时间 s|', '|---|---|---|---|']
        for name, b in r['policies'].items():
            lines.append(f'|{name}|{b["success_count"]}/8|{b["mean_retreat_distance_m"]:.4f}|{b["mean_exit_simulation_s"]:.4f}|')
        lines += ['', '平均值仅包含各自成功样本；成功样本集合不同时不能将均值差直接解释为策略收益。', '',
                  '|已接收局部解的残差项目|最大残差|', '|---|---|']
        for k, v in r['max_accepted_local_residuals'].items():
            lines.append(f'|{k}|{v:.8g}|')
    lines += ['', '## 结论与方法定位', '',
              '最终等价缩放 QP 完整运行中两个角色均 8/8，固定 A 与筛选也均 8/8，成功率收益消失。原差异主要反映数值求解与接受规则，而不是证明一个角色物理上无法退出。',
              '固定邻车预测、固定横向重叠掩码、固定阶段目标后，x/v是加速度序列的仿射函数，所有纵向约束线性；目标为正权重平方和，含正定加速度项，故局部问题为严格凸QP。终端双向不等式合并为等式及正数行/目标缩放不改变可行域和最优解。道路区域并集与配对完整路径检查在QP外，整个候选生成器不是一个凸QP。',
              'SLSQP独立验收重跑接受了先前未收敛但满足检查的解，但后续仍可能出现违反终端速度或目标边界的返回解；这些解继续被拒绝。OSQP主求解实验从头完整运行，没有用QP结果替换旧日志中的失败，也没有失败后重试。',
              '当前定位：离线完整闭环模拟预演工具。阻塞事件后先在冻结的环境快照上串行预演两条直到退出的候选，完成后才选角；预演期间没有实际车辆推进模型，也没有在线DMPC延迟补偿。两个基线在此离线比较中共享相同等待成本，真实固定角色实现可以省去另一条预演，不能把共享预算称为其必要部署成本。',
              '每个初值的总墙钟时间、主求解调用、QP诊断调用、重试和选角前等待详见各运行 summary.csv/json。墙钟时间包含生成、审计和原始日志写入；计时不包含Python进程启动。主OSQP计时还包含首次模块加载，按实测保留。',
              'SLSQP上限120次，OSQP上限20000次ADMM迭代，迭代不可等价比较；跨求解器比较需看实测墙钟时间。每个求解器内部两种基线的规则和预算相同。诊断运行的延迟包含额外QP交叉验证，不能当成无诊断部署成本。',
              '尚未验证实时最坏时延、重复试验分布、deadline中断或超时备用控制，因此不能称为已满足0.4 s实时预算的在线方法。可作为事件触发恢复规划的研究原型，但实际执行前仍需处理等待期间环境变化与重新验证。',
              '退出模拟时间只到退让车停驻让出通道，不是后续恢复或任务完成时间；本轮只能比较退让距离、退出时长和计算代价。连续时间安全与转向可达性仍未证明。']
    (args.output/'report.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps([{k:r[k] for k in ('run','role_success','primary_solve_calls','qp_diagnostic_calls','nonconverged_accepted_solves','decision_wait_min_s','decision_wait_max_s','policies')} for r in results], ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
