#!/usr/bin/env python3
"""Build the M0 stopping/acceptance closure record without relabeling old failures."""
import argparse
import csv
import hashlib
import json
from pathlib import Path


def read(path):return json.loads(path.read_text())
def lines(path):return [json.loads(line) for line in path.read_text().splitlines()]
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def table(headers,rows):return '\n'.join(['|'+'|'.join(headers)+'|','|'+'|'.join(['---']*len(headers))+'|',*['|'+'|'.join(map(str,row))+'|' for row in rows]])
def write(path,value):path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def report(root):
    closed=root/'closed_loop';metrics=read(closed/'comparison.json');cases=read(root/'hand_cases.json');local=read(root/'local_paired/results.json')
    results=[];diagnostics={};repeat={}
    for name,r in metrics.items():
        directory=closed/name;rows=list(csv.DictReader((directory/'actual.csv').open()));cross=[]
        for car in ('A','B'):
            rr=[x for x in rows if x['car']==car]
            for before,after in zip(rr,rr[1:]):
                product=float(before['v'])*float(after['v'])
                if product < -1e-10:cross.append(dict(car=car,time_s=float(after['time_s']),before_v=float(before['v']),after_v=float(after['v']),product=product))
        solves=lines(directory/'solves.jsonl')
        adjustments=[dict(cycle=s['cycle'],update=i,**a) for s in solves for i,q in enumerate(s['subproblems'],1) for a in q.get('stop_input_adjustments',[])]
        diagnostics[name]=dict(crossings=cross,stop_input_adjustments=adjustments,actual_sha256=digest(directory/'actual.csv'))
        results.append(dict(run=name,completed=r['completed'],original_full_acceptance=r['independent_full_task_passed'],
            completion_s=r['completion_sim_s'],sign_crossings=len(cross),input_adjustments=len(adjustments),
            minimum_gap_m=r['minimum_sampled_gap_m'],cycle_p50_ms=r['cycle_p50_ms'],cycle_p95_ms=r['cycle_p95_ms'],
            cycle_p99_ms=r['cycle_p99_ms'],cycle_max_ms=r['cycle_max_ms'],over_200ms=r['cycles_over_200ms']))
    for arm in ('unrepaired','repaired'):
        repeat[arm]=dict(actual_byte_identical=(closed/f'{arm}_r1/actual.csv').read_bytes()==(closed/f'{arm}_r2/actual.csv').read_bytes())
    repeat['unrepaired']['matches_M0_001']=(closed/'unrepaired_r1/actual.csv').read_bytes()==(root.parent/'M0_001/B2_sequence_warm_r1/actual.csv').read_bytes()
    write(root/'diagnostics.json',diagnostics);write(root/'repeatability.json',repeat)
    with (root/'comparison.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(results[0]));writer.writeheader();writer.writerows(results)
    source=read(closed/'provenance.json')
    for name,sha in source['source_sha256'].items():assert digest(closed/'source'/name)==sha,name
    frozen=read(root/'frozen/provenance.json');origin=Path(frozen['source'])
    for name,sha in frozen['original_hashes'].items():assert digest(origin/name)==sha,name
    for name,sha in frozen['frozen_sha256'].items():assert digest(root/'frozen'/name)==sha,name
    assert all(c['result']['passed']==c['expected_passed'] and c['result']['gear_sign_jump']==c['expected_sign_jump'] for c in cases)
    assert all(x['accepted'] and x['original_candidate_reproduced'] for x in local)
    fixed=all(metrics[f'repaired_r{i}']['independent_full_task_passed'] and metrics[f'repaired_r{i}']['future_execution_consistency_passed'] for i in (1,2))
    baseline=all(metrics[f'unrepaired_r{i}']['completed'] and not metrics[f'unrepaired_r{i}']['independent_full_task_passed'] and len(diagnostics[f'unrepaired_r{i}']['crossings'])==3 for i in (1,2))
    status=dict(closed_under_original_M0_contract=bool(fixed and baseline and repeat['unrepaired']['matches_M0_001']),
        repaired_full_original_acceptance=fixed,original_three_failures_reproduced=baseline,
        frozen_cycles=[x['cycle'] for x in local],hand_cases_checked=len(cases),historical_inputs_unchanged=True,
        sampled_sign_policy_is_not_physical_equivalence=True,no_general_or_realtime_proof=True)
    write(root/'closure.json',status)
    head='通过' if status['closed_under_original_M0_contract'] else '未通过，保留反例'
    result_table=table(['运行','到达目标','原完整验收','任务s','跨零数','首输入修复数','最小采样净空m','全周期最大ms'],
        [[r['run'],r['completed'],r['original_full_acceptance'],r['completion_s'],r['sign_crossings'],r['input_adjustments'],f"{r['minimum_gap_m']:.9f}",f"{r['cycle_max_ms']:.2f}"] for r in results])
    case_table=table(['人工案例','期望通过','原判据跨零','实际通过','含义'],
        [[c['name'],c['expected_passed'],c['result']['gear_sign_jump'],c['result']['passed'],c['meaning']] for c in cases])
    text=f'''# M0 停车与验收衔接关闭记录

**本轮结论：{head}。** 关闭限定于当前双车场景、原采样验收与保留的最小输入修复，不把跨零判据称为普遍正确的物理停车定义。

## 历史证据与范围

以[原T1](../B1&B2-T1/report.md)和[M0_001](../M0_001/report.md)为起点。M0_001两轮热启动都在81.0 s到达双方目标，但因三个速度跨零样本未通过原完整验收；该结论、日志及哈希保持不变。之前的M0_002修复结果另存，本轮不拿它代替新闭环复核。

本轮保留已有最小停车输入修复，未再叠加运动算法改动。新增相邻周期冻结、人工正确/错误案例，以及**热启动两臂均开启**的修复前后完整闭环各两轮。不是冷启动与热启动的混合对比；运行器断言两臂自行车源码剥去停车修复后逐字节相同，参考QP、动态积分、原验收器、Flag也逐字节核对一致。

## 链一：参考、候选、实际输入、积分

冻结旧cycle120–123、241–244、402–404，共11个状态，覆盖三个跨零周期121/242/403及制动、静止修正或转向邻接。每车初态与实际日志核对到2e-12；参考QP全部参数必须重建一致。非MOVE的冻结控制队列保存该日志的完整N步控制，用于本次有限预测重建，不声称恢复未知的更长队列。

每个周期保存参考QP矩阵、参考解、参考首输入与积分、最终自行车解、实际已执行加速度/转角速度、10个0.02 s子步速度及解析v0+t*a。首个样本的a字段表示前一积分区间；后10个样本逐一核对当前控制。原候选整段控制重建到1e-10，实际速度积分到1e-14。

11个冻结状态中，原版仅在121/242/403出现本周期跨零；它们的参考首步末速度为正约4.9–5.7e-6 m/s，自行车层稍强的制动使实际末速度约-1e-7 m/s。实际控制等于自行车层最终输入，速度符合积分方程，因此排除参考直接给负速度、执行器换了输入或日志状态跳变等解释。11个同参考修复候选均通过原仿射、信赖域和非线性候选验收，原三个跨零在修复后不再发生。

## 链二：停车要求与验收语义

原阶段切换需要实际路径位置到位（1e-6 m范围）及低速：abs(v)<1e-6时若仍大于1e-12，继续用一周期真实输入制动；到近零后才切段。原目标容差速度0.025 m/s属于任务完成口径，不是段间停车阈值。WAIT标签本身既不能改变状态，也不能证明停车。

原check_series使用v[k]*v[k+1]<-tolerance²（此场景为-1e-10 (m/s)²）。这与动力学连续性、输入可执行性及实际停车不是同一个判据：有界加速度可以在两个采样点间平滑经过零，原判据仍拒绝；反之足够小的正负速度乘积可能落在死区而通过。原M0的三个样本是数值小幅过制动与更严格采样验收之间的衔接问题，不能仅凭gear_sign_jump宣称物理不连续或碰撞。

人工案例以闭式恒加速度构造，不依赖任一RK4，调用原check_series断言期望正确/错误；另直接调用Agent.advance，验证未到位置、仍在运动或存在-1e-7残速时不会提前切段，且状态数组不变。

{case_table}

人工平滑换向案例揭示了判据的保守性；这不据以删除M0_001失败项。本轮沿用原完整验收契约，选择让停车控制同时满足物理积分和该契约。未证明原判据在任意反向、变采样或噪声工况都正确；若以后重新定义它，必须另设验收消融。

## 最小修复与防伪通过

仅在MOVE且stop_at_end、数值QP返回原允许状态后，若首输入导致方向上的v0+h*a<0，采用a_stop=-v0/h。改的是执行加速度；车辆状态始终由原积分器产生，保留数值舍入的约1e-18 m/s末速度，不覆盖成0。角速度和后续输入不在该修复中更改。

修复先于最终全部原仿射残差、预测状态、目标值、输入记录、信赖域和非线性重演检查；不符合原界或门限仍拒绝。保留求解状态与stop_input_adjustments原/新加速度记录，不把未收敛改为成功，也不将修复后的候选声称为原QP精确最优解。人工速度/姿态重置案例由分量方程拒绝，提前WAIT案例由原切段逻辑阻止。

参考热启动、导数、最多3次更新、目标/地图/物理界、数值带与验收容差、Flag和完整未来释放检查保持原样。

## 新完整闭环与独立验收

{result_table}

新日志独立重读：两半步标量RK4逐区间及从原初态累计重积分，重新检查控制界、道路、旋转车身、间距、原目标、授权/释放、跨零，以及提交样本数量、事件owner和所有出口计时。几何函数仍复用原审计实现，不宣称第二套独立几何算法。

修复前两轮保留三个失败；修复后两轮是否通过以closure.json为准。各臂actual.csv两轮是否逐字节相同见repeatability.json；原版是否复现M0_001原轨迹也单列。失败仍在新日志中保留，不因任务到达而改报通过。完整周期与单QP耗时分列，包含同步完整未来释放预演。

## 已排除与未排除

已排除：这三个旧实例由参考首步负速度、实际控制错配、速度状态跳写引起；本轮收益来自热启动/导数/停车目标/验收门限一起改变；删掉原失败项或提前WAIT制造通过。

尚未排除：跨零判据在其他采样率、噪声和反向任务中的语义误判；其他初态/道路下输入修复破坏约束而被正确拒绝；有限采样之外的违约。未证明连续时间安全、递归可行性、实时性或DMPC。最大完整周期仍含长时间同步预演，不满足0.2 s控制周期。

## 工件与复现

frozen/与local_paired/保存11个原始/修复局部状态；hand_cases.json保存人工案例；classification.json保存问题性质；closed_loop/保存四条完整新日志、源码哈希、环境和命令；comparison.csv、diagnostics.json、repeatability.json、closure.json保存本轮结果。

```bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH=/tmp/t1_qp_deps:$PYTHONPATH OPENBLAS_NUM_THREADS=1 ROS_DOMAIN_ID=98 ROS_LOG_DIR=/tmp/m0_closure_ros python3 examples/head_on_mpc/scripts/run_m0_closure.py --output examples/head_on_mpc/results/M0_closure_002/closed_loop
```

局部冻结/人工案例命令与准确环境见对应源码快照及provenance；新运行使用新目录，历史材料不覆盖。临时OSQP1.1.3/joblib依赖按历史清单SHA256重建，未改项目或系统依赖。
'''
    (root/'report.md').write_text(text)
    print(json.dumps(status))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True);args=parser.parse_args();report(args.run)
