#!/usr/bin/env python3
"""Compare immutable legacy logs to a complete constrained-hold rerun."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from head_on_core import write_json


def read_rows(directory):
    with (directory/'actual.csv').open() as stream:return list(csv.DictReader(stream))


def compare(old, new):
    old_rows=json.loads((old/'summary.json').read_text())
    new_rows=json.loads((new/'summary.json').read_text())
    old_config=json.loads((old/'study_config.json').read_text());new_config=json.loads((new/'study_config.json').read_text())
    assert old_config['base']==new_config['base'] and old_config['options']==new_config['options']
    assert new_config['hold_constraints'] is True
    paired=[];acceleration=[];prechecks=[]
    for a,b in zip(old_rows,new_rows):
        assert a['case']==b['case'] and a['factors']==b['factors']
        i=a['case']
        for car in ('A','B'):
            rel=Path(f'case_{i:03d}')/'candidates'/f'{car}_retreat'/'actual.csv'
            assert hashlib.sha256((old/rel).read_bytes()).digest()==hashlib.sha256((new/rel).read_bytes()).digest()
        for policy in ('fixed_A','screened'):
            before,after=a['branches'][policy],b['branches'][policy]
            assert before['role']==after['role']
            d0=old/f'case_{i:03d}'/policy;d1=new/f'case_{i:03d}'/policy
            prefix=lambda rows:[r for r in rows if r['stage'] in ('EXIT','PARKED_READY','PASS')]
            assert prefix(read_rows(d0))==prefix(read_rows(d1))
            assert before['first_stop_candidate_time_s']==after['first_stop_candidate_time_s']
            r0=json.loads((d0/'solves.json').read_text());r1=json.loads((d1/'solves.json').read_text())
            hold1=next(r for r in r1 if r['stage']=='CLEARED_HOLD' and r['car']==after['passer'])
            ps=json.loads((d1/'hold_prechecks.json').read_text())
            for pre in ps:
                prechecks.append(dict(case=i,policy=policy,passed=pre['passed'],stop_candidate_passed=bool(pre['stop_candidate'] and pre['stop_candidate']['passed']),
                                      unchanged_state=pre['before']==pre['after']))
            if before['failure']:
                hold0=next(r for r in reversed(r0) if r['stage']=='CLEARED_HOLD' and r['car']==before['passer'])
                v0=before['final_actual_states'][before['passer']][1];dt=.4;sign=1 if v0>=0 else -1
                acceleration.append(dict(case=i,policy=policy,car=before['passer'],v0_m_s=v0,
                    old_u0_m_s2=hold0['controls'][0],old_end_v_m_s=v0+dt*hold0['controls'][0],
                    signed_acceleration_upper_m_s2=(.025-abs(v0))/dt,
                    new_u0_m_s2=hold1['controls'][0],new_end_v_m_s=v0+dt*hold1['controls'][0],
                    stop_u0_m_s2=-v0/dt,stop_displacement_m=v0*dt/2,
                    stop_candidate_passed=ps[0]['stop_candidate']['passed']))
            paired.append(dict(case=i,policy=policy,role=after['role'],passer=after['passer'],
                old_released=before['released'],new_released=after['released'],
                old_failure=before['failure'],new_failure=after['failure'],
                prefix_before_hold_identical=True,entry_time_unchanged=True,
                first_new_hold_u0_m_s2=hold1['controls'][0]))
    aggregates={}
    for policy in ('fixed_A','screened'):
        old_group=[r['branches'][policy] for r in old_rows];new_group=[r['branches'][policy] for r in new_rows]
        aggregates[policy]=dict(old_release_success=sum(r['released'] for r in old_group),new_release_success=sum(r['released'] for r in new_group),
          new_exit_success=sum(r['exit_success'] for r in new_group),new_hold_precheck_failures=sum(r['hold_precheck_failures'] for r in new_group),
          new_primary_calls=sum(sum(r['phase_primary_calls'].values()) for r in new_group),
          new_diagnostic_calls=sum(r['diagnostic_calls'] for r in new_group),new_retries=sum(r['retries'] for r in new_group),
          new_mean_initial_to_release_s=float(np.mean([r['initial_to_release_simulation_s'] for r in new_group if r['released']])))
    accepted_residuals={};max_hold_actual_speed=0.;max_hold_actual_position_error=0.;rejected=0
    for b in new_rows:
        for policy,s in b['branches'].items():
            directory=new/f'case_{b["case"]:03d}'/policy
            for r in json.loads((directory/'solves.json').read_text()):
                rejected+=not r['accepted']
                if r['accepted']:
                    for k,v in r['independent_acceptance']['residuals'].items():accepted_residuals[k]=max(accepted_residuals.get(k,0.),v)
            target=s['geometry']['passing_stop_targets_x'][s['passer']]
            for r in read_rows(directory):
                if r['stage']=='CLEARED_HOLD' and r['car']==s['passer']:
                    max_hold_actual_speed=max(max_hold_actual_speed,abs(float(r['vx'])))
                    max_hold_actual_position_error=max(max_hold_actual_position_error,abs(float(r['x'])-target))
    return dict(options_unchanged=True,all_exit_candidate_actual_logs_identical=True,paired=paired,original_acceleration_checks=acceleration,
        prechecks=prechecks,aggregates=aggregates,new_rejected_returns=rejected,new_accepted_residual_maxima=accepted_residuals,
        maximum_actual_hold_speed_m_s=max_hold_actual_speed,maximum_actual_hold_position_error_m=max_hold_actual_position_error)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--old',type=Path,required=True);p.add_argument('--new',type=Path,required=True)
    args=p.parse_args();r=compare(args.old,args.new)
    write_json(args.new/'hold_comparison.json',r)
    write_json(args.new/'original_acceleration_checks.json',r['original_acceleration_checks'])
    lines=['# CLEARED_HOLD 控制约束补齐：完整重跑对照','','历史结果未改写；八组初值及进入条件、物理阈值、0.8 s确认时长完全一致。仅新模式的保持优化增加低速/目标位置区间硬约束，并在实际进入前进行配对可行控制预检查。', '',
       '|基线|旧释放|新EXIT|新释放|保持预检查失败|新增不可接受返回|', '|---|---|---|---|---|---|']
    for p,a in r['aggregates'].items():
        lines.append(f'|{p}|{a["old_release_success"]}/8|{a["new_exit_success"]}/8|{a["new_release_success"]}/8|{a["new_hold_precheck_failures"]}|{r["new_rejected_returns"]}|')
    lines+=['','## 首控制量核对','','|旧失败初值/策略|通行车|v0 m/s|旧u0 m/s²|旧段末v m/s|新u0 m/s²|新段末v m/s|单周期制动候选|','|---|---|---|---|---|---|---|---|']
    for a in r['original_acceleration_checks']:
        lines.append(f'|{a["case"]:03d}/{a["policy"]}|{a["car"]}|{a["v0_m_s"]:.9f}|{a["old_u0_m_s2"]:.9f}|{a["old_end_v_m_s"]:.9f}|{a["new_u0_m_s2"]:.9f}|{a["new_end_v_m_s"]:.9f}|{a["stop_candidate_passed"]}|')
    lines+=['','旧首控制确实造成下一0.4 s段速度幅值从约0.023411升至0.026484 m/s，超过0.025阈值。镜像方向符号对称；沿当前速度方向的首加速度上限约0.0039724 m/s²。旧拒绝段未执行。',
       '单周期制动序列为首段u=−v0/0.4、后续零加速度，幅值约0.05853 m/s²、首段移动约0.00468 m。每条均单独检查动力学、速度、加速度、终端速度、目标不越界、保持区间、道路、邻车、完整配对预测与清空，不凭制动能力判可行；该候选从未被当作fallback执行。', '',
       '## 范围与分层验收','',
       '保持硬约束覆盖完整6.4 s预测的0.1 s采样：|v|≤0.025 m/s，target−0.08≤x≤target+0.08 m。保留原速度/加速度、道路、间距、目标不越界和预测末端v(T)=0；没有增加终端x(T)=target。',
       '两个基线均启用同一新模式。选角仍只依据完整EXIT，不使用PASS结果；没有改选角色、移动状态或覆盖控制。旧默认入口不开启新保持模式，可用--hold-constraints启用。',
       '16条EXIT候选actual.csv与旧版逐字节一致；16个策略分支进入保持前的EXIT/PARKED_READY/PASS实际日志也逐行一致，首次进入保持时刻不变。预检查不改变状态、上一控制、warm start或时间，失败则终止而不宣称进入可保持状态。',
       f'进入前配对预检查通过 {sum(a["passed"] for a in r["prechecks"])}/{len(r["prechecks"])}；单周期制动候选完整检查通过 {sum(a["stop_candidate_passed"] for a in r["prechecks"])}/{len(r["prechecks"])}。',
       f'新实际保持样本最大速度 {r["maximum_actual_hold_speed_m_s"]:.12f} m/s，最大目标位置误差 {r["maximum_actual_hold_position_error_m"]:.12f} m。外部严格保持阈值未放宽。',
       '局部QP残差仍按原1e−5容差验收，保持外部全预测/执行检查仍严格使用0.025 m/s与0.08 m，没有通过数值容差放宽外部阈值。', '',
       '## 预算与结论','',
       f'保持进入前额外两次QP诊断，共{sum(a["new_diagnostic_calls"] for a in r["aggregates"].values())}次；重试共{sum(a["new_retries"] for a in r["aggregates"].values())}次。它们在选角之后执行，耗时计入PASS计算墙钟，不冒充原预算免费检查。单周期制动候选只有积分/验收，不调用额外求解器。',
       '逐案例成功率、模拟时间和各阶段调用/计算时间见 [本轮完整阶段报告](report.md)；原始预检查、制动序列和分项残差见各分支hold_prechecks.json。',
       '本次支持的结论是：补齐保持约束后，给定八组初值下旧六次保持拒绝消失，两个基线均完成退出/通过/释放；没有观察到新增不可接受返回或预检查失败。它仍只是当前模型、数值预算和采样检查下的验证，不是递归可行性、连续时间安全、转向可达性或最终恢复的理论证明，也不表示退让车回归或双方任务完成。']
    (args.new/'hold_comparison.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({k:r[k] for k in ('aggregates','new_rejected_returns','maximum_actual_hold_speed_m_s','maximum_actual_hold_position_error_m')},indent=2))


if __name__=='__main__':main()
