#!/usr/bin/env python3
"""Summarize M0 retained outcomes, zero crossings, protocol use, and timing."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np


def lines(path):return [json.loads(line) for line in path.read_text().splitlines()]
def write(path,value):path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def table(headers,rows):return '\n'.join(['|'+'|'.join(headers)+'|','|'+'|'.join(['---']*len(headers))+'|',*['|'+'|'.join(map(str,row))+'|' for row in rows]])


def report(root):
    scene=json.loads((root/'scene.json').read_text());results=json.loads((root/'comparison.json').read_text())
    detail={};comparison=[]
    for name,result in results.items():
        directory=root/name;rows=list(csv.DictReader((directory/'actual.csv').open()))
        events=lines(directory/'events.jsonl');solves=lines(directory/'solves.jsonl')
        planning=json.loads((directory/'planning.json').read_text());validation=json.loads((directory/'independent_validation.json').read_text())
        crossings=[];per_car={};channel=[-scene['D']/2,scene['D']/2,-scene['W']/2,scene['W']/2]
        from bicycle_core import corners,clipped_area
        for c in ('A','B'):
            rr=[r for r in rows if r['car']==c]
            for previous,current in zip(rr,rr[1:]):
                product=float(previous['v'])*float(current['v'])
                if product < -scene['tolerance']**2:
                    cycle=round((float(current['time_s'])-scene['epoch_s'])/scene['h'])-1
                    source=next(s for s in solves if s['cycle']==cycle)
                    crossings.append(dict(car=c,time_s=float(current['time_s']),elapsed_s=float(current['time_s'])-scene['epoch_s'],
                        previous_v_m_s=float(previous['v']),next_v_m_s=float(current['v']),product_m2_s2=product,
                        criterion_m2_s2=-scene['tolerance']**2,stage=current['stage'],cycle=cycle,
                        record=source,actual_before=previous,actual_after=current))
            def event_time(kind):return next((e['time_s'] for e in events if e['kind']==kind and e.get('car')==c),None)
            request=event_time('request');grant=event_time('grant')
            states=np.array([[float(r[k]) for k in ('x','y','psi','v','delta')] for r in rr])
            occupied=[float(r['time_s']) for r,body in zip(rr,corners(states,scene['vehicle'])) if clipped_area(body,channel)>scene['tolerance']]
            entry=occupied[0] if occupied else None
            stopped=np.abs(states[:,3])<=scene['terminal']['speed_m_s']
            runs=[];start=None
            for i,parked in enumerate(stopped):
                if parked and start is None:start=float(rr[i]['time_s'])
                if start is not None and (not parked or i==len(stopped)-1):
                    end=float(rr[i-1 if not parked else i]['time_s']);runs.append(dict(start_s=start,end_s=end,duration_s=end-start));start=None
            per_car[c]=dict(request_s=request,grant_s=grant,request_to_grant_s=grant-request if grant is not None and request is not None else None,
                channel_entry_s=entry,grant_to_channel_entry_s=entry-grant if grant is not None and entry is not None else None,
                last_channel_occupied_sample_s=occupied[-1] if occupied else None,release_s=event_time('release'),completion_s=event_time('car_completed'),
                stationary_intervals_s=runs,stopped_to_moving_transitions=int(np.sum(stopped[:-1]&~stopped[1:])))
        refs=[s.get('reference_record',s) for s in solves]
        detail[name]=dict(acceptance_failure='original_velocity_sign_crossing' if crossings else result['failure'],velocity_crossings=crossings,
            per_car=per_car,all_cycle_exits_timed=validation['all_cycle_exits_timed'],
            final_cycle=planning[-1],reference_warm_calls=sum(r['warm_start_used'] for r in refs),
            reference_status_histogram={status:sum(r['message']==status for r in refs) for status in sorted({r['message'] for r in refs})},
            longest_cycles=sorted(planning,key=lambda p:p['total_cycle_wall_s'],reverse=True)[:3],
            failure_scope='sampler velocity sign criterion; no new reference rejection' if crossings else 'reference solver convergence',
            actual_sha256=hashlib.sha256((directory/'actual.csv').read_bytes()).hexdigest())
        comparison.append(dict(method=name,reached_both_goals=result['completed'],independent_full_task_passed=result['independent_full_task_passed'],
            failure=detail[name]['acceptance_failure'],completion_sim_s=result['completion_sim_s'],
            executed_cycles=result['executed_control_cycles'],timed_cycles=result['timed_cycles'],
            sign_crossings=len(crossings),min_gap_m=result['minimum_sampled_gap_m'],
            **{k:result[k] for k in ('cycle_p50_ms','cycle_p95_ms','cycle_p99_ms','cycle_max_ms','cycles_over_200ms','reference_rejections','deadline_rejections')}))
    write(root/'diagnostics.json',detail)
    with (root/'comparison.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(comparison[0]));writer.writeheader();writer.writerows(comparison)
    reproducible={}
    for method in ('B2_sequence_cold','B2_sequence_warm'):
        hashes=[detail[f'{method}_r{repeat}']['actual_sha256'] for repeat in (1,2)]
        reproducible[method]=dict(actual_trajectory_byte_identical=hashes[0]==hashes[1],sha256=hashes[0])
    write(root/'repeatability.json',reproducible)
    first=detail['B2_sequence_warm_r1'];cross=first['velocity_crossings']
    rows=[[r['method'],r['reached_both_goals'],r['independent_full_task_passed'],r['completion_sim_s'],r['timed_cycles'],
           f"{r['cycle_p50_ms']:.2f}/{r['cycle_p95_ms']:.2f}/{r['cycle_p99_ms']:.2f}/{r['cycle_max_ms']:.2f}",r['cycles_over_200ms']] for r in comparison]
    text=f'''# M0：B2 序列控制器完整闭环，冷启动与参考QP移位原始变量热启动

**M0 实验已完成，完整统一验收尚未通过。** 两轮冷启动均在cycle121（24.2 s）参考QP达20000次拒绝；两轮热启动均运行到双方原目标（81.0 s），但独立验收都在B车3个速度跨零样本处拒绝。不能将内部405个周期accepted写成完整验收成功，也不能据此前进条件宣布M0已经通过。

## 改动与控制变量

工作代码 `longitudinal_joint.py` 增加可选原始变量种子：只缓存实际提交周期的参考QP解，按车移位一格，末项补0；段索引或模式变化时丢弃该车旧种子；送入求解器前投影到当前加速度界，固定变量仍按原方法消元。不复用对偶或稀疏矩阵。BicycleController继承同一选项，候选拒绝不写缓存。未来完整预演使用深拷贝缓存，不能污染实际执行缓存。

两组共同补全finally出口计时，覆盖成功、参考拒绝、自行车拒绝、异常、释放预演和最终完成检查；每行区分实际提交、状态时刻、单调钟起止、拒绝层、预算余量与分段耗时。汇总用committed数计算执行周期，不再把失败/完成检查误计为执行。

两组均保留原单个0.2 s RK4导数、最多3次更新、所有权重、走廊、信赖域、1e-5 m等模型门限、原停车条件和数值带、20000次参考迭代上限、地图/车型/初态/目标、ROS Flag和完整未来释放检查。唯一运动算法变量是参考QP原始变量热启动。热启动后历史轨迹会变化，因此不能把闭环cycle121的迭代数直接当成T1同一冻结实例的17500次。

未修改Q1/Q2、四/六车布局或局部DMPC接口；原通道从授权即锁定owner，释放仍核对owner与原未来见证。本轮是集中式联合优化、实际ROS2三节点DDS、理想通信、计算时仿真时间冻结。

## 结果

{table(['运行','双方到目标','独立完整通过','任务时间s','计时周期数','全周期P50/P95/P99/最大ms','超过200ms周期'],rows)}

冷启动各有122条完整计时，其中121次提交和1次拒绝；热启动各有406条，其中405次提交和1次最终完成检查。冷热各两轮actual.csv均逐字节一致，见repeatability.json。这是单场景的确定性重复，不是多场景成功率。

热启动两轮没有参考QP拒绝、没有自行车QP拒绝、没有0.5 s规划超时；各12个主周期使用第二次更新。两次完整未来释放预演均通过，预演与后续实际执行的一致性检查通过。原完整验收仍为false，原因如下。

## 未通过的原判据与故障分类

原独立验收 `verify_entry_bicycle.check_series()` 使用相邻样本 `v[k]*v[k+1] < -tolerance**2` 判定gear_sign_jump；当前阈值为-1e-10 (m/s)²。原判据保留，未改为单看负速度绝对值，也未删除这一项。

{table(['经过s','实际控制cycle','阶段','前样本速度m/s','后样本速度m/s','速度乘积(m/s)²'],[[f"{e['elapsed_s']:.2f}",e['cycle'],e['stage'],f"{e['previous_v_m_s']:.9e}",f"{e['next_v_m_s']:.9e}",f"{e['product_m2_s2']:.9e}"] for e in cross])}

三次负速度绝对值约1e-7 m/s，虽小于普通1e-5物理残差容差，仍触发原乘积判据。独立两半步RK4全程累计重演与实际日志的最大状态偏差约1.2e-10，支持它确实来自执行控制积分，而非日志姿态重置。原始实际样本、对应参考/自行车求解、模型差与信赖域检查均保存在diagnostics.json，可直接冻结后继续诊断。

分类：原参考收敛障碍在这两轮没有再次出现；新暴露的是停车末端数值微小反向速度与原速度跨零验收的衔接问题。当前证据不支持称为碰撞、道路不可行或新的参考求解失败；也不支持忽略原判据宣布修复完成。依照附件的M0边界保留反例，本轮没有再叠加停车表述修改、约束放松或新求解算法。

## 独立验收范围

verify_m0.py从新日志重读实际轨迹与控制，使用独立标量方程、每0.02 s内两次0.01 s RK4重积分，既逐区间检查，也从原始初态全程累计检查。实际控制界、转角/速度界、旋转车身与道路、双方间距、原目标、申请/授权/释放、保护区未经授权进入、原完整未来见证均重新检查。另核对事件owner与每个实际控制区间owner，提交数量与实际采样数，以及所有出口计时。

两轮热启动的动力学/控制界/道路/间距/授权/释放/原目标均通过，最小采样净空{results['B2_sequence_warm_r1']['minimum_sampled_gap_m']:.9f} m；B车终点位置误差约5.825e-5 m。完整验收唯一失败项为B车gear_sign_jump。几何审计函数复用现有实现，不能将这称为第二套独立几何算法；积分与日志核对独立于执行积分器和内部accepted字段。冷启动只有已执行前缀通过，并未完成授权交接，不能与热启动完整成本排名。

## 时间、协议与结论边界

完整周期最大约13 s，包含释放前双方剩余完整任务同步预演；两轮均有2个周期超过0.2 s。参考求解、全部自行车更新、非线性验收和实际提交均计入主周期总时长；参考solver_wall_s另列。规划0.5 s预算保持不变，不能用这一预算或未超时来声称满足0.2 s控制周期。

每车申请到授权、授权到实际通道进入、最后占用样本、释放与完成、连续静止区间、停后再运动次数见diagnostics.json。保留stationary区间作为停滞诊断，不把授权等待或正常停车自动标为死锁。未在本轮引入新放行优先级、批次、延迟或备用控制。

M0没有为M2局部DMPC提供已经通过统一验收的运动实现。下一项具体问题是这三个停车末端样本的执行速度与既有跨零判据如何在不重置状态、不放松原验收的条件下衔接；它应作为独立改动再对照，而不是混入本轮热启动收益。

## 工件与复现

- scene.json、paths.json、source/、provenance.json、code_changes.patch：配置、运行时源码快照、环境/线程、精确命令和哈希。
- 四个运行目录：actual.csv、solves/predictions/constraints/events/ros_messages、planning.json、future_diagnostics、独立验收与未来一致性结果。
- comparison.json/csv、diagnostics.json、repeatability.json：结果、失败样本和可重复性。
- mainline_plan_v1.txt：用户要求原文，历史B1/B2/T1目录均未覆盖。

```bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH=/tmp/t1_qp_deps:$PYTHONPATH OPENBLAS_NUM_THREADS=1 ROS_DOMAIN_ID=96 ROS_LOG_DIR=/tmp/m0_ros_logs python3 examples/head_on_mpc/scripts/run_m0.py --output examples/head_on_mpc/results/M0_002
python3 examples/head_on_mpc/scripts/report_m0.py --run examples/head_on_mpc/results/M0_002
python3 examples/head_on_mpc/scripts/verify_m0.py --run examples/head_on_mpc/results/M0_002/B2_sequence_warm_r1
```

报告器需要numpy；运行器还需原ROS2/OSQP1.1.3依赖，/tmp临时依赖目录需在重启或换机后重建。新运行使用不存在的新目录。

未验证连续时间安全、递归可行性、真实延迟、备用控制或DMPC；现有计算期间车辆状态冻结，退出仿真不等同实车安全处置。
'''
    (root/'report.md').write_text(text)
    print('M0 report, comparison CSV, diagnostics and repeatability generated')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True);args=parser.parse_args();report(args.run)
