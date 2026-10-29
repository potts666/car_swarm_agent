#!/usr/bin/env python3
"""Same-scene B1/B2 single-linearization/bounded-update comparison."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import time
import numpy as np
import scipy
import osqp
from bicycle_core import validate_scene,CARS
from longitudinal_joint import Controller,routes
from linearized_joint import BicycleController,STATE_TRUST,INPUT_TRUST,ERROR_LIMIT,STATE_WEIGHT,INPUT_WEIGHT,DEADLINE_S,MAX_UPDATES
from ros_flag import RosFlag
from run_joint_e2 import run_b1,verify_ros,verify_future_execution
from run_entry_bicycle import ROOT,write
from verify_entry_bicycle import verify_policy


def read_lines(path):return [json.loads(line) for line in path.read_text().splitlines()]


def summarize(directory,scene):
    summary=json.loads((directory/'summary.json').read_text());validation=json.loads((directory/'validation.json').read_text())
    ros=json.loads((directory/'ros_validation.json').read_text());solves=read_lines(directory/'solves.jsonl')
    planning=json.loads((directory/'planning.json').read_text());witnesses=read_lines(directory/'future_diagnostics.jsonl')
    is_b2=directory.name!='B1';subproblems=[r for s in solves for r in s.get('subproblems',[])]
    future_solves=[s for w in witnesses for s in w['solves']]
    times=[p['total_cycle_wall_s'] for p in planning];effort=0.;endpoint={}
    rows=list(csv.DictReader((directory/'actual.csv').open()))
    for car in CARS:
        rr=[r for r in rows if r['car']==car]
        for r in rr[1:]:effort+=scene['sample_s']*(float(r['a'])**2+float(r['omega'])**2)
        final=np.array([float(rr[-1][k]) for k in ('x','y','psi','v','delta')]);goal=np.array(scene['goals'][car])
        endpoint[car]=dict(position_m=float(np.linalg.norm(final[:2]-goal[:2])),heading_rad=float(abs(final[2]-goal[2])),speed_m_s=float(abs(final[3])),steering_rad=float(abs(final[4])))
    errors={k:max((e.get(k,0.) for s in solves for e in s.get('mapping_errors',{}).values()),default=0.) for k in ('position_m','heading_rad','speed_m_s','steering_rad')}
    if is_b2:
        main_reference=sum(s.get('reference_numeric_qp_calls',0) for s in solves)
        main_bicycle=sum(s.get('bicycle_numeric_qp_calls',0) for s in solves)
        future_reference=sum(s.get('reference_numeric_qp_calls',0) for s in future_solves)
        future_bicycle=sum(s.get('bicycle_numeric_qp_calls',0) for s in future_solves)
        histogram={str(k):sum(s.get('qp_updates',0)==k for s in solves) for k in range(MAX_UPDATES+1)}
        accepted_sub=sum(r['accepted'] for r in subproblems)
    else:
        main_reference=sum(s['free_variables']>0 for s in solves);main_bicycle=0
        future_reference=sum(s['free_variables']>0 for s in future_solves);future_bicycle=0
        histogram={'0':len(solves)};accepted_sub=sum(s['accepted'] for s in solves)
    accepted_cycles=sum(s['accepted'] for s in solves)
    result=dict(completed=summary['both_completed'],unified_validation_passed=validation['passed'],ROS_validation_passed=ros['passed'],
        failure=summary['failure'],completion_sim_s=summary.get('T_complete_sim_s'),attempted_control_cycles=len(solves),executed_control_cycles=len(planning),
        accepted_control_cycles=accepted_cycles,cycle_acceptance_fraction=accepted_cycles/len(solves) if solves else 0.,
        minimum_sampled_gap_m=validation.get('trajectory',{}).get('minimum_gap_m'),
        normalized_execution_effort=effort,endpoint_errors=endpoint,maximum_model_interface_errors=errors,maximum_position_model_error_m=errors['position_m'],maximum_heading_model_error_rad=errors['heading_rad'],maximum_speed_model_error_m_s=errors['speed_m_s'],
        main_reference_numeric_QPs=main_reference,main_bicycle_numeric_QPs=main_bicycle,
        future_reference_numeric_QPs=future_reference,future_bicycle_numeric_QPs=future_bicycle,
        update_count_histogram=histogram,accepted_subproblems=accepted_sub,attempted_subproblems=len(subproblems) if is_b2 else len(solves),
        maximum_QPs_per_main_cycle=max((s.get('reference_numeric_qp_calls',0)+s.get('bicycle_numeric_qp_calls',0) for s in solves),default=0) if is_b2 else 1,
        sequential_update_cycles=sum(s.get('qp_updates',0)>1 for s in solves),
        deadline_rejections=sum(s.get('deadline_exceeded',False) for s in solves),
        hessian_min_eigenvalue=min((r['hessian_min_eigenvalue'] for r in (subproblems if is_b2 else solves)),default=None),
        maximum_affine_violation=max((r['affine_violation'] for r in (subproblems if is_b2 else solves)),default=None),
        cycle_mean_ms=float(np.mean(times)*1000) if times else None,cycle_max_ms=max(times,default=0.)*1000,
        cycles_over_200ms=sum(t>scene['h'] for t in times),
        prediction_planning_mean_ms=float(np.mean([s['solve_wall_s'] for s in solves])*1000) if solves else None,
        future_check_wall_s=sum(w['wall_s'] for w in witnesses),
        total_policy_wall_s=summary['total_policy_wall_s'],scope='one scene; fractions are cycle counts, not scenario success probabilities')
    write(directory/'metrics_e3.json',result)
    return result


def report(output,results):
    fields=['completed','unified_validation_passed','ROS_validation_passed','failure','completion_sim_s',
        'cycle_acceptance_fraction','maximum_position_model_error_m','maximum_heading_model_error_rad','maximum_speed_model_error_m_s','minimum_sampled_gap_m','normalized_execution_effort',
        'main_reference_numeric_QPs','main_bicycle_numeric_QPs','future_reference_numeric_QPs','future_bicycle_numeric_QPs',
        'sequential_update_cycles','deadline_rejections','hessian_min_eigenvalue','maximum_affine_violation',
        'prediction_planning_mean_ms','cycle_mean_ms','cycle_max_ms','cycles_over_200ms','future_check_wall_s']
    table=['|指标|B1|B2 单次|B2 最多三次更新|','|---|---|---|---|']
    names=['B1','B2_single','B2_sequence']
    for key in fields:
        vals=[results[n][key] for n in names]
        table.append('|'+key+'|'+'|'.join(f'{v:.9g}' if isinstance(v,float) else str(v) for v in vals)+'|')
    write(output/'comparison.json',results)
    with (output/'comparison.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=['method',*fields]);writer.writeheader()
        for n in names:writer.writerow(dict(method=n,**{key:results[n][key] for key in fields}))
    eligible=[n for n in names if results[n]['completed'] and results[n]['unified_validation_passed'] and results[n]['ROS_validation_passed']]
    selected=min(eligible,key=lambda n:(results[n]['cycle_max_ms'],results[n]['prediction_planning_mean_ms'])) if eligible else None
    # Lower worst-cycle cost wins only after full task and independent validation.
    recommendation=dict(selected_inner_method=selected,rule='complete original tasks, pass unified and ROS validation, then minimum measured maximum full cycle cost; ties by mean prediction-planning cost',
        retained_comparators=[n for n in names if n!=selected],real_time_proved=False,single_scene_only=True)
    write(output/'method_selection.json',recommendation)
    lines=['# E3：时变仿射自行车联合 QP 对照','','\n'.join(table),'',
        '同一原场景、地图、目标、车辆能力、固定路径阶段和 A→B ROS Flag 顺序。B1 重新运行；B2_single 与 B2_sequence 仅在线性化更新次数上不同，误差阈值和走廊/信赖域相同。',
        'B2 先调用一次 B1 纵向参考 QP，再求解保留 x/y/ψ/v/δ 的 128 输入变量联合 QP。参考生成的 QP 数量和耗时计入全部主周期与未来诊断成本，不能把 B2 称为只需一次总 QP。',
        '每轮固定全部凸道路走廊、保护区分离半空间、双方分离轴及初始参考的航向车身包络。序列更新只重建仿射动态与局部信赖域，不在子问题内优化走廊选择。',
        '单次与序列采用相同非线性误差阈值：位置 1e-4 m、航向 5e-5 rad、速度及转角 1e-6。阶段衔接的原停车/转向和完整未来释放检查保留。',
        'prediction_planning_mean_ms 含参考生成、走廊、所有 B2 更新及非线性重演。cycle_max_ms 还包含完整未来释放预演；不能用子问题求解耗时声称在线实时。',
        'normalized_execution_effort=Σ sample_s·[(a/1 m/s²)²+(ω/1 rad/s)²]，各方法同口径。内部优化目标不同，不直接比较内部 objective_without_constant。失败分支的控制成本和最小间距只覆盖已执行前缀，不能当作完成任务的收益。',
        '每周期 QP 数、每次更新状态/残差/模型误差见各分支 solves.jsonl；线性预测、固定走廊、信赖域与实际控制均保存。单场景的周期验收比例不是场景成功概率。',
        '单个子问题是凸 QP；序列求解不表示原非线性问题全局凸或全局最优。统一验收为原非线性方程、旋转车身、道路/双方间距/权限、原目标和完整未来见证的 0.02 s 采样重放。',
        '本轮依据完整任务验收后最小实测最长完整周期选择后续内层方法：'+str(selected)+'；其余保留对照。选择限于当前场景，不是一般性能排序。',
        '数学推导、有效域和固定参数见 mathematics.md；版本、准确命令和源码快照见 provenance.json/source/。']
    (output/'report.md').write_text('\n\n'.join(lines)+'\n')
    render(output,names)


def render(output,names):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,3,figsize=(16,11))
    for col,name in enumerate(names):
        rows=list(csv.DictReader((output/name/'actual.csv').open()))
        for car,color in [('A','tab:blue'),('B','tab:orange')]:
            rr=[r for r in rows if r['car']==car];t=[float(r['time_s'])-100 for r in rr]
            axes[0,col].plot([float(r['x']) for r in rr],[float(r['y']) for r in rr],color=color,label=car)
            axes[1,col].plot(t,[float(r['v']) for r in rr],color=color,label=car)
            axes[2,col].plot(t,[float(r['delta']) for r in rr],color=color,label=car)
        axes[0,col].set(title=name,xlabel='x (m)',ylabel='y (m)',ylim=(-6,6))
        axes[1,col].set(xlabel='simulation time (s)',ylabel='speed (m/s)')
        axes[2,col].set(xlabel='simulation time (s)',ylabel='steering (rad)')
        for ax in axes[:,col]:ax.grid(True);ax.legend()
    fig.tight_layout();fig.savefig(output/'comparison.png',dpi=140);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    scene=json.loads((ROOT/'config/entry_bicycle.json').read_text());validate_scene(scene);path=routes(scene)
    write(args.output/'scene.json',scene);write(args.output/'paths.json',path)
    files=[*Path(__file__).parent.glob('*.py'),ROOT/'config/entry_bicycle.json',ROOT/'tests/test_linearized_joint.py',ROOT/'bicycle_joint_qp.md',ROOT/'requirements-qp.txt']
    source=args.output/'source';source.mkdir()
    for p in files:shutil.copy2(p,source/p.name)
    shutil.copy2(ROOT/'bicycle_joint_qp.md',args.output/'mathematics.md')
    write(args.output/'provenance.json',dict(git_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files},python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,osqp=osqp.__version__,
        command='source /opt/ros/jazzy/setup.bash && PYTHONPATH=/tmp/e2_qp_deps:$PYTHONPATH ROS_DOMAIN_ID='+os.environ.get('ROS_DOMAIN_ID','0')+' OPENBLAS_NUM_THREADS=1 python3 scripts/run_bicycle_e3.py --output '+str(args.output),
        state_trust=STATE_TRUST.tolist(),input_trust=INPUT_TRUST.tolist(),error_limits=ERROR_LIMIT,state_weights=STATE_WEIGHT.tolist(),input_weights=INPUT_WEIGHT.tolist(),
        prediction_deadline_s=DEADLINE_S,maximum_updates=MAX_UPDATES,future_check_deadline_s=30.,no_realtime_or_global_optimality_claim=True))
    results={}
    for name,updates in [('B1',0),('B2_single',1),('B2_sequence',MAX_UPDATES)]:
        print('Running',name,flush=True);bus=RosFlag()
        try:
            controller=Controller(scene,path) if not updates else BicycleController(scene,path,updates)
            run_b1(scene,path,args.output/name,bus,controller=controller)
        finally:bus.close()
        config=json.loads((args.output/name/'config.json').read_text());config['controller']=name;write(args.output/name/'config.json',config)
        try:validation=verify_policy(args.output/name,scene)
        except (ValueError,KeyError,StopIteration,IndexError) as exc:validation=dict(passed=False,validation_error=type(exc).__name__+': '+str(exc))
        consistency=verify_future_execution(args.output/name,scene);write(args.output/name/'future_execution_consistency.json',consistency)
        write(args.output/name/'validation.json',validation);verify_ros(args.output/name)
        results[name]=summarize(args.output/name,scene)
        print(name,json.dumps(results[name]),flush=True)
    report(args.output,results)
    if not results['B1']['completed'] or not results['B1']['unified_validation_passed']:raise SystemExit(1)


if __name__=='__main__':main()
