#!/usr/bin/env python3
"""B0/B1 fixed-scene comparison with ROS 2 Flag and bicycle execution."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import time
import numpy as np
import scipy
import osqp

from bicycle_core import CARS,SIGN,audit,completed,clipped_area,corners,validate_scene
from longitudinal_joint import Controller,WEIGHTS,routes
from ros_flag import RosFlag
from run_entry_bicycle import ROOT,run_policy,rows_for,save_rows,write,jsonl
from verify_entry_bicycle import verify_policy


def run_b1(scene,path,directory,bus):
    directory.mkdir();controller=Controller(scene,path)
    rows=[];events=[];forecasts=[];checks=[];planning=[];solves=[];constraints=[];witnesses=[]
    owner=None;requested=set();released=set();ready_sent=False;done={};reserved=0.;failure=None
    begun=time.perf_counter();now=scene['epoch_s']
    def event(kind,**fields):events.append(dict(time_s=now,kind=kind,**fields))
    try:
        for cycle in range(round(scene['timeout_s']/scene['h'])):
            cycle_started=time.perf_counter();now=round(scene['epoch_s']+cycle*scene['h'],10);bus.cycle=cycle
            requests=[(now,c) for c,a in controller.agents.items() if c not in requested and
                      SIGN[c]*a.z[0]>=-scene['request_rear_axle_abs_x']-1e-8]
            if requests:
                owner,grants=bus.request(requests)
                for _,c in requests:
                    requested.add(c);event('request',car=c)
                    event('grant' if grants[c] else 'deny',car=c,owner=owner)
            permissions={c:bus.received[c]['owner_id']==c for c in CARS}
            controller.prepare(permissions)
            b=controller.agents['B']
            if b.mode=='WAIT' and not b.steering and abs(b.z[4])<1e-7 and not ready_sent:
                bus.ready('B');ready_sent=True;event('waiting_ready',car='B',pose=b.z.tolist())
            for c,a in controller.agents.items():
                if a.mode=='DONE' and c not in done:
                    if not completed(a.z,scene['goals'][c],scene):raise ValueError('completed phase missed original goal')
                    done[c]=now-scene['epoch_s'];event('car_completed',car=c)
            if owner and owner not in released:
                a=controller.agents[owner]
                tail=SIGN[owner]*a.z[0]-scene['vehicle']['rear']
                if tail>=scene['release_plane_abs_x']+scene['release_margin_m'] and (owner=='B' or ready_sent):
                    witness=controller.future_witness();witnesses.append(dict(time_s=now,holder=owner,**witness))
                    checks.append(dict(time_s=now,kind='complete_future_release_witness',wall_s=witness['wall_s'],**witness['check']))
                    if not witness['check']['passed']:
                        failure='full_future_release_QP_rejected';event('failure',reason=failure);break
                    jsonl(directory/f'release_witness_{owner}.jsonl',[{k:v for k,v in witness.items() if k!='solves'}])
                    event('release',car=owner,actual_body_tail_x=float(a.z[0]-SIGN[owner]*scene['vehicle']['rear']))
                    released.add(owner);owner=bus.release(owner,witness['check'])
                    if owner:
                        event('grant',car=owner,owner=owner);event('return_started',car=owner)
                    permissions={c:bus.received[c]['owner_id']==c for c in CARS};controller.prepare(permissions)
            if len(done)==2:
                event('both_tasks_completed');break
            states={c:a.z.copy() for c,a in controller.agents.items()}
            controls,predictions,record=controller.predict(permissions)
            record.update(time_s=now,cycle=cycle)
            solves.append(record)
            for c in CARS:
                constraints.append(dict(car=c,cycle=cycle,g_i=int(permissions[c]),
                    receive_cycle=bus.received[c]['receive_cycle'],state_version=bus.received[c]['version'],
                    phase=controller.agents[c].mode,segment=controller.agents[c].index,**record['parameters'][c]))
            if not record['accepted']:
                failure='joint_QP_rejected';event('failure',reason=failure);break
            if any(max(v for k,v in e.items() if k!='path_s_end_m')>scene['tolerance'] for e in record['mapping_errors'].values()):
                failure='QP_bicycle_mapping_error';event('failure',reason=failure);break
            planning_wall=time.perf_counter()-cycle_started
            start=time.perf_counter();check=audit(predictions,scene)
            checks.append(dict(time_s=now,kind='dense_prediction',wall_s=time.perf_counter()-start,**check))
            n=round(scene['h']/scene['sample_s'])
            forecasts.append(dict(time_s=now,states={c:z.tolist() for c,z in states.items()},commands=controls,
                predicted_states={c:z[n::n].tolist() for c,z in predictions.items()},owner=owner))
            if not check['passed']:
                failure='prediction_acceptance_rejected';event('failure',reason=failure);break
            protection=[-scene['admission_protection_abs_x'],scene['admission_protection_abs_x'],-scene['W']/2,scene['W']/2]
            for c in CARS:
                if not permissions[c] and any(clipped_area(body,protection)>scene['tolerance'] for body in corners(predictions[c],scene['vehicle'])):
                    failure='unauthorized_prediction';event('failure',reason=failure,car=c);break
            if failure:break
            motions={c:z[:n+1] for c,z in predictions.items()}
            executed=audit(motions,scene);checks.append(dict(time_s=now,kind='executed_segment',wall_s=0.,**executed))
            if not executed['passed']:
                failure='actual_acceptance_rejected';event('failure',reason=failure);break
            rows.extend(rows_for(motions,{c:[controls[c][0]] for c in CARS},now,scene,owner))
            controller.commit(controls,predictions,record);reserved+=scene['h']*(owner is not None)
            planning.append(dict(time_s=now,planning_wall_s=planning_wall,acceptance_wall_s=time.perf_counter()-start,
                total_cycle_wall_s=time.perf_counter()-cycle_started,optimization_solver_calls=1,retries=0))
        else:
            failure='timeout';event('failure',reason=failure)
    except (ValueError,TimeoutError) as exc:
        failure=type(exc).__name__+': '+str(exc);event('failure',reason=failure)
    if not rows:
        rows=rows_for({c:np.array([a.z]) for c,a in controller.agents.items()},{c:[] for c in CARS},scene['epoch_s'],scene)
    save_rows(directory/'actual.csv',rows)
    for name,values in [('events',events),('predictions',forecasts),('checks',checks),('solves',solves),
                        ('constraints',constraints),('future_diagnostics',witnesses),('messages',[]),('detections',[])]:
        jsonl(directory/(name+'.jsonl'),values)
    jsonl(directory/'ros_messages.jsonl',bus.log)
    write(directory/'planning.json',planning);write(directory/'config.json',dict(scene=scene,policy='S1',controller='B1'))
    summary=dict(policy='S1',controller='B1',both_completed=len(done)==2 and failure is None,
        T_complete_sim_s=max(done.values()) if len(done)==2 and failure is None else None,
        completion_per_car_s=done,channel_reserved_s=reserved,optimization_solver_calls=len(solves),
        future_diagnostic_qp_calls=sum(w['qp_calls'] for w in witnesses),
        future_diagnostic_wall_s=sum(w['wall_s'] for w in witnesses),failure=failure,
        total_policy_wall_s=time.perf_counter()-begun,continuous_time_safety_proved=False)
    write(directory/'summary.json',summary)
    return summary


def verify_ros(directory):
    messages=[json.loads(l) for l in (directory/'ros_messages.jsonl').read_text().splitlines()]
    states=[m for m in messages if m['kind']=='publish_state' and m['owner_id']]
    order=list(dict.fromkeys(m['owner_id'] for m in states))
    errors=[]
    if order!=['A','B']:errors.append('grant order or incomplete handoff: '+str(order))
    if 'B' in order:
        grant=next(m for m in states if m['owner_id']=='B')
        released=[m for m in messages if m['kind']=='receive_release' and m['vehicle_id']=='A']
        ready=[m for m in messages if m['kind']=='receive_ready' and m['vehicle_id']=='B']
        if not released or not ready or not (released[0]['sequence']<grant['sequence'] and ready[0]['sequence']<grant['sequence']):
            errors.append('B granted before ready or A release')
    constraints=[json.loads(l) for l in (directory/'constraints.jsonl').read_text().splitlines()]
    for p in constraints:
        version=p['state_version'];car=p.get('car')
        received=[m for m in messages if m['kind']=='receive_state' and m['vehicle_id']==car and m['version']==version]
        effective=p.get('effective_cycle',p.get('cycle'))
        if not received or p['g_i']!=int(received[-1]['owner_id']==car) or received[-1]['cycle']>effective:
            errors.append('constraint does not match received permission');break
    result=dict(passed=not errors,grant_order=order,errors=errors,actual_DDS_log=True)
    write(directory/'ros_validation.json',result)
    return result


def comparison(directory):
    result={}
    for name in ('B0','B1'):
        d=directory/name;s=json.loads((d/'summary.json').read_text());v=json.loads((d/'validation.json').read_text())
        planning=json.loads((d/'planning.json').read_text());solves=[json.loads(l) for l in (d/'solves.jsonl').read_text().splitlines()] if (d/'solves.jsonl').exists() else []
        compute=[p['total_cycle_wall_s'] for p in planning];solve_time=[p['solve_wall_s'] for p in solves]
        metrics=v.get('metrics',{})
        result[name]=dict(completed=s['both_completed'],validation_passed=v['passed'] and json.loads((d/'ros_validation.json').read_text())['passed'],failure=s['failure'],
            completion_sim_s=s.get('T_complete_sim_s'),minimum_sampled_gap_m=v.get('trajectory',{}).get('minimum_gap_m'),
            distance_m=metrics.get('total_distance_m'),reverse_distance_m=metrics.get('total_reverse_distance_m'),
            planning_calls=len(planning),QP_calls=s.get('optimization_solver_calls',0),
            future_diagnostic_QP_calls=s.get('future_diagnostic_qp_calls',0),
            future_diagnostic_wall_s=s.get('future_diagnostic_wall_s',0.),
            cycle_mean_ms=float(np.mean(compute)*1000) if compute else None,cycle_max_ms=max(compute,default=0.)*1000,
            cycle_over_200ms=sum(t>.2 for t in compute),solve_mean_ms=float(np.mean(solve_time)*1000) if solve_time else None,
            solve_max_ms=max(solve_time,default=0.)*1000,
            min_hessian_eigenvalue=min((r['hessian_min_eigenvalue'] for r in solves),default=None),
            maximum_affine_violation=max((r['affine_violation'] for r in solves),default=None),
            maximum_mapping_position_m=max((e['position_m'] for r in solves for e in r['mapping_errors'].values()),default=None),
            maximum_mapping_speed_m_s=max((e['speed_m_s'] for r in solves for e in r['mapping_errors'].values()),default=None),
            maximum_mapping_heading_rad=max((e['heading_rad'] for r in solves for e in r['mapping_errors'].values()),default=None),
            solve_rejections=sum(not r['accepted'] for r in solves))
    write(directory/'comparison.json',result)
    fields=['completed','validation_passed','failure','completion_sim_s','minimum_sampled_gap_m','distance_m',
            'reverse_distance_m','QP_calls','future_diagnostic_QP_calls','cycle_mean_ms','cycle_max_ms',
            'cycle_over_200ms','solve_mean_ms','solve_max_ms','min_hessian_eigenvalue',
            'maximum_affine_violation','maximum_mapping_position_m','maximum_mapping_speed_m_s',
            'maximum_mapping_heading_rad','solve_rejections']
    table=['|指标|B0 原控制|B1 联合 QP|','|---|---|---|']
    for f in fields:
        values=[result[n][f] for n in ('B0','B1')]
        table.append('|'+f+'|'+'|'.join(f'{v:.9g}' if isinstance(v,float) else str(v) for v in values)+'|')
    report=['# E2 固定路径纵向联合 QP：B0/B1','','\n'.join(table),'',
        '同一 entry_bicycle.json、初态、地图、车辆能力与目标；Flag 均为 A→B，实际 ROS 2 三节点消息。B0 每次重新运行，不读取历史轨迹作为成功依据。',
        'B1 的几何路径为原直线/双圆弧；B0 申请后的制动及接近合并为同一直线上的纵向优化。B1 在曲率断点停车转向，固定 N=32、h=0.2 s，不优化几何、顺序或时域。',
        '联合 QP 在 B 初始直线阶段包含双方进度的耦合间距行；圆弧阶段采用整段车身扫掠的保守固定 x 分离界；B 停靠后采用已检验的横向净空。模型定义见 mathematics.md。',
        '速度是沿固定段的非负路径速度，映射自行车有符号 v=d*v_path、a=d*a_path；运动时 delta 固定、omega=0，段间在静止时按原 steer() 限速转向。没有直接把路径坐标作为车辆位置写回。',
        '全部控制通过原 RK4 非线性自行车积分，以 0.02 s 采样重放分量方程、旋转车身、道路、双方间距、原目标、未授权进入及完整未来释放见证。',
        '释放前的完整未来见证再次以同一 QP 和自行车模型滚动至双方目标，诊断成本单独报告；该计算纳入所在执行周期的总墙钟，不能只用 QP 单次耗时声称实时。',
        '本轮为单场景、理想通信、仿真冻结计算的集中式 B1。道路扫掠边界是 0.01 m 几何采样，执行是 0.02 s 采样；不构成连续时间安全、递归可行性或真实车辆可行证明。',
        '源码/配置哈希、版本、环境与命令见 provenance.json/source/；逐次失败原样保留在 solves.jsonl、events.jsonl。']
    (directory/'report.md').write_text('\n\n'.join(report)+'\n')
    render(directory)
    return result


def render(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import csv
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    for col,name in enumerate(('B0','B1')):
        rows=list(csv.DictReader((directory/name/'actual.csv').open()))
        for c,color in [('A','tab:blue'),('B','tab:orange')]:
            rr=[r for r in rows if r['car']==c]
            axes[0,col].plot([float(r['x']) for r in rr],[float(r['y']) for r in rr],color=color,label=c)
            axes[1,col].plot([float(r['time_s'])-100 for r in rr],[float(r['v']) for r in rr],color=color,label=c)
        axes[0,col].set(title=name+' rear axle path',xlabel='x (m)',ylabel='y (m)',ylim=(-6,6))
        axes[1,col].set(xlabel='simulation time (s)',ylabel='signed speed (m/s)')
        for ax in axes[:,col]:ax.grid(True);ax.legend()
    fig.tight_layout();fig.savefig(directory/'comparison.png',dpi=150);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    scene=json.loads((ROOT/'config/entry_bicycle.json').read_text());validate_scene(scene);write(args.output/'scene.json',scene)
    path=routes(scene);write(args.output/'paths.json',path)
    source=args.output/'source';source.mkdir()
    files=[*Path(__file__).parent.glob('*.py'),ROOT/'config/entry_bicycle.json',ROOT/'tests/test_longitudinal_joint.py',ROOT/'longitudinal_joint_qp.md',ROOT/'requirements-qp.txt']
    for p in files:shutil.copy2(p,source/p.name)
    shutil.copy2(ROOT/'longitudinal_joint_qp.md',args.output/'mathematics.md')
    write(args.output/'provenance.json',dict(git_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        git_diff=subprocess.check_output(['git','diff'],text=True),source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,osqp=osqp.__version__,weights=WEIGHTS,
        command='source /opt/ros/jazzy/setup.bash && PYTHONPATH=/tmp/e2_qp_deps:$PYTHONPATH ROS_DOMAIN_ID=92 OPENBLAS_NUM_THREADS=1 python3 scripts/run_joint_e2.py --output '+str(args.output),
        solver='OSQP 1.1.3, max_iter 20000, eps_abs=eps_rel=1e-8, polishing=True; cold start; no retries',
        communication='three rclpy nodes, reliable DDS, ideal cycle-synchronous; simulation frozen during computation'))
    for name in ('B0','B1'):
        print('Running',name,flush=True);bus=RosFlag()
        try:
            if name=='B0':
                run_policy(scene,'S1',args.output/name,transport=bus)
                jsonl(args.output/name/'ros_messages.jsonl',bus.log)
                jsonl(args.output/name/'constraints.jsonl',bus.constraint_log)
            else:run_b1(scene,path,args.output/name,bus)
        finally:bus.close()
        try:
            validation=verify_policy(args.output/name,scene)
        except (ValueError,KeyError,StopIteration) as exc:
            validation=dict(passed=False,validation_error=type(exc).__name__+': '+str(exc))
        write(args.output/name/'validation.json',validation)
        verify_ros(args.output/name)
        print(name,'verification',validation['passed'],flush=True)
    result=comparison(args.output);print(json.dumps(result),flush=True)
    if not all(v['completed'] and v['validation_passed'] for v in result.values()):raise SystemExit(1)


if __name__=='__main__':main()
