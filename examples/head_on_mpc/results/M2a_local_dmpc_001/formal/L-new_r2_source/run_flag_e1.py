#!/usr/bin/env python3
"""E1 real ROS 2 permission integration using existing bicycle execution."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time

from bicycle_core import validate_scene
from run_entry_bicycle import run_policy, write, jsonl, ROOT
from ros_flag import RosFlag
from verify_entry_bicycle import verify_policy


def verify_messages(messages, constraints):
    grants = [m['owner_id'] for m in messages if m['kind']=='publish_state' and m['owner_id']]
    assert list(dict.fromkeys(grants)) == ['A', 'B'], grants
    release_a = next(m for m in messages if m['kind']=='receive_release' and m['vehicle_id']=='A')
    grant_b = next(m for m in messages if m['kind']=='publish_state' and m['owner_id']=='B')
    ready_b = next(m for m in messages if m['kind']=='receive_ready' and m['vehicle_id']=='B')
    assert release_a['sequence'] < grant_b['sequence'] and ready_b['sequence'] < grant_b['sequence']
    for p in constraints:
        state = next(m for m in messages if m['kind']=='receive_state' and
                     m['vehicle_id']==p['car'] and m['version']==p['state_version'])
        assert p['g_i']==int(state['owner_id']==p['car'])
        assert state['cycle']==p['receive_cycle']<=p['effective_cycle']
        assert p['prediction_accepted']
    before = next(p for p in constraints if p['car']=='B' and p['phase']=='MUTEX_WAIT')
    after = next(p for p in constraints if p['car']=='B' and p['g_i']==1)
    assert before['g_i']==0 and before['s_max_m']<1e-5
    assert after['s_max_m']>before['s_max_m'] and after['phase']=='RETURNING'
    return dict(passed=True, B_wait=before, B_granted=after,
                release_A_cycle=release_a['cycle'], grant_B_cycle=grant_b['cycle'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args=parser.parse_args()
    scene=json.loads((ROOT/'config/entry_bicycle.json').read_text());validate_scene(scene)
    args.output.mkdir(parents=True, exist_ok=False)
    write(args.output/'scene.json', scene)
    snapshot=args.output/'source';snapshot.mkdir()
    sources=[*Path(__file__).parent.glob('*.py'), ROOT/'config/entry_bicycle.json',
             ROOT/'tests/test_flag_e1.py', ROOT/'CMakeLists.txt', ROOT/'package.xml']
    for p in sources:
        shutil.copy2(p, snapshot/p.name)
    write(args.output/'provenance.json', dict(
        git_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        git_diff=subprocess.check_output(['git','diff'],text=True),
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        command='source /opt/ros/jazzy/setup.bash; ROS_DOMAIN_ID=91 OPENBLAS_NUM_THREADS=1 python3 scripts/run_flag_e1.py --output '+str(args.output),
        transport='rclpy / DDS reliable topics; three nodes in one process',
        execution='original nonlinear bicycle primitives; simulation freezes during communication'))
    bus=RosFlag();started=time.perf_counter()
    try:
        summary=run_policy(scene,'S1',args.output/'S1',transport=bus)
    finally:
        jsonl(args.output/'ros_messages.jsonl',bus.log)
        jsonl(args.output/'constraints.jsonl',bus.constraint_log)
        bus.close()
    print('Motion completed:',summary['both_completed'],flush=True)
    verification=verify_policy(args.output/'S1',scene)
    verification['logical_message_log_scope']='S1/messages.jsonl is inherited logical diagnostics; actual DDS evidence is ros_messages.jsonl'
    write(args.output/'validation.json',verification)
    protocol=verify_messages(bus.log,bus.constraint_log)
    write(args.output/'constraint_transition.json',protocol)
    timing=json.loads((args.output/'S1/planning.json').read_text())
    result=dict(communication_complete=protocol['passed'], constraint_interface_complete=True,
        optimization_closed_loop_complete=False, both_tasks_completed=summary['both_completed'],
        validation_passed=verification['passed'], wall_s=time.perf_counter()-started,
        cycle_mean_s=sum(p['total_cycle_wall_s'] for p in timing)/len(timing),
        cycle_max_s=max(p['total_cycle_wall_s'] for p in timing),
        cycle_over_h=sum(p['total_cycle_wall_s']>scene['h'] for p in timing),
        control_period_s=scene['h'], optimized_solver_calls=0)
    write(args.output/'summary.json',result)
    (args.output/'report.md').write_text('# E1 ROS 2 Flag 接入结果\n\n'+
        '通信接入：完成。约束接口：完成。实际优化闭环：未完成，属于 E2/E3。\n\n'+
        '原场景使用三个 ROS 2 节点，经 DDS 完成申请、通道状态接收、等待就绪、完整未来检查后释放与交接。'+
        '车辆按实际订阅的 owner_id 构造 g_i，约束先验收全预测再执行。\n\n'+
        '原控制器为非线性自行车运动原语，优化求解次数为零。纵向 QP 接口给出相对于剩余固定参考的弧长进度上界；'+
        '等待终点停车，许可后开放返回与通行路径。道路并集和保护区排除仍是几何接口，未声称已转成凸走廊矩阵。\n\n'+
        'B 停驻等待上界：'+str(protocol['B_wait']['s_max_m'])+' m；获准后：'+str(protocol['B_granted']['s_max_m'])+' m。'+
        '接收周期 '+str(protocol['B_granted']['receive_cycle'])+'，控制生效周期 '+str(protocol['B_granted']['effective_cycle'])+'。\n\n'+
        '详细验收与计时：summary.json、validation.json；实际消息序列：ros_messages.jsonl；'+
        '逐周期约束：constraints.jsonl；原轨迹、预测、释放见证：S1/；源码版本、快照和命令：provenance.json、source/。\n\n'+
        '理想通信、单进程三节点、仿真等待消息时暂停。仅 0.02 s 采样验收，不证明连续时间安全或在线实时性。\n',encoding='utf-8')
    print(json.dumps(result),flush=True)
    if not summary['both_completed'] or not verification['passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
