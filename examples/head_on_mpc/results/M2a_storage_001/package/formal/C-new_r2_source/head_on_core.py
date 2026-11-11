"""Head-on longitudinal MPC model and independent closed-loop simulation."""
import csv
from dataclasses import dataclass
import json
import time

import numpy as np
from scipy.optimize import minimize


@dataclass(frozen=True)
class Settings:
    dt: float = 0.4
    horizon: int = 16
    substeps: int = 4
    epoch: float = 100.0
    max_speed: float = 2.0
    max_acceleration: float = 1.0
    length: float = 4.7
    width: float = 2.0
    margin: float = 0.5
    lateral_seconds: float = 6.0
    tolerance: float = 1e-5
    solver_maxiter: int = 120


CFG = Settings()
CASES = ('head_on_inline_stop', 'head_on_pause_bay', 'head_on_reverse')


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def matrices(times, cfg=CFG):
    """Exact ZOH double-integrator response to horizon accelerations."""
    relative = np.asarray(times)[:, None] - np.arange(cfg.horizon)[None, :] * cfg.dt
    p = 0.5 * (np.maximum(relative, 0) ** 2 - np.maximum(relative-cfg.dt, 0) ** 2)
    v = np.clip(relative, 0, cfg.dt)
    return p, v

def integrate(state, acceleration, times):
    t = np.asarray(times)
    return np.column_stack((state[0] + t*state[1] + 0.5*t*t*acceleration,
                            state[1] + t*acceleration))

def lane_y(lane, times):
    start, end, stamp, duration = lane
    q = np.clip((np.asarray(times)-stamp) / duration, 0, 1)
    smooth = 10*q**3 - 15*q**4 + 6*q**5
    return start + (end-start)*smooth

def prediction(state, controls, lane, now, cfg=CFG):
    # The constant-velocity tail covers the next cycle's complete horizon.
    times = np.arange(cfg.horizon+2) * cfg.dt
    p, v = matrices(np.minimum(times, cfg.horizon*cfg.dt), cfg)
    x = state[0] + np.minimum(times, cfg.horizon*cfg.dt)*state[1] + p @ controls
    speed = state[1] + v @ controls
    x[-1] = x[-2] + cfg.dt*speed[-2]
    return {'stamp': now, 'times': (now+times).tolist(),
            'states': np.column_stack((x, speed, lane_y(lane, now+times))).tolist()}

def receive(packet, times):
    t, values = np.asarray(packet['times']), np.asarray(packet['states'])
    times = np.asarray(times)
    if (not np.all(np.isfinite(values)) or not np.all(np.isfinite(t)) or
            not np.all(np.diff(t) > 0) or abs(t[0]-packet['stamp']) > 1e-8 or
            times[0] < t[0]-1e-8 or times[-1] > t[-1]+1e-8):
        raise ValueError('Invalid or uncovered longitudinal prediction')
    return np.column_stack([np.interp(times, t, values[:, i]) for i in range(3)])

def body_gap(a, b, cfg=CFG):
    # Axis-aligned, padded rectangles: exact signed SAT gap for this abstraction.
    return np.maximum(np.abs(a[:, 0]-b[:, 0])-cfg.length,
                      np.abs(a[:, 2]-b[:, 2])-cfg.width)

def inside_road(states, road, cfg=CFG):
    x, y = states[:, 0], states[:, 2]
    inside = (x-cfg.length/2 >= road['xmin']-1e-8) & (x+cfg.length/2 <= road['xmax']+1e-8)
    main = (y-cfg.width/2 >= road['ymin']-1e-8) & (y+cfg.width/2 <= road['ymax']+1e-8)
    for bay in road.get('bays', []):
        main |= ((x-cfg.length/2 >= bay['xmin']-1e-8) &
                 (x+cfg.length/2 <= bay['xmax']+1e-8) &
                 (y-cfg.width/2 >= road['ymin']-1e-8) &
                 (y+cfg.width/2 <= bay['ymax']+1e-8))
    return inside & main


def build_longitudinal_qp(state, previous_acceleration, warm, now, target, front=None, rear=None, cfg=CFG, allow_reverse=False, road=None, terminal_stop=False):
    """Condensed convex QP: min .5*u.T*H*u+q.T*u, A*u >= b."""
    # ticks 当前时刻之后、覆盖整个预测时域的一组采样时间
    ticks = np.arange(1, cfg.horizon*cfg.substeps+1)*cfg.dt/cfg.substeps
    pmat, vmat = matrices(ticks, cfg)
    pbase = state[0]+ticks*state[1]
    vbase = np.full(len(ticks), state[1])
    rows, limits = [], []

    def lower(matrix, base, bound):
        rows.append(np.atleast_2d(matrix))
        limits.append(np.broadcast_to(bound, np.shape(base))-base)

    lower(vmat, vbase, -cfg.max_speed)
    lower(-vmat, -vbase, -cfg.max_speed)
    if terminal_stop:
        lower(vmat[-1], vbase[-1:], 0.)
        lower(-vmat[-1], -vbase[-1:], 0.)
    direction = 1 if target >= state[0] else -1
    # Legacy strategies constrain travel to their stage target direction.
    # The deadlock experiment permits both velocity signs with its forward goal.
    if not allow_reverse:
        lower(direction*vmat, direction*vbase, -1e-8)
    lower(-direction*pmat, -direction*pbase, -direction*target-1e-7)
    if road is not None:
        lower(pmat, pbase, road["xmin"]+cfg.length/2)
        lower(-pmat, -pbase, -road["xmax"]+cfg.length/2)
    distance = cfg.length+cfg.margin
    if front is not None:
        lower(-pmat, front[:, 0]-pbase, distance)
    if rear is not None:
        lower(pmat, pbase-rear[:, 0], distance)
    delta = np.eye(cfg.horizon)-np.eye(cfg.horizon, k=-1)
    delta_base = np.zeros(cfg.horizon)
    delta_base[0] = -previous_acceleration
    terms = ((.6, pmat, pbase-target), (.4, vmat, vbase),
             (.15, np.eye(cfg.horizon), np.zeros(cfg.horizon)), (.2, delta, delta_base))
    hess = sum(2*w*m.T@m for w, m, _ in terms)/len(ticks)
    linear = sum(2*w*m.T@b for w, m, b in terms)/len(ticks)
    matrix, bound = np.vstack(rows), np.concatenate(limits)
    return hess, linear, matrix, bound


def solve(state, previous_acceleration, warm, now, target, front=None, rear=None, cfg=CFG, allow_reverse=False, road=None, terminal_stop=False):
    """Solve one car's quadratic objective subject to linear constraints."""
    hess, linear, matrix, bound = build_longitudinal_qp(
        state, previous_acceleration, warm, now, target, front, rear, cfg,
        allow_reverse, road, terminal_stop)
    started = time.perf_counter()
    result = minimize(lambda u: .5*u@hess@u+linear@u, warm,
                      jac=lambda u: hess@u+linear, method='SLSQP',
                      bounds=[(-cfg.max_acceleration, cfg.max_acceleration)]*cfg.horizon,
                      constraints=[{'type': 'ineq', 'fun': lambda u: matrix@u-bound,
                                    'jac': lambda u: matrix}],
                      options={'ftol': 1e-7, 'maxiter': cfg.solver_maxiter})
    violation = float(max(0., np.max(bound-matrix@result.x),
                          np.max(np.abs(result.x))-cfg.max_acceleration))
    return result.x, {'stamp': now, 'accepted': bool(result.success and
                     np.all(np.isfinite(result.x)) and violation <= cfg.tolerance),
                     'solver_success': bool(result.success), 'message': str(result.message),
                     'iterations': int(result.nit), 'solve_seconds': time.perf_counter()-started,
                     'constraint_violation': violation, 'first_acceleration': float(result.x[0]),
                     'target': target}


def run_case(name, output, scene):
#     初始化场景
# 检查场景名称和时间参数，创建输出目录；
# 从场景配置加载道路、车辆初始状态和目标，初始化车辆横向位置、MPC 控制序列、预测包，以及当前仿真阶段。stopped、transition 和 record 是函数内部的小工具，分别用于判断停车、记录阶段切换和采集仿真数据。
    if name not in CASES:
        raise ValueError(f'Unknown head-on case: {name}')
    cfg = Settings(**scene['settings'])
    if cfg.dt <= 0 or cfg.horizon < 1 or cfg.substeps < 1:
        raise ValueError('Control period and horizon/substeps must be positive')
    output.mkdir(parents=True, exist_ok=False)
    road = scene['road']
    states = {car: np.array(scene['initial_states_x_v'][car], dtype=float) for car in ('A','B')}
    lanes = {car: [0., 0., cfg.epoch, cfg.lateral_seconds] for car in states}
    config = {**scene, 'case': name, 'origin': 'vehicle centre; A yaw=0, B yaw=pi',
              'model': 'exact longitudinal double integrator; prescribed lateral occupancy',
              'communication': {'transport': 'synchronous in-process prediction packets',
                                'nominal_packet_age_s': cfg.dt, 'bootstrap_packet_age_s': 0.,
                                'loss_rate': 0., 'jitter_s': 0., 'within_cycle_iterations': 0,
                                'fields': ['stamp', 'absolute times', 'x', 'vx', 'y'],
                                'tail': 'one cycle at zero acceleration'},
              'supervisor': 'centralized fixed A-first priority; case strategy selected externally',
              'initial_lane_occupancies': lanes,
              'cost': {'position': .6, 'speed': .4, 'acceleration': .15, 'acceleration_change': .2},
              'numerical_slack': {'target_position_m': 1e-7, 'velocity_direction_m_s': 1e-8}}
    write_json(output/'config.json', config)
    warm = {car: np.zeros(cfg.horizon) for car in states}
    previous_acc = dict.fromkeys(states, 0.)
    packets = {car: prediction(s, warm[car], lanes[car], cfg.epoch, cfg) for car,s in states.items()}
    phase = 'brake' if name == 'head_on_reverse' else 'approach'
    events, records, actual, publications = [], [], [], []
    min_gap, max_violation, failure = float('inf'), 0., None
    reverse_distance, arrivals = 0., {}

    def stopped(car, target):
        return (abs(states[car][0]-target) < scene['arrival_position_tolerance_m'] and
                abs(states[car][1]) < scene['arrival_speed_tolerance_m_s'])

    def transition(new, now, detail):
        nonlocal phase
        events.append({'time_s': round(now-cfg.epoch,10), 'from': phase, 'to': new, 'detail': detail})
        phase = new

    def record(now, motions, accelerations):
        nonlocal min_gap, max_violation, failure
        min_gap = min(min_gap, float(np.min(body_gap(motions['A'], motions['B'], cfg))))
        max_violation = max(max_violation, scene['sampled_gap_min_m']-min_gap)
        for car,motion in motions.items():
            if not np.all(inside_road(motion, road, cfg)):
                failure = f'{car}: sampled body outside road'
            max_violation = max(max_violation, float(np.max(np.abs(motion[:,1])))-cfg.max_speed,
                                abs(accelerations[car])-cfg.max_acceleration)
            for j,s in enumerate(motion):
                actual.append([round(now+j*cfg.dt/cfg.substeps,10), car, *s,
                               accelerations[car], phase, -s[1] if car=='B' else s[1]])
        if max_violation > cfg.tolerance:
            failure = 'sampled executed constraint violation'

    record(cfg.epoch, {car: np.array([[s[0],s[1],0.]]) for car,s in states.items()}, previous_acc)
    complete = False
#     逐周期闭环仿真
# 主循环从 head_on_core.py:191 开始，每轮对应一个控制周期：

# 根据车辆状态更新当前阶段，例如刹车、进入避让区、让对方通过、重新出发。
# 从对方上一个周期发来的预测包中，取得本周期控制时域内的邻车状态。
# 分别为两辆车调用 solve(...) 求控制序列，并检查优化结果是否可接受。只要有一个结果未通过检查，本周期就不执行控制，仿真记为失败。
# 用控制序列生成下一轮预测包；实际仿真则只执行当前周期的第一个加速度，并在子步内积分车辆运动。
# 记录位置、速度、车间距和约束违反情况，再更新车辆状态、上一周期加速度和 warm start 控制序列。
# 这就是 MPC 的滚动方式：每周期重新预测和求解，但只执行当前周期的控制，然后拿更新后的状态再算下一轮。
    for cycle in range(int(scene['cases'][name]['time_limit_s']/cfg.dt)):
        if failure:
            break
        now = cfg.epoch+cycle*cfg.dt
        bay = scene['reverse_bay_x'] if name=='head_on_reverse' else scene['pause_bay_x']
        if name != 'head_on_inline_stop':
            if phase=='brake' and stopped('B',scene['reverse_brake_x']):
                transition('retreat',now,'B near zero speed before reversing; no velocity reset')
            elif phase in ('approach','retreat') and stopped('B',bay):
                lanes['B'] = [0.,scene['bay_y'],now,cfg.lateral_seconds]
                transition('enter_bay',now,'Prescribed lateral occupancy starts after longitudinal stop')
            elif phase=='enter_bay' and now >= lanes['B'][2]+cfg.lateral_seconds-1e-8:
                transition('A_passes',now,'B occupies bay; release A stop line')
            elif phase=='A_passes' and states['A'][0] > bay+cfg.length+cfg.margin+.3:
                lanes['B'] = [scene['bay_y'],0.,now,cfg.lateral_seconds]
                transition('leave_bay',now,'A cleared bay; B returns to main lane')
            elif phase=='leave_bay' and now >= lanes['B'][2]+cfg.lateral_seconds-1e-8:
                transition('depart',now,'B resumes westbound travel')
        ticks = now+np.arange(1,cfg.horizon*cfg.substeps+1)*cfg.dt/cfg.substeps
        peers = {car: receive(packets[car],ticks) for car in states}
        controls, reports = {}, {}
        for car,state in states.items():
            if car=='A':
                target = scene['goals_x']['A'] if phase in ('A_passes','leave_bay','depart') else scene['A_hold_x']
            else:
                target = scene['reverse_brake_x'] if phase=='brake' else scene['goals_x']['B'] if phase=='depart' else bay
            other = 'B' if car=='A' else 'A'
            overlap = np.abs(lane_y(lanes[car],ticks)-peers[other][:,2]) < cfg.width+scene['sampled_gap_min_m']
            kwargs, used = {}, []
            if np.any(overlap):
                neighbor = peers[other].copy()
                is_front = states[other][0] >= state[0]
                neighbor[~overlap,0] = 1e4 if is_front else -1e4
                kwargs['front' if is_front else 'rear'] = neighbor
                used.append(other)
            u,report = solve(state,previous_acc[car],warm[car],now,target,cfg=cfg,**kwargs)
            report.update({'cycle':cycle,'car':car,'phase':phase,'controls':u.tolist(),
                           'peer_stamps':{peer:packets[peer]['stamp'] for peer in used}})
            controls[car],reports[car] = u,report
            records.append(report)
        if not all(r['accepted'] for r in reports.values()):
            failure = f'cycle {cycle}: rejected optimization; no controls executed'
            break
        outgoing = {car:prediction(states[car],controls[car],lanes[car],now,cfg) for car in states}
        publications.extend({'cycle':cycle,'car':car,**packet} for car,packet in outgoing.items())
        relative = np.arange(cfg.substeps+1)*cfg.dt/cfg.substeps
        motions = {car:np.column_stack((integrate(states[car],controls[car][0],relative),
                                        lane_y(lanes[car],now+relative))) for car in states}
        record(now,motions,{car:float(u[0]) for car,u in controls.items()})
        if name=='head_on_reverse':
            reverse_distance += float(np.sum(np.maximum(0,np.diff(motions['B'][:,0]))))
        for car in states:
            states[car] = motions[car][-1,:2]
            previous_acc[car] = float(controls[car][0])
            warm[car] = np.r_[controls[car][1:],controls[car][-1]]
        packets = outgoing
        for car,goal in scene['goals_x'].items():
            if stopped(car,goal) and car not in arrivals:
                arrivals[car] = round(now+cfg.dt-cfg.epoch,10)
        complete = len(arrivals)==2
        if failure or complete:
            break
#         整理并写出结果
# 循环结束后，根据是否失败、是否到达目标，确定 failed、completed 或 blocked 状态；汇总安全间距、求解耗时、到达时间等指标，并写出实际轨迹、求解记录、事件、预测包和摘要文件。
    status = 'failed' if failure else 'completed' if complete else 'blocked'
    elapsed = [r['solve_seconds'] for r in records]
    sums = {}
    for r in records:
        sums[r['cycle']] = sums.get(r['cycle'],0.)+r['solve_seconds']
    summary = {'case':name,'status':status,'expected_status':scene['cases'][name]['expected_status'],
               'expectation_met':status==scene['cases'][name]['expected_status'] and failure is None,
               'failure':failure,'final_phase':phase,'simulation_seconds':round(actual[-1][0]-cfg.epoch,10),
               'minimum_sampled_sat_gap_m':min_gap,'max_executed_sampled_violation':max(0.,max_violation),
               'max_prediction_violation':max((r['constraint_violation'] for r in records),default=0.),
               'final_states_x_v':{car:s.tolist() for car,s in states.items()},'arrival_times_s':arrivals,
               'B_reverse_distance_m':reverse_distance,'solve_count':len(records),
               'rejected_solves':sum(not r['accepted'] for r in records),
               'solve_seconds_mean':float(np.mean(elapsed)) if elapsed else None,
               'solve_seconds_p95':float(np.percentile(elapsed,95)) if elapsed else None,
               'cycles_over_control_period':sum(s>cfg.dt for s in sums.values()),
               'continuous_safety_certified':False,'steering_feasibility_tested':False,
               'autonomous_strategy_selection':False,'supervisor_is_centralized':True}
    with (output/'actual.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['absolute_time_s','car','x','vx','y','acceleration','phase','body_signed_speed'])
        writer.writerows(actual)
    write_json(output/'solves.json',records)
    write_json(output/'events.json',events)
    write_json(output/'summary.json',summary)
    with (output/'predictions.jsonl').open('w') as stream:
        for publication in publications:
            stream.write(json.dumps(publication,allow_nan=False)+'\n')
    print(f'{name}: {status}, t={summary["simulation_seconds"]:.1f}s',flush=True)
    return summary
