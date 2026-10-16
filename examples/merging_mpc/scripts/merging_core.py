"""Three-car longitudinal merging MPC and closed-loop logging."""
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
    headway: float = 0.8
    lateral_seconds: float = 6.0
    tolerance: float = 1e-5


CFG = Settings()
CASES = ('merge_no_cooperation', 'merge_cooperative')
ORDER = ('A', 'C', 'B')


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


def solve(state, previous_acceleration, warm, now, cruise, front=None,
          following=False, extra_gap=1.8, cfg=CFG):
    """Local acceleration QP; the front trajectory remains fixed during a solve."""
    ticks = np.arange(1,cfg.horizon*cfg.substeps+1)*cfg.dt/cfg.substeps
    pmat,vmat = matrices(ticks,cfg)
    pbase,vbase = state[0]+ticks*state[1],np.full(len(ticks),state[1])
    rows,limits = [],[]

    def lower(matrix,base,bound):
        rows.append(np.atleast_2d(matrix))
        limits.append(np.broadcast_to(bound,np.shape(base))-base)

    lower(vmat,vbase,0.)
    lower(-vmat,-vbase,-cfg.max_speed)
    terms = [(2.,vmat,vbase-cruise)]
    if front is not None:
        headway = cfg.headway if following else 0.
        lower(-pmat-headway*vmat,front[:,0]-pbase-headway*vbase,cfg.length+cfg.margin)
        if following:
            terms.append((1.2,-pmat-cfg.headway*vmat,
                          front[:,0]-pbase-cfg.headway*vbase-cfg.length-cfg.margin-extra_gap))
    delta = np.eye(cfg.horizon)-np.eye(cfg.horizon,k=-1)
    delta_base = np.zeros(cfg.horizon)
    delta_base[0] = -previous_acceleration
    terms.extend(((.15,np.eye(cfg.horizon),np.zeros(cfg.horizon)),(.2,delta,delta_base)))
    hess = sum(2*w*m.T@m for w,m,_ in terms)/len(ticks)
    linear = sum(2*w*m.T@b for w,m,b in terms)/len(ticks)
    matrix,bound = np.vstack(rows),np.concatenate(limits)
    started = time.perf_counter()
    result = minimize(lambda u:.5*u@hess@u+linear@u,warm,jac=lambda u:hess@u+linear,
                      method='SLSQP',bounds=[(-cfg.max_acceleration,cfg.max_acceleration)]*cfg.horizon,
                      constraints=[{'type':'ineq','fun':lambda u:matrix@u-bound,'jac':lambda u:matrix}],
                      options={'ftol':1e-7,'maxiter':120})
    violation = float(max(0.,np.max(bound-matrix@result.x),np.max(np.abs(result.x))-cfg.max_acceleration))
    return result.x,{'stamp':now,'accepted':bool(result.success and np.all(np.isfinite(result.x)) and
                    violation<=cfg.tolerance),'solver_success':bool(result.success),
                    'message':str(result.message),'iterations':int(result.nit),
                    'solve_seconds':time.perf_counter()-started,'constraint_violation':violation,
                    'first_acceleration':float(result.x[0]),'following_cost_active':following}


def admission(packets,now,buffer=1.,cfg=CFG):
    times = now+np.linspace(0,cfg.lateral_seconds,31)
    states = {car:receive(packets[car],times) for car in ORDER}
    residuals = {}
    for ahead,behind in zip(ORDER[:-1],ORDER[1:]):
        required = cfg.length+cfg.margin+cfg.headway*np.maximum(0.,states[behind][:,1])+buffer
        residuals[f'{ahead}-{behind}'] = float(np.min(states[ahead][:,0]-states[behind][:,0]-required))
    return min(residuals.values())>=-cfg.tolerance,residuals


def run_case(name,output,scene):
    if name not in CASES:
        raise ValueError(f'Unknown merging case: {name}')
    cfg = Settings(**scene['settings'])
    if (cfg.dt<=0 or cfg.horizon<1 or cfg.substeps<1 or cfg.lateral_seconds<=0 or
            cfg.lateral_seconds>cfg.horizon*cfg.dt or not 0<=scene['cruise_speed']<=cfg.max_speed):
        raise ValueError('Invalid control/horizon/lateral-duration/cruise settings')
    output.mkdir(parents=True,exist_ok=False)
    states = {car:np.array(scene['initial_states_x_v'][car],dtype=float) for car in ('A','B','C')}
    lanes = {car:[scene['merge_lane_y'] if car=='C' else scene['main_lane_y']]*2+
             [cfg.epoch,cfg.lateral_seconds] for car in states}
    cooperative = scene['cases'][name]['cooperative']
    config = {**scene,'case':name,'desired_order':list(ORDER),'origin':'vehicle centre; all cars face east',
              'model':'exact longitudinal double integrator; prescribed lateral occupancy',
              'communication':{'transport':'synchronous in-process full-mesh prediction packets',
                               'nominal_packet_age_s':cfg.dt,'bootstrap_packet_age_s':0.,
                               'loss_rate':0.,'jitter_s':0.,'within_cycle_iterations':0,
                               'fields':['stamp','absolute times','x','vx','y'],
                               'tail':'one cycle at zero acceleration'},
              'supervisor':'centralized admission; fixed A-C-B order; no decentralized negotiation',
              'cost':{'cruise_speed':2.,'following_gap':1.2,'acceleration':.15,'acceleration_change':.2}}
    write_json(output/'config.json',config)
    warm = {car:np.zeros(cfg.horizon) for car in states}
    previous_acc = dict.fromkeys(states,0.)
    packets = {car:prediction(state,warm[car],lanes[car],cfg.epoch,cfg) for car,state in states.items()}
    phase = 'following'
    events,records,actual,publications,guards = [],[],[],[],[]
    min_gap,max_violation,failure = float('inf'),0.,None
    complete = False

    def transition(new,now,detail):
        nonlocal phase
        events.append({'time_s':round(now-cfg.epoch,10),'from':phase,'to':new,'detail':detail})
        phase = new

    def record(now,motions,accelerations):
        nonlocal min_gap,max_violation,failure
        road = scene['road']
        for i,car in enumerate(states):
            motion = motions[car]
            x,y = motion[:,0],motion[:,2]
            inside = ((x-cfg.length/2>=road['xmin']-1e-8)&(x+cfg.length/2<=road['xmax']+1e-8)&
                      (y-cfg.width/2>=road['ymin']-1e-8)&(y+cfg.width/2<=road['ymax']+1e-8))
            if not np.all(inside):
                failure = f'{car}: sampled body outside road'
            max_violation = max(max_violation,-float(np.min(motion[:,1])),
                                float(np.max(motion[:,1]))-cfg.max_speed,
                                abs(accelerations[car])-cfg.max_acceleration)
            for other in list(states)[i+1:]:
                min_gap = min(min_gap,float(np.min(body_gap(motion,motions[other],cfg))))
                max_violation = max(max_violation,scene['sampled_gap_min_m']-min_gap)
            for j,s in enumerate(motion):
                actual.append([round(now+j*cfg.dt/cfg.substeps,10),car,*s,accelerations[car],phase])
        if max_violation>cfg.tolerance:
            failure = 'sampled executed constraint violation'

    record(cfg.epoch,{car:np.array([[s[0],s[1],lanes[car][0]]]) for car,s in states.items()},previous_acc)
    for cycle in range(int(scene['time_limit_s']/cfg.dt)):
        if failure:
            break
        now = cfg.epoch+cycle*cfg.dt
        if phase=='following' and now-cfg.epoch>=scene['request_time_s']-1e-8:
            transition('open_gap',now,'C requests insertion; activate virtual-front following only if cooperative')
        if phase=='open_gap':
            accepted,residuals = admission(packets,now,scene['admission_buffer_m'],cfg)
            guards.append({'time_s':round(now-cfg.epoch,10),'admitted':bool(accepted),
                           'minimum_predicted_residual_m':min(residuals.values()),'pair_residuals_m':residuals})
            if accepted:
                lanes['C'] = [scene['merge_lane_y'],scene['main_lane_y'],now,cfg.lateral_seconds]
                transition('merging',now,'All A-C-B pair predictions meet spacing over the entire lateral transfer')
        elif phase=='merging' and now>=lanes['C'][2]+cfg.lateral_seconds-1e-8:
            transition('merged',now,'C occupancy reached main lane; continue observing car following')
        ticks = now+np.arange(1,cfg.horizon*cfg.substeps+1)*cfg.dt/cfg.substeps
        peers = {car:receive(packets[car],ticks) for car in states}
        controls,reports = {},{}
        for car,state in states.items():
            following = cooperative and phase!='following' and car!='A'
            front_car = None if car=='A' else 'C' if car=='B' and following else 'A'
            front = peers[front_car] if front_car else None
            u,report = solve(state,previous_acc[car],warm[car],now,scene['cruise_speed'],front,
                             following,scene['following_extra_gap_m'],cfg)
            report.update({'cycle':cycle,'car':car,'phase':phase,'front_car':front_car,
                           'controls':u.tolist(),'peer_stamps':{front_car:packets[front_car]['stamp']} if front_car else {}})
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
        for car in states:
            states[car] = motions[car][-1,:2]
            previous_acc[car] = float(controls[car][0])
            warm[car] = np.r_[controls[car][1:],controls[car][-1]]
        packets = outgoing
        settled = all(abs(s[1]-scene['cruise_speed'])<scene['settled_speed_tolerance_m_s'] for s in states.values())
        correct_order = all(states[a][0]>states[b][0] for a,b in zip(ORDER[:-1],ORDER[1:]))
        complete = phase=='merged' and now+cfg.dt-cfg.epoch>=scene['observation_time_s'] and settled and correct_order
        if complete or failure:
            break
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
               'final_states_x_v':{car:s.tolist() for car,s in states.items()},
               'admitted_at_s':next((e['time_s'] for e in events if e['to']=='merging'),None),
               'merge_completed_at_s':next((e['time_s'] for e in events if e['to']=='merged'),None),
               'B_minimum_speed_m_s':min(row[3] for row in actual if row[1]=='B'),
               'solve_count':len(records),'rejected_solves':sum(not r['accepted'] for r in records),
               'solve_seconds_mean':float(np.mean(elapsed)) if elapsed else None,
               'solve_seconds_p95':float(np.percentile(elapsed,95)) if elapsed else None,
               'cycles_over_control_period':sum(s>cfg.dt for s in sums.values()),
               'continuous_safety_certified':False,'steering_feasibility_tested':False,
               'autonomous_order_selection':False,'supervisor_is_centralized':True}
    with (output/'actual.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['absolute_time_s','car','x','vx','y','acceleration','phase'])
        writer.writerows(actual)
    for filename,value in (('solves.json',records),('events.json',events),('admission_checks.json',guards),('summary.json',summary)):
        write_json(output/filename,value)
    with (output/'predictions.jsonl').open('w') as stream:
        for packet in publications:
            stream.write(json.dumps(packet,allow_nan=False)+'\n')
    print(f'{name}: {status}, admitted={summary["admitted_at_s"]}, merged={summary["merge_completed_at_s"]}',flush=True)
    return summary
