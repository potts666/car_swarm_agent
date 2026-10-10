#!/usr/bin/env python3
"""Fixed offline rear-axle bicycle experiment. No ROS vehicle control.
All validation is sampling-based; solver failure is never an impossibility proof.
"""
import argparse
import csv
import hashlib
import json
import sys
import time
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'frozen'))
import decentralized_mpc as m
from optimize_spatial_temporal import static_gaps, body_distance


def write(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False) + '\n')


def primitives(c):
    """Rest-to-rest controls with steering changes only at rest, no gear jumps."""
    radius = c['vehicle']['wheelbase_m'] / np.tan(.5)
    angle = np.arccos(1 - 3.4 / (2 * radius))
    def leg(distance, steering, sign=1, cycles=28):
        speed = distance / (m.DT * (cycles - 3))
        assert speed <= 1
        return [[0, steering / 2], [0, steering]] + [[sign * speed / (3*m.DT), steering]] * 3 + [[0, steering]] * (cycles - 6) + [[-sign * speed / (3*m.DT), steering]] * 3 + [[0, steering / 2], [0, 0]]
    a = leg(radius * angle, -.5, -1) + leg(radius * angle, .5, -1)
    straight = leg(24, 0, cycles=70)
    bypass = leg(radius * angle, .5) + leg(radius * angle, -.5)
    displacement = 2 * radius * np.sin(angle)
    bypass += leg(24 - 2 * displacement, 0, cycles=36)
    bypass += leg(radius * angle, -.5) + leg(radius * angle, .5)
    return np.array(a), np.array(straight), np.array(bypass)


def simulate(state, controls, substeps=40):
    # Integrate the same midpoint plant, denser solely for validation.
    return m.rollout(state, controls, substeps=substeps)


def errors(states, goals):
    return [dict(distance_m=float(np.linalg.norm(s[:2] - g[:2])),
                 yaw_error_rad=float(abs(np.arctan2(np.sin(s[2]-g[2]), np.cos(s[2]-g[2])))),
                 speed_m_s=float(s[3])) for s, g in zip(states, goals)]


def arrived(s, g, c):
    e = errors([s], [g])[0]
    return e['distance_m'] <= c['criteria']['position_m'] and e['yaw_error_rad'] <= c['criteria']['yaw_rad'] and abs(s[3]) <= c['criteria']['speed_m_s']


def validate(c, controls, goals, name, output):
    length = max(map(len, controls))
    padded = [np.vstack((u, np.zeros((length-len(u), 2)))) for u in controls]
    dense = [simulate(s, u) for s, u in zip(c['starts'], padded)]
    gaps = m.neighbor_gap(*dense)
    distances = body_distance(dense[0][:,:2], dense[0][:,2], dense[1][:,:2], dense[1][:,2])
    violations = {}
    finish = []
    reverse = []
    for i, (s, u) in enumerate(zip(dense, padded)):
        prefix = f'{i}_'
        violations[prefix+'static'] = max(0., .03-float(np.min(static_gaps(s[:,:2],s[:,2], np.array(c['obstacle_boxes'])))))
        violations[prefix+'speed'] = max(0., float(np.max(abs(s[:,3])))-1)
        violations[prefix+'acceleration'] = max(0.,float(np.max(abs(u[:,0])))-1)
        violations[prefix+'steering'] = max(0.,float(np.max(abs(u[:,1])))-.5)
        violations[prefix+'steering_rate'] = max(0.,float(np.max(abs(np.diff(np.r_[0,u[:,1]]))))/m.DT-1)
        lateral = s[1:,3]**2*np.tan(s[1:,4])/2.7
        violations[prefix+'total_acceleration'] = max(0.,float(np.max(np.hypot(np.repeat(u[:,0],40),lateral)))-2)
        violations[prefix+'goal'] = 0. if arrived(s[-1], goals[i], c) else 1.
        # Signed velocity must pass through zero before changing gear.
        signs = np.sign(s[:,3]); signs[np.abs(s[:,3])<1e-8] = 0
        violations[prefix+'gear_jump'] = float(np.any(signs[:-1]*signs[1:]<0))
        nz = signs[signs != 0]
        reverse.append(int(np.sum((nz[1:]<0)&(nz[:-1]>0))) + int(len(nz)>0 and nz[0]<0))
        hit = [j for j, x in enumerate(s) if arrived(x,goals[i],c)]
        finish.append(hit[0]*.01 if hit else None)
        with (output/f'{name}_{i}_trajectory.csv').open('w') as f:
            w=csv.writer(f); w.writerow(['t','x','y','yaw','speed','steering']); w.writerows(np.column_stack((np.arange(len(s))*.01,s)))
        write(output/f'{name}_{i}_controls.json',padded[i].tolist())
    violations['pair_gap'] = max(0.,.08-float(np.min(gaps)))
    result = dict(name=name, physical_envelope_pass=bool(max(violations.values())<=1e-5),
                  unchanged_baseline_envelope_pass=bool(max(violations.values())<=1e-5 and all(np.min(s[:,3])>=-1e-5 for s in dense)),
                  minimum_sampled_padded_body_distance_m=float(min(distances)),minimum_sampled_sat_gap_m=float(min(gaps)),
                  violations=violations, completion_time_s=finish, reverse_entries=reverse,
                  sample_dt_s=.01, continuous_time_certified=False, execution='open-loop controls through bicycle plant',
                  final_errors=errors([s[-1] for s in dense],goals))
    write(output/f'{name}_validation.json',result)
    return result,dense


def ref(states):
    r=np.zeros(len(states),dtype=[(k,float) for k in ('t','x','y','yaw','speed')])
    r['t']=np.arange(len(states))*.01
    for j,k in enumerate(('x','y','yaw','speed')): r[k]=states[:,j]
    return r


def closed_loop(c, refs, output, seed, policy='baseline'):
    np.random.seed(seed)
    states=np.array(c['starts'],float); goals=np.array(c['goals'])
    warm=[np.zeros((m.N,2)) for _ in range(2)]
    preds=[m.packet(m.EPOCH,m.rollout(s,u,tail=True)) for s,u in zip(states,warm)]
    records=[]; measures=[]; stop='observation_limit'; min_gap=float('inf'); solve_times=[]
    with (output/f'{policy}_{seed}.jsonl').open('w') as f:
        for cycle in range(round(c['criteria']['duration_s']/m.DT)):
            now=m.EPOCH+cycle*m.DT
            outputs=[]
            for i in range(2):
                permitted=True
                if policy=='rule_cycle': permitted=arrived(states[1-i],goals[1-i],c)
                if permitted:
                    u,r=m.solve(states[i],now,preds[1-i],refs[i],np.array(c['obstacle_boxes']),warm[i])
                else:
                    u=np.zeros((m.N,2)); r=dict(accepted=True,solver_success=None,solve_seconds=0,message='A waits for B through; B waits for A parked')
                solve_times.append(r['solve_seconds'])
                prediction=m.rollout(states[i],u,tail=True)
                row=dict(seed=seed,policy=policy,cycle=cycle,t=cycle*m.DT,car=i,permission=permitted,
                         state=states[i].tolist(),goal_errors=errors([states[i]],[goals[i]])[0],
                         peer_input={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in preds[1-i].items()},
                         reference_input=m.reference(refs[i],now+np.arange(1,m.N*m.SUBSTEPS+1)*.1).tolist(),
                         warm_input=warm[i].tolist(),controls=u.tolist(),prediction=prediction.tolist(),solver=r)
                outputs.append((u,r,prediction,row))
            if not all(r['accepted'] for _,r,_,_ in outputs):
                for _,_,_,row in outputs:
                    row['executed']=False; f.write(json.dumps(row)+'\n')
                stop='rejected_optimization_no_execution'; break
            dense=[simulate(s,u[:1]) for s,(u,_,_,_) in zip(states,outputs)]
            gap=float(np.min(m.neighbor_gap(*dense))); min_gap=min(min_gap,gap)
            safe=gap>=.08-1e-5 and all(np.min(static_gaps(s[:,:2],s[:,2],np.array(c['obstacle_boxes'])))>=.03-1e-5 for s in dense)
            for i,(u,r,p,row) in enumerate(outputs):
                nxt=m.rollout(states[i],u[:1])[-1]
                row.update(executed=True,actual_next_state=nxt.tolist(),displacement_m=float(np.linalg.norm(nxt[:2]-states[i,:2])),
                           direction=int(np.sign(nxt[3])) if abs(nxt[3])>1e-6 else 0,min_pair_sat_gap_m=gap,sampled_safe=safe)
                f.write(json.dumps(row)+'\n'); records.append(row)
                warm[i]=np.vstack((u[1:],u[-1])); states[i]=nxt
                preds[i]=m.packet(now,p)
            measures.append(states.copy())
            if not safe: stop='sampled_safety_violation'; break
            if all(arrived(s,g,c) for s,g in zip(states,goals)): stop='both_arrived'; break
    window=round(c['criteria']['stagnation_window_s']/m.DT)
    stalled=False
    for j in range(window,len(measures)+1):
        segment=np.array(measures[j-window:j]); extent=np.max(np.linalg.norm(segment[:,:,:2]-segment[0,:,:2],axis=2),axis=0)
        progress=np.array([np.linalg.norm(segment[0,i,:2]-goals[i,:2])-np.linalg.norm(segment[-1,i,:2]-goals[i,:2]) for i in range(2)])
        if np.all(extent<c['criteria']['max_extent_m']) and np.all(progress<c['criteria']['min_progress_m']) and all(not arrived(s,g,c) for s,g in zip(segment[-1],goals)):
            stalled=True; break
    result=dict(policy=policy,seed=seed,stop=stop,cycles=len(measures),success=stop=='both_arrived',
                classification='rule_cycle' if stalled and policy=='rule_cycle' else 'experimental_sustained_stagnation' if stalled else 'not_reproduced',
                minimum_sampled_sat_gap_m=min_gap if np.isfinite(min_gap) else None,
                solve_seconds_mean=float(np.mean(solve_times)),solve_seconds_max=max(solve_times),
                final_errors=errors(states,goals),permission_rule='both permitted' if policy=='baseline' else 'A waits B goal; B waits A goal')
    write(output/f'{policy}_{seed}_summary.json',result)
    return result,records


def plot(c,dense,output,records):
    fig,ax=plt.subplots(figsize=(13,5))
    for box in c['obstacle_boxes']:
        x,y,w,h=box; ax.add_patch(Polygon([[x,y],[x+w,y],[x+w,y+h],[x,y+h]],color='gray'))
    ax.add_patch(Polygon(c['parking_polygon'],fill=False,edgecolor='green',linewidth=2))
    for i,(s,g,path) in enumerate(zip(c['starts'],c['goals'],dense)):
        color=['tab:blue','tab:orange'][i]; ax.plot(path[:,0],path[:,1],color=color,label=['A reverse','B straight'][i])
        for label,p in [('start',s),('goal',g)]: ax.text(p[0],p[1],f'{"AB"[i]} {label}')
    # Full padded swept body, not rear-axle path, overlapping B's lane.
    for s in dense[0][::40]:
        ax.add_patch(Polygon(corners(s),color='red',alpha=.06))
    ax.set(xlim=(-15,20),ylim=(-7,7),xlabel='x / m',ylabel='y / m',title='Fixed scene; red: sampled full A body sweep / B lane conflict')
    ax.set_aspect('equal'); ax.legend(); fig.tight_layout(); fig.savefig(output/'scene.png',dpi=160); plt.close(fig)
    fig,axs=plt.subplots(3,1,figsize=(9,7),sharex=True)
    for i in range(2):
        r=[x for x in records if x['car']==i]; t=[x['t'] for x in r]
        axs[0].plot(t,[x['goal_errors']['distance_m'] for x in r],label='AB'[i])
        axs[1].plot(t,[x['displacement_m'] for x in r]); axs[2].plot(t,[x['actual_next_state'][3] for x in r])
    axs[0].set_ylabel('goal distance / m'); axs[0].legend(); axs[1].set_ylabel('step displacement / m'); axs[2].set_ylabel('speed / m/s'); axs[2].set_xlabel('simulation seconds'); fig.tight_layout(); fig.savefig(output/'baseline_series.png'); plt.close(fig)


def corners(s):
    local=np.array([[-1.1,-1], [3.6,-1],[3.6,1],[-1.1,1]])
    co,si=np.cos(s[2]),np.sin(s[2]); return local@np.array([[co,si],[-si,co]])+s[:2]


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--output',type=Path,default=ROOT/'results'); args=parser.parse_args()
    c=json.loads((ROOT/'scenario.json').read_text()); out=args.output; out.mkdir(parents=True,exist_ok=True)
    write(out/'scenario.json',c)
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in ROOT.rglob('*.py')}
    write(out/'execution_manifest.json',dict(source_hashes=hashes,scenario_sha256=hashlib.sha256((ROOT/'scenario.json').read_bytes()).hexdigest(),numpy=np.__version__,python=sys.version,baseline_source='frozen/decentralized_mpc.py',parameters=c))
    a,b,bypass=primitives(c)
    wait=np.vstack((np.zeros((len(a),2)),b))
    zero=np.zeros((1,2))
    validations=[]
    for name,controls in [('A_first_B_wait',[a,wait]),('B_bypass_A_parallel',[a,bypass]),('uncoordinated_conflict',[a,b])]:
        result,dense=validate(c,controls,c['goals'],name,out); validations.append(result)
        if name=='uncoordinated_conflict': conflict_dense=dense
    # Reference timing is fixed before baseline; independent from outcomes.
    refs=[ref(simulate(c['starts'][0],a)),ref(simulate(c['starts'][1],b))]
    summaries=[]; first=[]
    for seed in c['seeds']:
        result,records=closed_loop(c,refs,out,seed); summaries.append(result)
        if not first: first=records
    result,_=closed_loop(c,refs,out,c['seeds'][0],'rule_cycle'); summaries.append(result)
    plot(c,conflict_dense,out,first)
    write(out/'comparison.json',dict(counterfactuals=validations,closed_loop=summaries,
          conclusion='Physical signed-speed witness and forward-only baseline use different admissible input sets. Strict evidence A under unchanged baseline constraints is NOT established. No infinite deadlock or scene impossibility proof.'))
    print(json.dumps(dict(counterfactuals=validations,closed_loop=summaries),indent=2))

if __name__=='__main__': main()
