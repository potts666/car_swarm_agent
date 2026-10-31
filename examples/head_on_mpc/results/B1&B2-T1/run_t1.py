"""Frozen-instance T1; run with OPENBLAS_NUM_THREADS=1 and OSQP 1.1.3 available."""
import copy
import csv
import hashlib
import json
import platform
from pathlib import Path
import sys
import time
from unittest.mock import patch

import numpy as np
import scipy
from scipy.sparse import csc_matrix, eye, vstack
import osqp

ROOT = Path(__file__).resolve().parent
ORIGINAL = ROOT.parent/'bicycle_e3_002'
sys.path.insert(0, str(ROOT/'source'))
import linearized_joint as bj
import longitudinal_joint as lj
from bicycle_core import CARS, rollout, step, audit, corners, clipped_area


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def lines(path):
    return [json.loads(s) for s in path.read_text().splitlines()]


SCENE = json.loads((ORIGINAL/'scene.json').read_text())
PATHS = json.loads((ORIGINAL/'paths.json').read_text())
OLD_JAC = bj.rk4_jacobians


def composed(z, u, h, vehicle):
    count = round(h/SCENE['sample_s'])
    assert abs(count*SCENE['sample_s']-h) < 1e-12
    A, B = np.eye(5), np.zeros((5, 2))
    for _ in range(count):
        F, G = OLD_JAC(z, u, SCENE['sample_s'], vehicle)
        A, B = F@A, F@B+G
        z = step(z, u, SCENE['sample_s'], vehicle)
    return A, B


def snapshot(branch, cycle):
    directory = ORIGINAL/branch
    solves = lines(directory/'solves.jsonl')
    constraints = [r for r in lines(directory/'constraints.jsonl') if r['cycle'] == cycle]
    forecasts = {round(r['time_s'], 8): r for r in lines(directory/'predictions.jsonl')}
    now = SCENE['epoch_s']+cycle*SCENE['h']
    actual = {c: [] for c in CARS}
    for r in csv.DictReader((directory/'actual.csv').open()):
        if abs(float(r['time_s'])-now) < 1e-8:
            actual[r['car']].append([float(r[k]) for k in ('x','y','psi','v','delta')])
    controller = bj.BicycleController(SCENE, PATHS, 3)
    permissions = {}
    for r in constraints:
        c = r['car']; a = controller.agents[c]
        assert actual[c], (branch, cycle, c)
        a.z = np.array(actual[c][-1]); a.index = r['segment']; a.mode = r['phase']
        a.s = r['s0']; a.ready = c == 'B' and a.index >= 3
        a.stop_steps = r['stop_index'] if r['stop_at_end'] and a.mode == 'MOVE' else None
        prior = forecasts[round(now-SCENE['h'], 8)]
        a.previous = a.segment['direction']*prior['commands'][c][0][0] if a.mode == 'MOVE' else 0.
        # A new segment resets previous acceleration; the logged phase identifies it.
        previous_row = next(x for x in lines(directory/'constraints.jsonl') if x['cycle']==cycle-1 and x['car']==c)
        if previous_row['segment'] != a.index or previous_row['phase'] != a.mode:
            a.previous = 0.
        if a.mode != 'MOVE':
            from bicycle_core import steer
            target = 0. if a.mode in ('WAIT','DONE') else a.segment['delta']
            a.steering = steer(a.z[4], target, SCENE, 'MUTEX_WAIT' if a.mode=='WAIT' else a.segment['stage'])
        permissions[c] = bool(r['g_i'])
        np.testing.assert_allclose(a.z, prior['predicted_states'][c][0], atol=2e-12, rtol=0)
    built = lj.build_qp(controller.agents, permissions, SCENE)
    logged = solves[cycle]['parameters']
    assert built[-1] == logged, (branch, cycle, built[-1], logged)
    frozen = dict(branch=branch,cycle=cycle,state_timestamp_s=now,permissions=permissions,
        agents={c:dict(z=a.z.tolist(),index=a.index,mode=a.mode,s=a.s,previous=a.previous,
                      ready=a.ready,stop_steps=a.stop_steps,steering=a.steering) for c,a in controller.agents.items()},
        parameters=logged,original_record=solves[cycle],reconstruction='actual.csv at failed timestamp, cross-checked against preceding accepted prediction; phase/parameters from constraints; prior applied command from predictions')
    name = f'{branch}_{cycle}'
    write(ROOT/'frozen'/f'{name}.json', frozen)
    H,q,A,lo,hi,E,b,bounds,_ = built
    np.savez_compressed(ROOT/'frozen'/f'{name}_reference.npz',H=H,q=q,A=A,lo=lo,hi=hi,E=E,b=b,bounds=bounds)
    return controller, permissions, frozen


def errors(predicted, dense):
    return {c:dict(position_m=float(np.max(np.linalg.norm(predicted[c][:,:2]-dense[c][::10,:2],axis=1))),
        heading_rad=float(np.max(abs(predicted[c][:,2]-dense[c][::10,2]))),
        speed_m_s=float(np.max(abs(predicted[c][:,3]-dense[c][::10,3]))),
        steering_rad=float(np.max(abs(predicted[c][:,4]-dense[c][::10,4])))) for c in CARS}


def derivative_study(controller, permissions, name):
    acceleration, ref = lj.solve(controller.agents, permissions, SCENE, bj.BicycleController.reference_feasibility_band)
    assert ref['accepted'], ref
    with patch.object(lj, 'solve', return_value=(acceleration, ref)):
        commands, dense, _ = lj.Controller.predict(controller, permissions)
    target = {c:dense[c][::10] for c in CARS}
    inputs = {c:np.array(commands[c])[:,:2].astype(float) for c in CARS}
    np.savez_compressed(ROOT/'frozen'/f'{name}_target.npz',**{f'{c}_{k}':v for c in CARS for k,v in [('states',target[c]),('inputs',inputs[c])]})
    comparison = []; fd = []
    for label, derivative in [('single',OLD_JAC),('composed',composed)]:
        worst = 0.
        for c in CARS:
            for z,u in zip(target[c][:-1], inputs[c]):
                A,B = derivative(z,u,SCENE['h'],SCENE['vehicle'])
                eps = 1e-5
                def endpoint(zz, uu):
                    for _ in range(10): zz = step(zz,uu,SCENE['sample_s'],SCENE['vehicle'])
                    return zz
                numeric = np.column_stack([(endpoint(z+eps*e,u)-endpoint(z-eps*e,u))/(2*eps) for e in np.eye(5)])
                numeric_u = np.column_stack([(endpoint(z,u+eps*e)-endpoint(z,u-eps*e))/(2*eps) for e in np.eye(2)])
                worst = max(worst,float(np.max(abs(A-numeric))),float(np.max(abs(B-numeric_u))))
        fd.append(dict(derivative=label,max_abs_fd_error=worst,epsilon=1e-5))
        if label == 'composed': assert worst < 2e-8
        # Same bounded deterministic perturbation for both models; no optimizer confound.
        perturbed = {c:inputs[c]+np.tile([-.001,.001],(SCENE['N'],1)) for c in CARS}
        # FD/perturbation is a mathematical probe, independent of actuator acceptance.
        replay = {}
        for c in CARS:
            zz = [controller.agents[c].z.copy()]
            for u in perturbed[c]:
                for _ in range(10): zz.append(step(zz[-1],u,SCENE['sample_s'],SCENE['vehicle']))
            replay[c] = np.array(zz)
        with patch.object(bj,'rk4_jacobians',derivative):
            predicted = {}
            for c in CARS:
                base,M,_ = bj.condensed(controller.agents[c].z,target[c],inputs[c],SCENE)
                predicted[c] = np.vstack([controller.agents[c].z,base+np.einsum('kij,j->ki',M,perturbed[c].ravel())])
        comparison.append(dict(derivative=label,errors=errors(predicted,replay)))
    runs=[]; fixed_candidate=None
    for label, derivative in [('single',OLD_JAC),('composed',composed)]:
        for updates in (1,3):
            agent=copy.deepcopy(controller);agent.updates=updates
            stages=[]
            original_subproblem=bj.subproblem
            def timed_subproblem(*args):
                started=time.perf_counter();answer=original_subproblem(*args)
                stages.append(dict(build_solve_check_s=time.perf_counter()-started,solver_s=answer[2]['solver_wall_s']))
                return answer
            started=time.perf_counter()
            original_setup=osqp.OSQP.setup
            captures=[]
            def capture_setup(solver, **kwargs):
                captures.append(kwargs)
                return original_setup(solver,**kwargs)
            with patch.object(osqp.OSQP,'setup',capture_setup), patch.object(bj,'rk4_jacobians',derivative), patch.object(lj,'solve',return_value=(acceleration,copy.deepcopy(ref))), patch.object(bj,'subproblem',timed_subproblem):
                _,_,record=agent.predict(permissions)
            elapsed=time.perf_counter()-started
            files=[]
            for index,kwargs in enumerate(captures):
                path=ROOT/'frozen'/f'{name}_{label}_{updates}_qp{index}.npz'
                np.savez_compressed(path,**{k:(v.toarray() if hasattr(v,'toarray') else v) for k,v in kwargs.items()})
                files.append(str(path.relative_to(ROOT)))
            if label=='single' and updates==1:
                if name=='B2_single_56':
                    original=json.loads((ROOT/'frozen'/f'{name}.json').read_text())['original_record']
                    np.testing.assert_allclose(record['solution'],original['solution'],atol=1e-10,rtol=0)
                fixed_candidate=np.array(record['solution']).reshape(2,SCENE['N'],2)
            runs.append(dict(derivative=label,update_limit=updates,frozen_reference=True,
                offline_bicycle_wall_s=elapsed,reference_build_solve_check_s=ref['solve_wall_s'],
                state_timestamp_s=SCENE['epoch_s']+int(name.rsplit('_',1)[1])*SCENE['h'],
                start_monotonic_s=started,end_monotonic_s=started+elapsed,budget_remaining_s=bj.DEADLINE_S-elapsed,
                committed=False,rejection_layer=None if record['accepted'] else 'bicycle_replay_or_solver',
                qp_files=files,stages=stages,record=record))
    candidate_probe=[];candidate_predictions={}
    for label,derivative in [('single',OLD_JAC),('composed',composed)]:
        predicted={};replay={}
        with patch.object(bj,'rk4_jacobians',derivative):
            for i,c in enumerate(CARS):
                base,M,_=bj.condensed(controller.agents[c].z,target[c],inputs[c],SCENE)
                predicted[c]=np.vstack([controller.agents[c].z,base+np.einsum('kij,j->ki',M,fixed_candidate[i].ravel())])
                replay[c]=rollout(controller.agents[c].z,[(a,w,'FIXED') for a,w in fixed_candidate[i]],SCENE)
        candidate_predictions[label]=predicted
        candidate_probe.append(dict(derivative=label,errors=errors(predicted,replay)))
    derivative_effect={c:float(np.max(np.linalg.norm(candidate_predictions['single'][c][:,:2]-candidate_predictions['composed'][c][:,:2],axis=1))) for c in CARS}
    amplitude_probe=[]
    with patch.object(bj,'rk4_jacobians',composed):
        for amplitude in (.25,.5,1.):
            predicted={};replay={}
            for i,c in enumerate(CARS):
                candidate=inputs[c]+amplitude*(fixed_candidate[i]-inputs[c])
                base,M,_=bj.condensed(controller.agents[c].z,target[c],inputs[c],SCENE)
                predicted[c]=np.vstack([controller.agents[c].z,base+np.einsum('kij,j->ki',M,candidate.ravel())])
                replay[c]=rollout(controller.agents[c].z,[(a,w,'AMPLITUDE') for a,w in candidate],SCENE)
            amplitude_probe.append(dict(amplitude=amplitude,errors=errors(predicted,replay)))
    return dict(instance=name,finite_difference=fd,same_input_perturbation=comparison,
                same_optimized_candidate=candidate_probe,derivative_effect_position_m=derivative_effect,
                composed_candidate_amplitude_probe=amplitude_probe,runs=runs)


def row_keys(parameters, agents):
    keys=[];equal=[];N=SCENE['N']
    for c in CARS:
        p=parameters[c];K=p['stop_index']
        if not p['cap']:continue
        count=K-1 if p['stop_at_end'] else K
        keys += [(c,'position',k) for k in range(count)]
        keys += [(c,'speed',k) for k in range(max(0,K-1))]
        if not(p['stop_at_end'] and K==1):
            equal.append((c,'terminal_speed',0))
            if p['stop_at_end']:equal.append((c,'terminal_position',0))
    b=agents['B']
    if not b.ready or (b.index>=3 and b.mode!='WAIT'):keys += [('pair','separation',k) for k in range(N)]
    return keys+equal+[(c,'input',k) for c in CARS for k in range(N)]


def reference_problem(controller, permissions):
    started=time.perf_counter()
    H,q,A,lo,hi,E,b,bounds,p=lj.build_qp(controller.agents,permissions,SCENE)
    matrix=vstack([csc_matrix(A),csc_matrix(E),eye(len(q))],format='csc')
    band=controller.reference_feasibility_band
    lower=np.r_[lo-band,b-band,np.array(bounds)[:,0]];upper=np.r_[hi+band,b+band,np.array(bounds)[:,1]]
    free=np.flatnonzero(np.array(bounds)[:,0]!=np.array(bounds)[:,1]);fixed=np.setdiff1d(np.arange(len(q)),free)
    x=np.zeros(len(q));x[fixed]=np.array(bounds)[fixed,0]
    offset=np.asarray(matrix@x).ravel();reduced=matrix[:,free]
    norms=np.asarray(abs(reduced).sum(axis=1)).ravel();keep=norms>1e-12
    keys=row_keys(p,controller.agents);assert len(keys)==matrix.shape[0]
    return dict(H=H,q=q,matrix=matrix,lower=lower,upper=upper,free=free,x=x,offset=offset,
        reduced=reduced[keep],row_scale=np.maximum(1.,norms[keep]),keys=[keys[i] for i in np.flatnonzero(keep)],
        objective_scale=max(1.,np.max(abs(q)),np.max(abs(H))),parameters=p,all_keys=keys,hessian_min=float(np.linalg.eigvalsh(H)[0]),build_s=time.perf_counter()-started)


def solve_reference(p, mode, previous=None):
    start=time.perf_counter();free=p['free'];x=p['x'].copy();scale=p['objective_scale']
    rows=p['row_scale'] if mode!='no_row_scaling' else np.ones(len(p['keys']))
    # Same feasible set and objective, only positive row normalization changes.
    solver=osqp.OSQP();solver.setup(P=csc_matrix(p['H'][np.ix_(free,free)]/scale),q=(p['q']+p['H']@x)[free]/scale,
        A=csc_matrix(p['reduced'].multiply(1/rows[:,None])),l=(p['lower']-p['offset'])[np.asarray(abs(p['matrix'][:,free]).sum(axis=1)).ravel()>1e-12]/rows,
        u=(p['upper']-p['offset'])[np.asarray(abs(p['matrix'][:,free]).sum(axis=1)).ravel()>1e-12]/rows,
        eps_abs=1e-8,eps_rel=1e-8,max_iter=20000,polishing=True,verbose=False,adaptive_rho_interval=50)
    setup_s=time.perf_counter()-start
    if mode in ('shift_primal','shift_primal_dual'):
        shifted=np.concatenate([np.r_[previous['x'][i*32+1:(i+1)*32],0.] for i in range(2)])
        if mode=='shift_primal':solver.warm_start(x=shifted[free])
        else:
            raw={tuple(k):v*previous['scale']/s for k,v,s in zip(previous['keys'],previous['y'],previous['rows'])}
            mapped=[]
            for c,kind,k in p['keys']:
                old=(c,kind,k if kind.startswith('terminal') else k+1)
                mapped.append(raw.get(old,0.))
            solver.warm_start(x=shifted[free],y=np.array(mapped)*rows/scale)
    started=time.perf_counter();result=solver.solve(raise_error=False);solve_s=time.perf_counter()-started
    x[free]=result.x;residual=np.asarray(p['matrix']@x).ravel()
    violation=max(0.,float(np.max(p['lower']-residual)),float(np.max(residual-p['upper'])))
    # Check the original terminal equations separately, without the numerical band.
    terminal=0.
    for i,c in enumerate(CARS):
        a=p['parameters'][c]
        if a['cap'] and a['stop_at_end']:
            K=a['stop_index'];P,V=lj.response([K*SCENE['h']],SCENE['h'],32);u=x[i*32:(i+1)*32]
            terminal=max(terminal,abs(float(a['s0']+K*SCENE['h']*a['v0']+(P@u)[0]-a['upper'])),abs(float(a['v0']+(V@u)[0])))
    by_kind={}
    for key,value,low,high in zip(p['all_keys'],residual,p['lower'],p['upper']):
        kind=key[1];by_kind[kind]=max(by_kind.get(kind,0.),float(max(0.,low-value,value-high)))
    raw_dual=np.zeros(p['matrix'].shape[0]);keep=np.asarray(abs(p['matrix'][:,free]).sum(axis=1)).ravel()>1e-12
    raw_dual[keep]=result.y*scale/rows
    gradient=p['H']@x+p['q']+p['matrix'].T@raw_dual
    stat=float(np.max(abs(gradient[free])))
    complementarity=0.
    for y,value,lower,upper in zip(raw_dual,residual,p['lower'],p['upper']):
        active=upper if y>0 else lower
        if np.isfinite(active):complementarity=max(complementarity,abs(float(y*(value-active))))
    info=dict(mode=mode,status=result.info.status,iterations=int(result.info.iter),solver_primal_residual=float(result.info.prim_res),
        solver_dual_residual=float(result.info.dual_res),physical_constraint_violation=violation,violation_by_row_kind=by_kind,terminal_equation_error=terminal,hessian_min_eigenvalue=p['hessian_min'],
        complementarity_original=complementarity,stationarity_original_free=stat,accepted=bool(result.info.status_val in (1,2) and np.isfinite(x).all() and max(terminal,violation)<=SCENE['tolerance'] and p['hessian_min']>=-1e-10),
        start_monotonic_s=start,end_monotonic_s=time.perf_counter(),build_s=p['build_s'],setup_s=setup_s,solver_s=solve_s,total_s=time.perf_counter()-start+p['build_s'],
        objective=float(.5*x@p['H']@x+p['q']@x),solution=x.tolist())
    return info,dict(x=x,y=result.y,keys=p['keys'],rows=rows,scale=scale)


def main():
    (ROOT/'frozen').mkdir(exist_ok=True)
    write(ROOT/'solver_settings.json',dict(reference=dict(eps_abs=1e-8,eps_rel=1e-8,max_iter=20000,polishing=True,adaptive_rho_interval=50,
        feasibility_band=bj.BicycleController.reference_feasibility_band,acceptance_tolerance=SCENE['tolerance']),
        bicycle=dict(eps_abs=1e-7,eps_rel=1e-7,max_iter=10000,polishing=True,adaptive_rho_interval=50,deadline_s=bj.DEADLINE_S,
        numerical_band=bj.NUMERICAL_BAND,model_error_limits=bj.ERROR_LIMIT,state_trust=bj.STATE_TRUST.tolist(),input_trust=bj.INPUT_TRUST.tolist()),
        dual_mapping='semantic car/kind/step; shift nodal and input rows by one, preserve terminal kind, unmatched rows zero; convert y to original objective and row units before remapping'))
    original_provenance=json.loads((ORIGINAL/'provenance.json').read_text())
    for name in ('linearized_joint.py','longitudinal_joint.py','bicycle_core.py'):
        assert hashlib.sha256((ROOT/'source'/name).read_bytes()).hexdigest()==original_provenance['source_sha256'][name]
    derivative=[];parking=[];snapshots={}
    for branch,cycles in [('B2_single',[56]),('B2_sequence',list(range(55,62))+list(range(117,122)))]:
        for cycle in cycles:
            controller,permissions,frozen=snapshot(branch,cycle);name=f'{branch}_{cycle}';snapshots[name]=(controller,permissions,frozen)
            if cycle<=61:
                print('derivative',name,flush=True)
                derivative.append(derivative_study(controller,permissions,name))
    write(ROOT/'derivative_results.json',derivative)
    previous=None
    for cycle in range(117,122):
        controller,permissions,frozen=snapshots[f'B2_sequence_{cycle}'];p=reference_problem(controller,permissions)
        print('parking',cycle,flush=True)
        for mode in ('cold','shift_primal','shift_primal_dual','no_row_scaling'):
            if previous is None and mode.startswith('shift'):continue
            for repeat in range(3):
                info,state=solve_reference(p,mode,previous)
                if repeat==0:
                    np.savez_compressed(ROOT/'frozen'/f'B2_sequence_{cycle}_{mode}_solver.npz',x=state['x'],y=state['y'],
                        rows=state['rows'],objective_scale=state['scale'],row_keys=np.array(state['keys'],dtype=str),free=p['free'])
                acceleration={c:state['x'][i*32:(i+1)*32] for i,c in enumerate(CARS)}
                with patch.object(lj,'solve',return_value=(acceleration,{**info,'parameters':p['parameters']})):
                    commands,dense,replayed=lj.Controller.predict(controller,permissions)
                if info['accepted']:
                    mapping=max(e[k] for e in replayed['mapping_errors'].values() for k in ('position_m','heading_rad','speed_m_s'))
                    check=audit(dense,SCENE)
                    protection=[-SCENE['admission_protection_abs_x'],SCENE['admission_protection_abs_x'],-SCENE['W']/2,SCENE['W']/2]
                    permission_ok=all(permissions[c] or all(clipped_area(body,protection)<=SCENE['tolerance'] for body in corners(dense[c],SCENE['vehicle'])) for c in CARS)
                    replay_result=dict(mapping_evaluated=True,mapping_errors=replayed['mapping_errors'],safety=check,permission_passed=permission_ok,
                        path_geometry_diagnostic_passed=bool(mapping<=controller.mapping_tolerance and check['passed'] and permission_ok))
                else:replay_result=dict(mapping_evaluated=False,path_geometry_diagnostic_passed=False)
                if cycle==121 and repeat==0 and info['accepted']:
                    record={**frozen['original_record']['reference_record'],**info,'parameters':p['parameters'],
                        'free_variables':len(p['free']),'solve_wall_s':info['total_s']}
                    started=time.perf_counter()
                    with patch.object(lj,'solve',return_value=(acceleration,record)):
                        _,_,downstream=copy.deepcopy(controller).predict(permissions)
                    replay_result['downstream_bicycle_original_3_updates']=dict(record=downstream,
                        offline_bicycle_wall_s=time.perf_counter()-started,
                        measured_reference_plus_offline_bicycle_s=info['total_s']+time.perf_counter()-started)
                if mode=='cold' and repeat==0:
                    assert info['iterations']==frozen['original_record']['reference_record']['iterations'], (cycle,info['iterations'])
                parking.append(dict(cycle=cycle,repeat=repeat,state_timestamp_s=SCENE['epoch_s']+cycle*SCENE['h'],committed=False,                    rejection_layer=None if info['accepted'] else 'reference_solver',budget_remaining_s=.5-info['total_s'],replay=replay_result,**info))
                if mode=='cold' and repeat==0:current=state
        previous=current
    write(ROOT/'parking_results.json',parking)
    provenance=dict(python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,osqp=osqp.__version__,
        platform=platform.platform(),original='bicycle_e3_002/source',command="OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/t1_qp_deps python3 'examples/head_on_mpc/results/B1&B2-T1/run_t1.py'",
        repeats='deterministic timing repetitions, not scenario success rates',
        hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ROOT/'run_t1.py',ROOT/'verify_t1.py',ROOT/'solver_settings.json',*sorted((ROOT/'source').glob('*.py')),*sorted((ROOT/'frozen').glob('*'))]},
        original_hashes={str(p.relative_to(ORIGINAL)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [ORIGINAL/'scene.json',ORIGINAL/'paths.json',ORIGINAL/'provenance.json',*[ORIGINAL/b/f for b in ('B2_single','B2_sequence') for f in ('solves.jsonl','constraints.jsonl','actual.csv','predictions.jsonl')]]})
    write(ROOT/'provenance.json',provenance)
    print('T1 complete',flush=True)


if __name__=='__main__':main()
