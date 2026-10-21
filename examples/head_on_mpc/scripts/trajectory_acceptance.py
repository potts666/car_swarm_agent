"""Independent numeric checks; optimizer status is diagnostic, never a gate."""
import time
import numpy as np
from head_on_core import build_longitudinal_qp, matrices


def road_residual(motion, road, cfg):
    """Maximum body-containment violation in union of rectangular road pieces."""
    x, y = motion[:, 0], motion[:, 2]
    longitudinal = np.maximum(road['xmin']+cfg.length/2-x, x+cfg.length/2-road['xmax'])
    pieces = [np.maximum(road['ymin']+cfg.width/2-y, y+cfg.width/2-road['ymax'])]
    for b in road.get('bays', []):
        pieces.append(np.maximum.reduce([b['xmin']+cfg.length/2-x, x+cfg.length/2-b['xmax'],
                                        road['ymin']+cfg.width/2-y, y+cfg.width/2-b['ymax']]))
    return float(max(0., np.max(np.maximum(longitudinal, np.min(pieces, axis=0)))))


def audit_controls(state, u, target, cfg, road, front=None, rear=None):
    """Reintegrate per substep and compare to condensed states independently."""
    if np.shape(u) != (cfg.horizon,) or not np.all(np.isfinite(u)):
        return {'passed': False, 'finite': False, 'residuals': {}}
    ticks = np.arange(1, cfg.horizon*cfg.substeps+1)*cfg.dt/cfg.substeps
    p, v = matrices(ticks, cfg)
    condensed = np.column_stack((state[0]+ticks*state[1]+p@u, state[1]+v@u))
    x, speed = map(float, state)
    independent = []
    step = cfg.dt/cfg.substeps
    for acceleration in u:
        for _ in range(cfg.substeps):
            x += step*speed+.5*step**2*acceleration
            speed += step*acceleration
            independent.append([x, speed])
    independent = np.array(independent)
    direction = 1 if target >= state[0] else -1
    neighbor = 0.
    if front is not None:
        neighbor = max(neighbor, float(np.max(cfg.length+cfg.margin-(front[:, 0]-independent[:, 0]))))
    if rear is not None:
        neighbor = max(neighbor, float(np.max(cfg.length+cfg.margin-(independent[:, 0]-rear[:, 0]))))
    residuals = {
        'dynamics_position_m': float(np.max(np.abs(independent[:, 0]-condensed[:, 0]))),
        'dynamics_velocity_m_s': float(np.max(np.abs(independent[:, 1]-condensed[:, 1]))),
        'speed_m_s': float(max(0., np.max(np.abs(independent[:, 1]))-cfg.max_speed)),
        'acceleration_m_s2': float(max(0., np.max(np.abs(u))-cfg.max_acceleration)),
        'terminal_velocity_m_s': float(abs(independent[-1, 1])),
        'road_longitudinal_m': road_residual(np.column_stack((independent, np.zeros(len(ticks)))), road, cfg),
        'neighbor_m': max(0., neighbor),
        'target_overshoot_m': float(max(0., np.max(direction*(independent[:, 0]-target))-1e-7))}
    return {'passed': bool(all(r <= cfg.tolerance for r in residuals.values())),
            'finite': True, 'tolerance': cfg.tolerance, 'residuals': residuals}


def solve_qp(state, previous, warm, now, target, cfg, road, **peers):
    """Equivalent OSQP solve, using the same H, q, A, b as SLSQP."""
    import osqp
    from scipy.sparse import csc_matrix, vstack, eye
    started = time.perf_counter()
    H, q, A, b = build_longitudinal_qp(state, previous, warm, now, target,
                                     cfg=cfg, allow_reverse=True, terminal_stop=True, road=road, **peers)
    # Encode the two opposite terminal rows as one exact equality, then
    # normalize rows and objective by positive constants (same feasible set/minimizer).
    terminal = 2*cfg.horizon*cfg.substeps
    keep = np.arange(len(b)) != terminal+1
    upper = np.full(len(b), np.inf)
    upper[terminal] = b[terminal]
    row_scale = np.maximum(np.linalg.norm(A[keep], axis=1), 1.)
    objective_scale = max(1., float(np.max(np.abs(q))))
    solver = osqp.OSQP()
    solver.setup(P=csc_matrix(H/objective_scale), q=q/objective_scale,
                 A=vstack([csc_matrix(A[keep]/row_scale[:, None]), eye(cfg.horizon)], format='csc'),
                 l=np.r_[b[keep]/row_scale, np.full(cfg.horizon, -cfg.max_acceleration)],
                 u=np.r_[upper[keep]/row_scale, np.full(cfg.horizon, cfg.max_acceleration)],
                 eps_abs=1e-8, eps_rel=1e-8, max_iter=20000, polishing=False, verbose=False,
                 adaptive_rho_interval=50)
    solver.warm_start(x=warm)
    result = solver.solve(raise_error=False)
    u = result.x
    audit = audit_controls(state, u, target, cfg, road, **peers)
    report = {'solver': 'OSQP', 'version': osqp.__version__, 'message': result.info.status,
              'solver_success': result.info.status_val in (1, 2),
              'accepted': bool(result.info.status_val in (1, 2) and audit['passed']),
              'iterations': int(result.info.iter), 'solve_seconds': time.perf_counter()-started,
              'max_iterations': 20000, 'equivalent_scaling': 'positive row and objective scaling; terminal equality merged', 'hessian_min_eigenvalue': float(np.linalg.eigvalsh(H)[0]),
              'primal_residual': float(result.info.prim_res), 'dual_residual': float(result.info.dual_res),
              'constraint_violation': max(audit['residuals'].values(), default=None),
              'first_acceleration': float(u[0]), 'target': target, 'stamp': now,
              'independent_acceptance': audit,
              'objective': float(.5*u@H@u+q@u)}
    return u, report


def crosscheck_qp(state, previous, warm, now, target, cfg, road, slsqp_u, **peers):
    """Independent algorithm for identical QP; diagnostic only, no fallback."""
    u, report = solve_qp(state, previous, warm, now, target, cfg, road, **peers)
    H, q, _, _ = build_longitudinal_qp(state, previous, warm, now, target,
                                     cfg=cfg, allow_reverse=True, terminal_stop=True, road=road, **peers)
    objective = lambda z: float(.5*z@H@z+q@z)
    report.update(objective_slsqp=objective(slsqp_u), objective_qp=objective(u),
                  objective_gap=objective(slsqp_u)-objective(u),
                  max_control_difference=float(np.max(np.abs(slsqp_u-u))))
    return report
