#!/usr/bin/env python3
"""Paused project-extension research backend: quintic Hermite states + uniform segment time.

Dependencies: numpy, scipy. Runs the native Hybrid A*/S-T exporter and the same
C++ controller/model on both references. This is an offline experiment, not a
continuous-time certified planner or a ROS publishing path.
"""
import argparse
import json
from pathlib import Path
import subprocess
import time

import numpy as np
from scipy.optimize import minimize


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def coefficients(states, h):
    p, v, a = states[:, 0:2], states[:, 2:4], states[:, 4:6]
    c = np.zeros((len(p) - 1, 6, 2))
    c[:, 0] = p[:-1]
    c[:, 1] = h * v[:-1]
    c[:, 2] = h * h * a[:-1] / 2
    dp = p[1:] - c[:, 0] - c[:, 1] - c[:, 2]
    dv = h * v[1:] - c[:, 1] - 2 * c[:, 2]
    da = h * h * a[1:] - 2 * c[:, 2]
    c[:, 3] = 10 * dp - 4 * dv + da / 2
    c[:, 4] = -15 * dp + 7 * dv - da
    c[:, 5] = 6 * dp - 3 * dv + da / 2
    return c


def evaluate(c, h, local, derivative=0):
    i = np.minimum((local / h).astype(int), len(c) - 1)
    u = np.clip(local / h - i, 0, 1)
    basis = np.zeros((len(local), 6))
    for k in range(derivative, 6):
        factor = np.prod(np.arange(k - derivative + 1, k + 1))
        basis[:, k] = factor * u ** (k - derivative) / h ** derivative
    return np.einsum("ni,nij->nj", basis, c[i])


def stop_limits(c, h, gear):
    t = np.array([0.0, len(c) * h])
    a, snap = evaluate(c, h, t, 2), evaluate(c, h, t, 4)
    return gear * np.array([1., -1.]) * (a[:, 0] * snap[:, 1] - a[:, 1] * snap[:, 0]) / (
        3 * np.maximum(np.linalg.norm(a, axis=1), 1e-8) ** 3)


def rectangle_gap(p, yaw, other_p, other_yaw, other_half):
    """Signed SAT separating gap (conservative lower bound on body distance)."""
    c, s = np.cos(yaw), np.sin(yaw)
    ego_axes = np.stack((np.stack((c, s), -1), np.stack((-s, c), -1)), 1)
    center = p + 1.25 * ego_axes[:, 0]
    oc, os = np.cos(other_yaw), np.sin(other_yaw)
    other_axes = np.stack((np.stack((oc, os), -1), np.stack((-os, oc), -1)), 1)
    axes = np.concatenate((ego_axes, other_axes), 1)
    delta = other_p - center
    separation = np.abs(np.einsum("nki,ni->nk", axes, delta))
    ego_r = np.einsum("nki,nji->nkj", axes, ego_axes)
    other_r = np.einsum("nki,nji->nkj", axes, other_axes)
    return np.max(separation - np.abs(ego_r) @ np.array([2.35, 1.0]) -
                  np.sum(np.abs(other_r) * other_half[:, None, :], axis=2), axis=1)


def static_gaps(p, yaw, obstacles):
    # Exact same occupied cells as the C++ map, plus its four outer boundaries.
    c, s = np.cos(yaw), np.sin(yaw)
    center = p + 1.25 * np.stack((c, s), 1)
    half_x, half_y = obstacles[:, 2] / 2, obstacles[:, 3] / 2
    dx = obstacles[None, :, 0] + half_x - center[:, None, 0]
    dy = obstacles[None, :, 1] + half_y - center[:, None, 1]
    ac, ass = np.abs(c)[:, None], np.abs(s)[:, None]
    gap = np.maximum.reduce([
        np.abs(dx * c[:, None] + dy * s[:, None]) - 2.35 - half_x * ac - half_y * ass,
        np.abs(-dx * s[:, None] + dy * c[:, None]) - 1 - half_x * ass - half_y * ac,
        np.abs(dx) - half_x - 2.35 * ac - ass,
        np.abs(dy) - half_y - 2.35 * ass - ac])
    result = np.min(gap, axis=1)
    radius = np.stack((2.35 * np.abs(c) + np.abs(s), 2.35 * np.abs(s) + np.abs(c)), 1)
    return np.minimum(result, np.min(30 - np.abs(center) - radius, axis=1))



class Problem:
    def __init__(self, directory, time_weight=1.0, step=0.2, piece_seconds=4.0):
        self.before = load(directory / "before.csv")
        self.peer = load(directory / "peer.csv")
        obstacle = load(directory / "obstacles.csv")
        # Merge neighboring occupied cells into axis-aligned runs. Every cell
        # remains represented; this only accelerates static SAT evaluation.
        boxes = np.array([list(row) for row in obstacle])
        runs = []
        for y in np.unique(boxes[:, 1]):
            row = boxes[boxes[:, 1] == y]
            row = row[np.argsort(row[:, 0])]
            x, end = row[0, 0], row[0, 0] + row[0, 2]
            for b in row[1:]:
                if abs(b[0] - end) < 1e-8:
                    end += b[2]
                else:
                    runs.append([x, y, end - x, 0.5]); x, end = b[0], b[0] + b[2]
            runs.append([x, y, end - x, 0.5])
        self.obstacles = np.array(runs)
        self.segments, self.initial, self.bounds = [], [], []
        self.step, self.time_weight = step, time_weight
        for row in np.atleast_1d(load(directory / "segments.csv")):
            start, end, gear = float(row[0]), float(row[1]), int(row[2])
            n = max(2, int(np.ceil((end - start) / piece_seconds))) if gear else 1
            times = np.linspace(start, end, n + 1)
            states = np.array([[np.interp(t, self.before["t"], self.before[k])
                                for k in ("x", "y", "vx", "vy", "ax", "ay")] for t in times])
            headings = np.interp(times, self.before["t"], np.unwrap(self.before["yaw"]))
            # Fixed pose and signed velocity at stop/reversal boundaries.
            for j in (0, n):
                speed = np.hypot(*states[j, 2:4])
                if j == n or start > 0:
                    speed = 0  # Exact S-T stop boundary, not interpolated residual speed.
                if speed < 1e-5:
                    states[j, 2:4] = 0
                    states[j, 4:6] = (1 if j == 0 else -1) * gear * 0.35 * np.array(
                        [np.cos(headings[j]), np.sin(headings[j])])
                else:
                    states[j, 4:6] = 0
            if not gear:
                states[:, 2:6] = 0
            offset = len(self.initial)
            self.initial.append(np.log((end - start) / n))
            self.bounds.append((np.log(max(0.15, (end - start) / n * 0.45)),
                                np.log((end - start) / n * 2.5)))
            for state in states[1:-1]:
                self.initial.extend(state)
                self.bounds.extend([(state[0] - 1.5, state[0] + 1.5),
                                    (state[1] - 1.5, state[1] + 1.5),
                                    (-1.0, 1.0), (-1.0, 1.0), (-1.0, 1.0), (-1.0, 1.0)])
            self.segments.append((states, gear, headings, offset))
        self.initial = np.array(self.initial)

    def unpack(self, z):
        result, start = [], 0.0
        for fixed, gear, yaw, offset in self.segments:
            h = np.exp(z[offset]); states = fixed.copy(); n = len(states) - 1
            if n > 1:
                states[1:-1] = z[offset + 1:offset + 1 + 6 * (n - 1)].reshape(n - 1, 6)
            c = coefficients(states, h)
            result.append((start, h, gear, yaw, c))
            start += n * h
        return result

    def samples(self, z, step=None):
        result = []
        for start, h, gear, headings, c in self.unpack(z):
            total = len(c) * h
            # Fixed physical time step, always augment with both ends and knots.
            t = np.unique(np.r_[np.arange(0, total, step or self.step),
                                 np.arange(len(c) + 1) * h,
                                 1e-4, total - 1e-4])
            t = t[(t >= 0) & (t <= total)]
            p, v, a = [evaluate(c, h, t, d) for d in range(3)]
            speed = np.linalg.norm(v, axis=1)
            yaw = np.arctan2(gear * v[:, 1], gear * v[:, 0])
            yaw[speed < 1e-8] = np.interp(t[speed < 1e-8], [0, total], [headings[0], headings[-1]])
            cross = v[:, 0] * a[:, 1] - v[:, 1] * a[:, 0]
            curvature = gear * cross / np.maximum(speed, 1e-8) ** 3
            curvature[speed < 1e-8] = 0
            limits = stop_limits(c, h, gear)
            for j, boundary in enumerate((0, total)):
                curvature[(speed < 1e-8) & (np.abs(t - boundary) < 1e-8)] = limits[j]
            tangent_a = np.sum(v * a, axis=1) / np.maximum(speed, 1e-8)
            result.append((start + t, p, yaw, speed, tangent_a, a, curvature))
        return [np.concatenate([s[i] for s in result]) for i in range(7)]

    def violations(self, z, step=None):
        t, p, yaw, speed, tangent_a, a, curvature = self.samples(z, step)
        peer_p = np.column_stack([np.interp(t, self.peer["t"], self.peer[k]) for k in ("x", "y")])
        peer_yaw = np.interp(t, self.peer["t"], self.peer["yaw"])
        peer_center = peer_p + 1.25 * np.column_stack([np.cos(peer_yaw), np.sin(peer_yaw)])
        gap = rectangle_gap(p, yaw, peer_center, peer_yaw,
                            np.tile([2.35, 1.0], (len(p), 1)))
        constraints = np.column_stack((speed - 1, np.abs(tangent_a) - 1,
            np.linalg.norm(a, axis=1) - 2,
            (np.abs(np.arctan(2.7 * curvature)) - 0.5) if step is not None else
            (np.abs(curvature) - np.tan(0.5) / 2.7) * speed ** 3 / np.maximum(speed ** 3, 0.01),
            0.08 - gap, 0.03 - static_gaps(p, yaw, self.obstacles)))
        # Unknown future is a constraint, not extrapolation permission.
        return constraints, max(0.0, t[-1] - self.peer["t"][-1])

    def jerk_cost(self, z):
        # Exact integral: Gauss-Legendre with 3 points integrates quartic |jerk|².
        nodes, weights = np.polynomial.legendre.leggauss(3)
        cost, duration = 0.0, 0.0
        for _, h, _, _, c in self.unpack(z):
            for i in range(len(c)):
                local = (i + (nodes + 1) / 2) * h
                j = evaluate(c, h, local, 3)
                cost += h / 2 * np.dot(weights, np.sum(j * j, axis=1))
            duration += len(c) * h
        return cost, duration

    def heading_equalities(self, z):
        values = []
        for (fixed, gear, yaw, _), (_, h, _, _, c) in zip(self.segments, self.unpack(z)):
            if not gear:
                continue
            # At zero speed a tangent acceleration and tangent jerk give a
            # finite steering limit. Never hide the flatness singularity by
            # silently dropping constraints around the stop.
            for idx, u, angle in ((0, 0.0, yaw[0]), (-1, 1.0, yaw[-1])):
                if np.linalg.norm(fixed[idx, 2:4]) < 1e-8:
                    j = evaluate(c[[idx]], h, np.array([u * h]), 3)[0]
                    values.append(j[0] * np.sin(angle) - j[1] * np.cos(angle))
        return np.array(values)

    def write(self, z, path):
        with open(path, "w") as out:
            for _, h, gear, headings, c in self.unpack(z):
                for piece in c:
                    row = np.r_[h, gear, headings[0], piece[:, 0], piece[:, 1]]
                    out.write(" ".join(f"{v:.17g}" for v in row) + "\n")


def body_distance(p, yaw, peer_p, peer_yaw):
    """Exact Euclidean distance between the padded rectangular bodies at samples.

    Overlap is reported as zero; optimization uses a signed SAT gap separately.
    """
    def corners(pos, angle):
        c, s = np.cos(angle), np.sin(angle)
        longitudinal = np.stack((c, s), 1)
        lateral = np.stack((-s, c), 1)
        center = pos + 1.25 * longitudinal
        return np.stack([center + x * longitudinal + y * lateral
                         for x, y in ((2.35, 1), (-2.35, 1), (-2.35, -1), (2.35, -1))], 1)
    a, b = corners(p, yaw), corners(peer_p, peer_yaw)
    best = np.full(len(p), np.inf)
    for vertices, edges in ((a, b), (b, a)):
        for i in range(4):
            start, end = edges[:, i], edges[:, (i + 1) % 4]
            delta = end - start
            for j in range(4):
                point = vertices[:, j]
                u = np.clip(np.sum((point - start) * delta, axis=1) /
                            np.sum(delta * delta, axis=1), 0, 1)
                best = np.minimum(best, np.linalg.norm(point - start - u[:, None] * delta, axis=1))
    other_center = peer_p + 1.25 * np.column_stack([np.cos(peer_yaw), np.sin(peer_yaw)])
    gap = rectangle_gap(p, yaw, other_center, peer_yaw, np.tile([2.35, 1.0], (len(p), 1)))
    best[gap <= 0] = 0
    return best


def metrics(data, peer, actual=False):
    t = data["t"]
    p = np.column_stack((data["x"], data["y"]))
    peer_p = np.column_stack([np.interp(t, peer["t"], peer[k]) for k in ("x", "y")])
    peer_yaw = np.interp(t, peer["t"], peer["yaw"])
    if t[-1] > peer["t"][-1]:
        raise ValueError("Peer does not cover metric interval")
    if actual:
        tangent = np.column_stack([np.cos(data["yaw"]), np.sin(data["yaw"])])
        normal = np.column_stack([-np.sin(data["yaw"]), np.cos(data["yaw"])])
        k = np.tan(data["steering"]) / 2.7
        acc = data["acceleration"][:, None] * tangent + (k * data["speed"] ** 2)[:, None] * normal
    else:
        acc = np.column_stack((data["ax"], data["ay"]))
    # Same finite-difference estimator/0.01 s grid on all three curves. Captures
    # acceleration jumps in the S-T seed and at any mandatory stop boundary.
    jerk = np.diff(acc, axis=0) / np.diff(t)[:, None]
    value = {"duration_s": float(t[-1]), "sampled_jerk_energy": float(
        np.sum(np.sum(jerk * jerk, axis=1) * np.diff(t))),
        "min_body_distance_m": float(np.min(body_distance(p, data["yaw"], peer_p, peer_yaw)))}
    if actual:
        value.update(rms_tracking_m=float(np.sqrt(np.mean(data["position_error"] ** 2))),
                     max_tracking_m=float(np.max(data["position_error"])))
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=Path("build/car_swarm_agent/compare_spatial_temporal"))
    parser.add_argument("--output", type=Path, default=Path("log/spatial_temporal"))
    scenarios = parser.add_mutually_exclusive_group()
    scenarios.add_argument("--reverse", action="store_true", help="curved half-turn stress case")
    scenarios.add_argument("--gear-smoke", action="store_true", help="forward/reverse/forward boundary regression")
    parser.add_argument("--time-weight", type=float, default=1.0)
    parser.add_argument("--iterations", type=int, default=200)
    args = parser.parse_args()
    if args.time_weight <= 0 or args.iterations < 1:
        parser.error("time-weight and iterations must be positive")
    directory = args.output.resolve(); directory.mkdir(parents=True, exist_ok=True)
    for filename in ("coefficients.txt", "comparison.json", "after.csv", "after_actual.csv",
                     "after_actual.json", "optimization.json", "candidate.npy"):
        (directory / filename).unlink(missing_ok=True)
    binary = str(args.binary.resolve())
    subprocess.run([binary, "--export", str(directory)] + (["reverse"] if args.reverse else ["gear-smoke"] if args.gear_smoke else []), check=True)
    problem = Problem(directory, args.time_weight, piece_seconds=2.0 if args.reverse else 4.0)
    z = problem.initial.copy(); start = time.monotonic(); history = []
    # Smooth violation penalty + a final constrained polishing pass. Time and
    # all free Cartesian knot position/velocity/acceleration states vary jointly.
    eq = problem.heading_equalities(z)
    equalities = [{"type": "eq", "fun": problem.heading_equalities}] if len(eq) else []
    for weight in (100, 10000):
        def objective(x):
            jerk, duration = problem.jerk_cost(x)
            violations, future = problem.violations(x)
            penalty = problem.step * np.sum(np.maximum(violations, 0) ** 2) + future ** 2
            return jerk + args.time_weight * duration + weight * penalty
        result = minimize(objective, z, method="SLSQP", bounds=problem.bounds,
                          constraints=equalities, options={"maxiter": args.iterations, "ftol": 1e-7})
        z = result.x
        history.append({"penalty_weight": weight, "success": bool(result.success),
                        "message": result.message, "iterations": result.nit})
        print(history[-1], flush=True)
    # Polishing uses a fixed normalized sample count so constraint vector size
    # stays constant for SLSQP. The fixed physical grid remains in the
    # penalty objective; the independent finer check is the acceptance gate.
    sample_count = 101
    def fixed_constraints(x):
        values = []
        for start_t, h, gear, headings, c in problem.unpack(x):
            ts = np.linspace(0, len(c) * h, sample_count)
            p, v, a = [evaluate(c, h, ts, d) for d in range(3)]
            speed = np.linalg.norm(v, axis=1)
            yaw = np.arctan2(gear * v[:, 1], gear * v[:, 0])
            yaw[speed < 1e-8] = np.interp(ts[speed < 1e-8], [0, ts[-1]], [headings[0], headings[-1]])
            k = gear * (v[:, 0] * a[:, 1] - v[:, 1] * a[:, 0]) / np.maximum(speed, 1e-8) ** 3
            k[speed < 1e-8] = 0
            limits = stop_limits(c, h, gear)
            for j, boundary in enumerate((0, ts[-1])):
                k[(speed < 1e-8) & (np.abs(ts - boundary) < 1e-8)] = limits[j]
                if gear and speed[0 if j == 0 else -1] < 1e-8:
                    values.append(np.array([np.tan(0.5) / 2.7 - abs(limits[j])]))
            tangent_a = np.sum(v * a, axis=1) / np.maximum(speed, 1e-8)
            peer_p = np.column_stack([np.interp(start_t + ts, problem.peer["t"], problem.peer[key]) for key in ("x", "y")])
            peer_yaw = np.interp(start_t + ts, problem.peer["t"], problem.peer["yaw"])
            center = peer_p + 1.25 * np.column_stack((np.cos(peer_yaw), np.sin(peer_yaw)))
            gap = rectangle_gap(p, yaw, center, peer_yaw, np.tile([2.35, 1.0], (len(ts), 1)))
            values.extend([1 - 0.0002 * np.sin(np.pi * ts / ts[-1]) ** 2 - speed, 1 - np.abs(tangent_a), 2 - np.linalg.norm(a, axis=1),
                           (np.tan(0.5) / 2.7 - np.abs(k)) * speed ** 3 / np.maximum(speed ** 3, 0.01), gap - 0.10,
                           static_gaps(p, yaw, problem.obstacles) - 0.03])
        values.append(np.array([problem.peer["t"][-1] - problem.jerk_cost(x)[1]]))
        return np.concatenate(values)
    for sample_count in (101, 301, 901):
        result = minimize(lambda x: sum(np.array(problem.jerk_cost(x)) * [1, args.time_weight]),
                          z, method="SLSQP", bounds=problem.bounds,
                          constraints=equalities + [{"type": "ineq", "fun": fixed_constraints}],
                          options={"maxiter": args.iterations, "ftol": 1e-9})
        z = result.x
        violations, future = problem.violations(z, step=0.01)
        maximum = max(float(np.max(violations)), future)
        accepted = bool(result.success and maximum <= 1e-5 and
                        np.max(np.abs(problem.heading_equalities(z)), initial=0) <= 1e-5)
        print(f"polish samples/segment={sample_count}: max fine violation={maximum:g}", flush=True)
        if accepted:
            break
    np.save(directory / "candidate.npy", z)
    report = {"accepted_at_samples": accepted, "continuous_plan_certified": False,
              "solver_seconds": time.monotonic() - start, "time_weight": args.time_weight,
              "max_fine_grid_violation": maximum,
              "violation_max_by_constraint": np.max(violations, axis=0).tolist(), "solver_success": bool(result.success),
              "solver_message": result.message, "history": history,
              "initial_quintic_jerk_energy": problem.jerk_cost(problem.initial)[0],
              "initial_quintic_max_violation": float(np.max(problem.violations(problem.initial, 0.01)[0])),
              "initial_quintic_objective": sum(np.array(problem.jerk_cost(problem.initial)) * [1, args.time_weight]),
              "optimized_objective": sum(np.array(problem.jerk_cost(z)) * [1, args.time_weight]),
              "plan_check_scope": "independent sampled checks, not continuous-time certification",
              "jerk_metric_scope": "finite differences with actual intervals at nominal dt=.01, including stop acceleration jumps",
              "optimized_analytic_jerk_energy": problem.jerk_cost(z)[0],
              "comparison_dt_s": 0.01, "penalty_dt_s": problem.step,
              "body_distance_includes_margin_m_per_car": 0.1}
    (directory / "optimization.json").write_text(json.dumps(report, indent=2) + "\n")
    if not accepted:
        raise RuntimeError(f"Optimization rejected; max violation {maximum:g}: {result.message}")
    problem.write(z, directory / "coefficients.txt")
    try:
        subprocess.run([binary, "--evaluate", str(directory)], check=True)
    except subprocess.CalledProcessError:
        (directory / "coefficients.txt").unlink(missing_ok=True)
        report.update(accepted_at_samples=False, cpp_sample_check_passed=False)
        (directory / "optimization.json").write_text(json.dumps(report, indent=2) + "\n")
        raise
    report["cpp_sample_check_passed"] = True
    (directory / "optimization.json").write_text(json.dumps(report, indent=2) + "\n")
    report["before_plan"] = metrics(load(directory / "before.csv"), problem.peer)
    report["after_plan"] = metrics(load(directory / "after.csv"), problem.peer)
    for name in ("before_actual", "after_actual"):
        report[name] = metrics(load(directory / f"{name}.csv"), problem.peer, actual=True)
        report[name].update(json.loads((directory / f"{name}.json").read_text()))
    (directory / "comparison.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
