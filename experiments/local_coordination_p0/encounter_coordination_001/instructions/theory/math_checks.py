"""Exact proof-consistency fixtures; no optimizer and no project control run."""
from fractions import Fraction as F
from itertools import permutations, product
from pathlib import Path
import json


def gap(a, b):
    return max(F(0), a[0]-b[1], b[0]-a[1])


def topo(n, edges):
    pending = set(range(n))
    order = []
    while pending:
        ready = sorted(i for i in pending if not any(v == i and u in pending for u, v in edges))
        if not ready:
            return None
        i = ready[0]
        pending.remove(i)
        order.append(i)
    return order


def initialization_fixture():
    h = F(1, 5)
    half_body = F(1, 10)
    safe, mu = F(1, 2), F(1, 10)
    starts = list(map(F, [0, 3, 6]))
    goals = list(map(F, [10, 11, 12]))
    homes = [(x-half_body, x+half_body) for x in starts]
    ends = [(x-half_body, x+half_body) for x in goals]
    sweeps = [(x-half_body, y+half_body) for x, y in zip(starts, goals)]
    edges = set()
    for i, j in permutations(range(3), 2):
        if gap(sweeps[i], homes[j]) < safe+mu:
            edges.add((j, i))
        if gap(sweeps[i], ends[j]) < safe+mu:
            edges.add((i, j))
    order = topo(3, edges)
    assert order == [2, 1, 0]
    occupied = dict(enumerate(homes))
    ticks = 0
    for i in order:
        assert all(gap(sweeps[i], body) >= safe+mu for j, body in occupied.items() if j != i)
        length = goals[i]-starts[i]
        m = 1
        while length/(m*h) > 2 or length/(m*m*h*h) > 1:
            m += 1
        a = length/(m*m*h*h)
        x, v = starts[i], F(0)
        for control in [a]*m + [-a]*m:
            x += h*v+h*h*control/2
            v += h*control
            assert 0 <= v <= 2 and abs(control) <= 1
        assert x == goals[i] and v == 0
        occupied[i] = ends[i]
        ticks += 2*m
    assert topo(2, {(0, 1), (1, 0)}) is None
    return dict(order=order, ticks=ticks, cyclic_dependency_detected=True,
                fixture='Analytic straight delta=0 bicycle and monotone swept intervals')


def buffer_fixture():
    # Connectors here are abstract certified-edge fixtures; tests check slot logic.
    location = {0: 'x', 1: 'y'}
    goal = {0: 'y', 1: 'x'}
    buffers = ['b0', 'b1']
    transfers = []
    done = set()
    for i in [0, 1]:
        occupied = {slot: car for car, slot in location.items()}
        if goal[i] in occupied and occupied[goal[i]] != i:
            j = occupied[goal[i]]
            assert j not in done
            free = next(b for b in buffers if b not in occupied)
            location[j] = free
            transfers.append((j, free))
        if location[i] != goal[i]:
            location[i] = goal[i]
            transfers.append((i, goal[i]))
        done.add(i)
        assert len(set(location.values())) == 2
    assert location == goal and len(transfers) <= 4
    return dict(transfers=transfers, fixture='Endpoint reservation logic only')


def main():
    checks = {}
    checks['DAG_initialization'] = initialization_fixture()
    checks['buffer_initialization'] = buffer_fixture()

    def keys(interval):
        lo, hi = interval
        return set(range(lo.numerator//lo.denominator, hi.numerator//hi.denominator+1))
    count = 0
    for x, y in product([F(i, 4) for i in range(-8, 9)], repeat=2):
        if abs(x-y) <= F(1, 2):
            assert keys((x-F(1, 4), x+F(1, 4))) & keys((y-F(1, 4), y+F(1, 4)))
            count += 1
    checks['inflated_registry_cover'] = dict(pairs=count)

    version = 1
    seen = version
    version += 1  # A second prepared intent changes the shared dependency.
    assert seen != version
    goal_interval = (2, float('inf'))
    assert goal_interval[0] <= 100 < goal_interval[1]
    checks['intent_and_permanent_goal'] = dict(stale_version_rejected=True)

    mu, total_length, radius = F(1, 10), F(4), F(1)
    rp, rt = mu/4, mu/(4*(total_length+radius))
    traversed = F(0)
    for step in [F(1), F(2), F(1)]:
        traversed += step
        propagated = rp+traversed*rt
        assert propagated+(total_length-traversed+radius)*rt <= mu/2
    checks['receiver_radius_chain'] = dict(position_radius=str(rp), heading_radius=str(rt))

    base = (F(1, 3), F(2, 3))
    delta = F(1, 10)
    xi = F(1, 12)
    n = round(xi/delta)
    q = (base[0]+delta*n, base[1]-delta*n)
    assert sum(q) == sum(base) == 1
    error2 = sum((qj-bj-dj)**2 for qj, bj, dj in zip(q, base, (xi, -xi)))
    assert error2 <= delta**2*F(2, 4)
    checks['affine_quantization_lattice'] = dict(terminal_sum=str(sum(q)), error_squared=str(error2))

    assert F(3)*F(1, 50)+F(1, 50)**2 <= F(1, 10)
    assert F(-1, 10)+F(1, 20) < 0
    checks['anchor_margin_and_radius'] = dict(zero_candidate_feasible=True)

    deadline_cases = 0
    for b in product(range(6), repeat=4):
        nxt = [max(x-1, 0) for x in b]
        assert max(nxt) == max(max(b)-1, 0)
        deadline_cases += 1
    checks['deadline_rank'] = dict(cases=deadline_cases)

    # Independent blocks: activity count is additive over feasible pool products.
    pools = [[(1, 2), (2, 1)], [(0, 3), (1, 2)]]
    chosen = [max(p, key=lambda x: (x[0], -x[1])) for p in pools]
    assert sum(c[0] for c in chosen) == max(sum(c[0] for c in cs) for cs in product(*pools))
    checks['finite_activity_composition'] = dict(passed=True)

    target, neighborhood = F(1, 3), F(1, 100)
    for level in range(20):
        mesh = F(1, 2**level)
        approximation = mesh*round(target/mesh)
        if abs(approximation-target) <= neighborhood:
            assert abs(approximation-target) <= mesh/2
            break
    else:
        raise AssertionError('Dyadic grid fixture did not enter strict neighborhood')
    checks['resolution_cover_fixture'] = dict(level=level, approximation=str(approximation))

    result = dict(passed=True, checks=checks, check_categories=len(checks),
                  optimizer_calls=0, project_control_runs=0,
                  scope='Exact illustrative consistency checks; mathematical proofs in theory.md; no current-map certification')
    out = Path(__file__).resolve().parent/'math_checks.json'
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
