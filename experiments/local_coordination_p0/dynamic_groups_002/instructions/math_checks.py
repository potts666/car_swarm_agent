"""Exact consistency examples for MA-DMPC; no optimizer or control experiment."""
from fractions import Fraction as F
from itertools import product
import json


def choose(pool):
    valid = [p for p in pool if p['safe'] and p['b'] <= p['cap']]
    return min(valid, key=lambda p: (-p['active'], -p['progress'], p['b'], p['cost'], p['id']))


def main():
    old = dict(id=0, active=2, progress=F(2), b=10, cap=10, cost=1, safe=True)
    new = dict(id=1, active=3, progress=F(3), b=10, cap=10, cost=2, safe=True)
    unsafe = dict(id=2, active=4, progress=F(4), b=9, cap=10, cost=0, safe=False)
    delayed = dict(id=3, active=4, progress=F(4), b=11, cap=10, cost=0, safe=True)
    assert choose([old, new, unsafe, delayed])['id'] == 1
    assert choose([old, unsafe, delayed])['id'] == 0
    pools = [[old, new], [dict(old, active=0), dict(new, active=1)]]
    independent = sum(choose(p)['active'] for p in pools)
    joint = max(sum(p['active'] for p in ps) for ps in product(*pools))
    assert independent == joint == 4

    # Straight delta=0 bicycle motion gives this exact rational position update.
    h = F(1, 5)
    controls = [F(1), F(0), F(-1)]
    def flow(wait):
        x = v = F(0)
        motion = []
        for k, a in enumerate([F(0)] * wait + controls):
            x += h*v + h*h*a/2
            v += h*a
            if k >= wait:
                motion.append((x, v))
        return motion
    assert flow(5) == flow(3)
    assert h*h/2 == F(1, 50)

    cases = 0
    for b in product(range(5), repeat=4):
        nxt = tuple(max(x-1, 0) for x in b)
        assert max(nxt) == max(max(b)-1, 0)
        cases += 1

    # Keeping only singleton proposals misses a coordinated improvement.
    singleton_scores = [2, 2, 2]
    bundled_scores = singleton_scores + [3]
    assert max(bundled_scores) > max(singleton_scores)
    print(json.dumps(dict(passed=True, checks=5, deadline_cases=cases,
                          optimizer_calls=0, controller_runs=0,
                          scope='Exact illustrative consistency checks; proof and map certification separate')))


if __name__ == '__main__':
    main()
