"""Analytic route-family certificate; integer outward intervals, no simulation."""
import hashlib
import itertools
import json
import math
from fractions import Fraction as F
from pathlib import Path
from interval_source import I, Q

ROOT = Path(__file__).resolve().parent
BODY_ERROR = F(1, 20)
SAFE = F(1, 2)
SOURCE_INITIAL = {'A1': (-35, 3), 'A2': (-39, -3), 'B1': (37, 3), 'B2': (40, -3)}
INITIAL = {'A1': (-30, 3), 'A2': (-30, -3), 'B1': (30, 3), 'B2': (30, -3)}


def sqrt_upper(x, grid=10**15):
    x = F(x)
    n = math.isqrt(x.numerator * grid * grid // x.denominator)
    while F(n * n, grid * grid) < x:
        n += 1
    return F(n, grid)


SQ7 = I.raw(math.isqrt(7 * Q * Q), math.isqrt(7 * Q * Q) + 1)
TRAVEL = 3 * SQ7
HALF_TAN = I(1) / SQ7


def interval(lo, hi):
    return I.raw(I(lo).l, I(hi).u)


def body(x, y, co, si, direction):
    return [(direction * (x + co * bx - si * by),
             direction * (y + si * bx + co * by))
            for bx, by in itertools.product((F(-1), F(7, 2)), (F(-9, 10), F(9, 10)))]


def parked(car, done=False):
    direction = 1 if car[0] == 'A' else -1
    x, y = INITIAL[car]
    return body(I(46 if done else -abs(x)), I(direction * y), I(1), I(0), direction)


def segments(car):
    direction = 1 if car[0] == 'A' else -1
    x, y = INITIAL[car]
    y = direction * y
    pre = I(-23) - TRAVEL
    end_pre = I(46) - TRAVEL
    result = [('staging', 'line', I(-abs(x)), I(y), pre, I(y), 0)]
    for tag, xx, yy, endx, endy, sign in (
            ('merge', pre, I(y), I(-23), I(0), -1 if y > 0 else 1),
            ('parking', end_pre, I(0), I(46), I(y), 1 if y > 0 else -1)):
        if tag == 'parking':
            result += [('approach', 'line', I(-23), I(0), I(-22), I(0), 0),
                       ('crossing', 'line', I(-22), I(0), I(34), I(0), 0),
                       ('exit', 'line', I(34), I(0), end_pre, I(0), 0)]
        result += [(tag + '_first', 'first', xx, yy, endx, endy, sign),
                   (tag + '_second', 'second', xx, yy, endx, endy, sign)]
    return result


def moving(car, segment, lower, upper):
    tag, kind, x0, y0, x1, y1, sign = segment
    t = interval(lower, upper)
    if kind == 'line':
        x, y, co, si = x0 + (x1 - x0) * t, y0, I(1), I(0)
    else:
        r = HALF_TAN * t
        r2 = r * r
        co, si = (I(1) - r2) / (I(1) + r2), 2 * r / (I(1) + r2)
        if kind == 'first':
            x, y = x0 + 6 * si, y0 + sign * 6 * (I(1) - co)
        else:
            x, y = x1 - 6 * si, y1 - sign * 6 * (I(1) - co)
        si = sign * si
    direction = 1 if car[0] == 'A' else -1
    return body(x, y, co, si, direction), (co, si)


def road_clearance(corners):
    xmin = min(F(x.l, Q) for x, _ in corners)
    xmax = max(F(x.u, Q) for x, _ in corners)
    ymin = min(F(y.l, Q) for _, y in corners)
    ymax = max(F(y.u, Q) for _, y in corners)
    return max(min(xmin - lo, hi - xmax, ymin - bot, top - ymax) - BODY_ERROR
               for lo, hi, bot, top in ((-52, 52, F(-7, 5), F(7, 5)),
                                         (-52, -10, F(-11, 2), F(11, 2)),
                                         (10, 52, F(-11, 2), F(11, 2))))


def pair_clearance(left, right, rotation=(I(1), I(0))):
    co, si = rotation
    c, s = F(co.l + co.u, 2 * Q), F(si.l + si.u, 2 * Q)
    axes = ((F(1), F(0)), (F(0), F(1)), (c, s), (-s, c))
    best = F(-1000)
    for nx, ny in axes:
        norm = sqrt_upper(nx * nx + ny * ny)
        if norm == 0:
            continue
        lp = [nx * x + ny * y for x, y in left]
        rp = [nx * x + ny * y for x, y in right]
        gap = max(min(x.l for x in rp) - max(x.u for x in lp),
                  min(x.l for x in lp) - max(x.u for x in rp))
        if gap >= 0:
            best = max(best, F(gap, Q) / norm - 2 * BODY_ERROR)
    return best


def certify(car, segment):
    stack = [(F(0), F(1), 0)]
    leaves = []
    while stack:
        lo, hi, depth = stack.pop()
        corners, rotation = moving(car, segment, lo, hi)
        road = road_clearance(corners)
        checks = {other + ':' + state: pair_clearance(corners, parked(other, state == 'done'), rotation)
                  for other in INITIAL if other != car for state in ('waiting', 'done')}
        gap = min(checks.values())
        if road >= 0 and gap >= SAFE:
            leaves.append({'lo': str(lo), 'hi': str(hi), 'road_lower': str(road),
                           'pair_lower': str(gap)})
        elif depth < 15:
            mid = (lo + hi) / 2
            stack.extend(((lo, mid, depth + 1), (mid, hi, depth + 1)))
        else:
            return {'passed': False, 'car': car, 'segment': segment[0], 'interval': [str(lo), str(hi)],
                    'road': str(road), 'pairs': {k: str(v) for k, v in checks.items()}}
    leaves.sort(key=lambda row: F(row['lo']))
    assert leaves[0]['lo'] == '0' and leaves[-1]['hi'] == '1'
    assert all(a['hi'] == b['lo'] for a, b in zip(leaves, leaves[1:]))
    return {'passed': True, 'car': car, 'segment': segment[0], 'leaves': leaves,
            'minimum_road': min(float(F(x['road_lower'])) for x in leaves),
            'minimum_pair': min(float(F(x['pair_lower'])) for x in leaves)}


def main():
    assert F(1807, 500) ** 2 >= F(653, 50)
    checks = []
    for a, b in itertools.combinations(INITIAL, 2):
        for da, db in itertools.product((False, True), repeat=2):
            clearance = pair_clearance(parked(a, da), parked(b, db))
            checks.append({'cars': [a, b], 'done': [da, db], 'gap': str(clearance)})
            assert clearance >= SAFE
    results = []
    for car in INITIAL:
        for segment in segments(car):
            item = certify(car, segment)
            results.append(item)
            if not item['passed']:
                print(json.dumps(item, ensure_ascii=False))
                (ROOT / 'geometry_counterexample.json').write_text(json.dumps(item, indent=2), encoding='utf-8')
                return 1
    value = {'passed': True, 'body_tube_radius_m': str(BODY_ERROR),
             'scope': 'Whole-parameter interval geometric certificate; exact analytic ZOH command construction described separately',
             'interval_source_sha256': hashlib.sha256((ROOT / 'interval_source.py').read_bytes()).hexdigest(),
             'static_checks': checks, 'segments': results}
    (ROOT / 'geometry_certificate.json').write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'passed': True, 'segments': len(results), 'leaves': sum(len(x['leaves']) for x in results),
                      'minimum_road': min(x['minimum_road'] for x in results),
                      'minimum_pair': min(x['minimum_pair'] for x in results)}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
