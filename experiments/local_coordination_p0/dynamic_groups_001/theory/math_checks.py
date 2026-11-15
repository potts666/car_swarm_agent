"""Exact algebra checks supporting the text proofs; no optimizer or simulation."""
import itertools
import json
from fractions import Fraction as F
from pathlib import Path


def dot(a, b):
    return sum((x * y for x, y in zip(a, b)), F(0))


def integral(coefficients, end):
    return sum((a * end ** (j + 1) / (j + 1) for j, a in enumerate(coefficients)), F(0))


def main():
    results = []
    h = F(1, 5)
    # K matches the saved next velocity and steering, including reverse/circular motion.
    count = 0
    for vr, dr, ar, wr, ev, ed in itertools.product(
            (F(-1, 2), F(0), F(1, 2)), (F(-2, 5), F(0), F(2, 5)),
            (F(-1, 5), F(0), F(1, 5)), (F(-1, 5), F(0), F(1, 5)),
            (F(-1, 100), F(1, 100)), (F(-1, 1000), F(1, 1000))):
        a, w = ar - ev / h, wr - ed / h
        assert vr + ev + h * a == vr + h * ar
        assert dr + ed + h * w == dr + h * wr
        assert integral([F(1), -1 / h], h) == h / 2
        count += 1
    results.append(dict(name='signed_K_and_nonzero_reference', cases=count, passed=True))

    for q in (F(0), F(1, 100), F(3, 5)):
        assert q * (h - h * h / (2 * h)) == h * q / 2
        assert integral([F(0), q, -q / (2 * h)], h) == h * h * q / 3
    results.append(dict(name='feedback_integral_coefficients', cases=3, passed=True))

    # Counterexample: both agents consume the entire old-neighbor clearance reserve.
    ds, baseline = F(1, 2), F(7, 10)
    move = F(3, 20)
    assert baseline - move >= ds
    assert baseline - 2 * move < ds
    e1, e2 = F(2, 25), F(3, 25)
    assert baseline - e1 - e2 == ds
    results.append(dict(name='simultaneous_motion_budget', passed=True,
                        individual_pass=True, unchecked_combination_fails=True,
                        allocated_combination_pass=True))

    old = (F(-1, 4), F(1, 4))
    new = (F(1, 20), F(11, 20))
    assert old[1] - old[0] == new[1] - new[0] == ds
    assert old[1] - new[0] < ds
    results.append(dict(name='two_level_fallback_counterexample', passed=True,
                        mixed_gap=str(old[1]-new[0]), valid_gap=str(ds)))

    # Local quadratic improvement in a nontrivial equality subspace.
    gradient = (F(1), F(-1), F(2), F(0))
    p = tuple(-x for x in gradient)
    alpha, rho = F(1, 1000), F(1, 100)
    delta = tuple(alpha * x for x in p)
    assert delta[0] + delta[1] == 0
    assert dot(delta, delta) <= rho * rho
    cost_change = dot(gradient, delta) + dot(delta, delta) / 2
    assert cost_change < 0
    assert F(2) * rho + F(3, 2) * rho * rho < F(1, 10)
    assert cost_change == -alpha * dot(p, p) + alpha * alpha * dot(p, p) / 2
    results.append(dict(name='positive_radius_and_projected_descent', passed=True,
                        cost_change=str(cost_change)))

    # Maximum completion deadline shifts once per physical tick, regardless of group size.
    count = 0
    for deadlines in itertools.product(range(6), repeat=4):
        B = max(deadlines)
        shifted = tuple(max(x-1, 0) for x in deadlines)
        assert max(shifted) == max(B-1, 0)
        count += 1
    old_b, accepted_b = (10, 8, 6, 4), (9, 10, 5, 6)
    blocks = ((0, 1), (2, 3))
    assert all(max(accepted_b[i] for i in D) <= max(old_b[i] for i in D) for D in blocks)
    assert max(accepted_b) <= max(old_b)
    results.append(dict(name='local_max_to_global_deadline', cases=count,
                        local_transaction_check=True, passed=True))

    # A nonzero continuation command, rather than an automatic zero tail.
    commands = [(F(1, 5), F(1, 10)), (F(-1, 10), F(1, 20)),
                (F(0), F(-1, 10)), (F(3, 20), F(1, 5))]
    v0, d0 = F(-1, 4), F(2, 5)
    v1, d1 = v0 + h*commands[0][0], d0 + h*commands[0][1]
    shifted = commands[1:]
    assert shifted[-1] != (F(0), F(0))
    assert v1 + h*sum(u[0] for u in shifted) == v0 + h*sum(u[0] for u in commands)
    assert d1 + h*sum(u[1] for u in shifted) == d0 + h*sum(u[1] for u in commands)
    results.append(dict(name='actual_tail_shift_with_nonzero_continuation', passed=True))

    # The parabola attaining the chord-error bound: t(h-t) = h^2/4 - (t-h/2)^2.
    for t in (F(0), h/4, h/2, h):
        assert t*(h-t) == h*h/4 - (t-h/2)**2
        assert t*(h-t) <= h*h/4
    results.append(dict(name='continuous_chord_bound_identity', passed=True))
    value = dict(passed=True, scope='Exact arithmetic consistency checks; proofs in complete derivation, no map certification',
                 checks=results, optimization_calls=0, controller_runs=0)
    (Path(__file__).resolve().parent/'math_checks.json').write_text(
        json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(value, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
