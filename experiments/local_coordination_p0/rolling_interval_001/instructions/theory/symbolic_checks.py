"""Exact symbolic consistency checks for CURVE-JOINT-v1; no controller or solver."""
from pathlib import Path
import sys
import json

root = Path(__file__).resolve().parent
local_deps = root / '_symbolic_deps'
if local_deps.exists():
    sys.path.insert(0, str(local_deps))
import sympy as s


def main():
    checks = []
    v, delta, ell, L = s.symbols('v delta ell L', real=True, positive=True)
    a, omega, t, x = s.symbols('a omega t x', real=True)
    tpos = s.symbols('tpos', positive=True)
    f = v*s.tan(delta)/L
    expected = s.Matrix([[0, s.sec(delta)**2/L],
                         [s.sec(delta)**2/L, 2*v*s.sec(delta)**2*s.tan(delta)/L]])
    assert (s.hessian(f, (v, delta))-expected).applyfunc(s.trigsimp) == s.zeros(2)
    checks.append('heading_integrand_hessian')

    integrand = (v+a*t)*s.tan(delta+omega*t)/L
    da = s.integrate(s.diff(integrand, a).subs({a: 0, omega: 0}), (t, 0, tpos))
    dw = s.integrate(s.diff(integrand, omega).subs({a: 0, omega: 0}), (t, 0, tpos))
    daw = s.integrate(s.diff(integrand, a, omega).subs({a: 0, omega: 0}), (t, 0, tpos))
    assert s.simplify(da-tpos**2*s.tan(delta)/(2*L)) == 0
    assert s.trigsimp(dw-v*s.sec(delta)**2*tpos**2/(2*L)) == 0
    assert s.trigsimp(daw-s.sec(delta)**2*tpos**3/(3*L)) == 0
    checks.append('one_slot_exact_first_and_mixed_variations')

    C, D2, lam = s.symbols('C D2 lam', real=True)
    M = s.Matrix([[0, C], [C, D2]])
    assert M.charpoly().all_coeffs() == [1, -D2, -C**2]
    checks.append('orthogonal_block_spectral_polynomial')

    h, tau = s.symbols('h tau', positive=True)
    k = s.symbols('k', integer=True, nonnegative=True)
    S0 = h**3*k*(3*k-1)/6+k*h**2*tau+tau**3/3
    assert s.simplify(s.diff(S0, tau)-(k*h**2+tau**2)) == 0
    assert s.simplify(S0.subs(tau, h)-S0.subs({k: k+1, tau: 0}, simultaneous=True)) == 0
    checks.append('prefix_second_moment_and_seam')

    psi, b1, b2, vv, v1, q1, q2 = s.symbols('psi b1 b2 vv v1 q1 q2', real=True)
    angle = psi+q1*x+q2*x**2/2
    e = s.Matrix([s.cos(angle), s.sin(angle)])
    e0 = e.subs(x, 0)
    J = s.Matrix([[0, -1], [1, 0]])
    calc = ((vv+v1*x)*e).diff(x, 2).subs(x, 0)
    assert (calc-(2*v1*q1+vv*q2)*J*e0+vv*q1**2*e0).applyfunc(s.simplify) == s.zeros(2, 1)
    checks.append('position_second_variation_chain_rule')

    b = s.Matrix([b1, b2])
    rot = s.Matrix([[s.cos(angle), -s.sin(angle)], [s.sin(angle), s.cos(angle)]])
    R0 = rot.subs(x, 0)
    assert (s.diff(rot*b, x, 2).subs(x, 0)-R0*(J*b*q2-b*q1**2)).applyfunc(s.simplify) == s.zeros(2, 1)
    checks.append('body_corner_second_variation')

    pieces = [t**2/2, ell**2/2+ell*t-t**2, (ell-t)**2/2]
    assert [s.diff(p, t, 2) for p in pieces] == [1, -2, 1]
    assert pieces[0].subs(t, ell) == pieces[1].subs(t, 0)
    assert pieces[1].subs(t, ell) == pieces[2].subs(t, 0)
    assert pieces[2].subs(t, ell) == 0
    assert s.diff(pieces[2], t).subs(t, ell) == 0
    assert s.expand(pieces[1]-((t-ell/2)*(ell/2-t)+3*ell**2/4)) == 0
    checks.append('three_block_double_primitive_and_positivity_form')

    z = s.symbols('z', positive=True)
    transform = sum(s.exp(-z*j*ell)*s.integrate(p*s.exp(-z*t), (t, 0, ell)) for j, p in enumerate(pieces))
    assert s.simplify(transform-(1-s.exp(-z*ell))**3/z**3) == 0
    checks.append('exact_finite_transform_for_progress_gain')

    chi = s.symbols('chi', real=True, nonzero=True)
    left = (1-s.exp(-s.I*chi*ell))**3/(s.I*chi)**3
    right = 8*s.exp(-3*s.I*chi*ell/2)*(s.sin(chi*ell/2)/chi)**3
    assert s.simplify(left-right.rewrite(s.exp)) == 0
    checks.append('trigonometric_progress_coefficient_identity')

    N, m = 32, 10
    r = s.Matrix([1]*m+[-2]*m+[1]*m+[0]*(N-3*m))
    weights = s.Matrix([2*N-2*j-1 for j in range(N)])
    assert sum(r) == 0 and (weights.T*r)[0] == 0
    assert (r.T*r)[0] == 6*m
    checks.append('N32_integer_moments_and_direction_norm')

    # Rational example for the local-frame lease; geometry is a separate certificate.
    assert 26 <= 406 <= 855 <= 924
    assert max(475, 855) <= max(475, 924)
    assert max(475, 855, 1373, 1822) == 1822
    checks.append('local_frame_window_and_deadline_arithmetic')

    R = s.Rational
    horizon, step = R(32, 5), R(1, 5)
    moments = []
    for power in range(3):
        value = 0
        for j in range(32):
            for degree in range(power+1):
                value += s.binomial(power, degree)*(j*step)**(power-degree)*(j*step**2*step**(degree+1)/(degree+1)+step**(degree+3)/(degree+3))
        moments.append(s.factor(value))
    S0v, S1v, S2v = moments
    Vb, K1b, K2b = R(51, 100), R(31, 100), R(3, 5)
    assert S0v == R(304, 75)
    assert K1b**2*R(27, 10)**2 >= R(1, 2)**2+(Vb*R(5, 4))**2
    Cb, D2b = R(5, 4), 2*Vb*R(5, 4)*R(1, 2)
    eig_upper = R(27, 10)*K2b
    assert eig_upper >= D2b/2 and eig_upper**2-D2b*eig_upper-Cb**2 >= 0
    Bpv = K1b*horizon*S0v+Vb*K2b*(horizon*S0v-S1v)+Vb*K1b**2*(horizon**2*S0v-S2v)/2
    assert Bpv <= 15
    assert horizon*S0v <= R(51, 10)**2
    assert horizon**3*S0v/3 <= 19**2
    assert R(51, 10)+Vb*K1b*19 <= R(41, 5)
    assert K1b**2*horizon*S0v <= R(8, 5)**2
    assert Bpv+4*(K2b*S0v+K1b**2*horizon*S0v) <= 35
    kap, kapd = R(5, 27), R(25, 54)
    Ac = 1+Vb**2*kap+4*(kap+Vb*kapd+Vb**2*kap**2)
    assert Ac <= 3
    nominal_delta = R(422853926133, 10**12)
    sin_lo = nominal_delta-nominal_delta**3/6
    sin_hi = sin_lo+nominal_delta**5/120
    cos_hi = 1-nominal_delta**2/2+nominal_delta**4/24
    cos_lo = cos_hi-nominal_delta**6/720
    assert sin_lo/cos_hi/R(27, 10) >= R(166, 1000)
    assert sin_hi/cos_lo/R(27, 10) <= R(167, 1000)
    assert nominal_delta+R(2, 1000) < R(43, 100)
    assert R(43, 100)/(1-R(43, 100)**2/2) < R(1, 2)
    alpha = R(1, 100000)
    norm2, norm_upper = 120*alpha**2, R(11, 100000)
    assert norm2 < norm_upper**2 and norm_upper < R(1, 1000)
    geometry = R(73, 5)*norm_upper+35*norm2/2+R(3, 12800)+R(1, 10**9)+R(1, 10**9)*norm_upper
    assert geometry < R(1841, 10**6) < R(1, 100)
    assert R(41, 5)*norm_upper+15*norm2/2+R(1, 10**9) < R(903, 10**6) < R(1, 400)
    assert K2b*S0v*norm2/2+R(1, 10**10) < R(16, 10**9) < R(1, 100000)
    assert R(1, 400)+54*R(1, 100000) < R(1, 100)
    Rmax, rmin = R(603, 100), R(597, 100)
    assert R(1000, 167)-(R(41, 5)/1000+R(15, 2)/1000**2) > rmin
    assert Rmax/rmin*15+Rmax/rmin**2*R(41, 5)**2 <= 27
    chi_max, zmin, zmax = R(167, 2000), R(1411, 5000), R(2839, 10000)
    C_lower = R(20, 27)*(1-chi_max**2/6)**3*(zmin-zmax**3/6)
    assert C_lower > R(1, 5)
    assert C_lower-2*R(1, 10**9)*11 > R(19, 100)
    improvement = R(19, 100)*alpha-28*norm2/2
    assert improvement == R(1732, 10**9) > 0
    assert alpha/R(1, 10**12) == 10**7
    checks.append('fully_rational_nonzero_step_certificate')

    result = dict(passed=True, sympy_version=s.__version__, check_categories=len(checks),
                  checks=checks, optimizer_calls=0, project_control_runs=0,
                  example=dict(moments=[str(x) for x in moments], Bp_upper='15', Bc_upper='35', alpha=str(alpha),
                               certified_objective_improvement=str(improvement),
                               assumptions='Positive face and receiver margins specified in theory; not current-map evidence'),
                  scope='Exact symbolic identities and illustrative arithmetic; current-map certification separate')
    (root/'symbolic_checks.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
