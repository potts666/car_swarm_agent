"""Offline certificate for E-normal; candidate synthesis is separate from rigorous verification."""
import bisect
import csv
import itertools
import json
import math
import sys
from pathlib import Path
from fractions import Fraction as F

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
ROUTE = ROOT / 'M2b_正式优化问题' / 'continuous_connection_certificate.json'
CFG = ROOT / 'M2b_transition_parallel_001/sources/996f85af9c01baaacb63b3138121e87d9191766c57ded15e746dd171c19fb422/config/m2b_fourcar.json'
H = F(1, 5)
L = F(27, 10)
RB = F(1807, 500)
Q = 10**20


class I:
    """Fixed-point closed interval; all operations round outward with integers."""
    __slots__ = ('l', 'u')

    def __init__(self, x=0):
        x = F(x) * Q
        self.l, self.u = x.numerator // x.denominator, -(-x.numerator // x.denominator)

    @classmethod
    def raw(cls, lo, hi):
        r = object.__new__(cls)
        r.l, r.u = lo, hi
        return r

    def __add__(x, y):
        y = iv(y)
        return I.raw(x.l+y.l, x.u+y.u)

    __radd__ = __add__

    def __neg__(x):
        return I.raw(-x.u, -x.l)

    def __sub__(x, y):
        return x + -iv(y)

    def __mul__(x, y):
        y = iv(y)
        z = (x.l*y.l, x.l*y.u, x.u*y.l, x.u*y.u)
        return I.raw(min(z)//Q, -(-max(z)//Q))

    __rmul__ = __mul__

    def __truediv__(x, y):
        y = iv(y)
        assert y.l > 0
        z = [(a*Q, b) for a in (x.l, x.u) for b in (y.l, y.u)]
        return I.raw(min(a//b for a, b in z), max(-(-a//b) for a, b in z))

    def mid(x):
        return (x.l+x.u)/(2*Q)

    def abs_upper(x):
        return F(max(-x.l, x.u), Q)


def iv(x):
    return x if isinstance(x, I) else I(x)


def trig(x, odd):
    # Taylor on the midpoint, followed by the global 1-Lipschitz bound.
    assert x.abs_upper() <= 1
    m = (x.l+x.u)//2
    t = I.raw(m, m)
    t2 = -(t*t)
    term = t if odd else I(1)
    val = term
    for k in range(1, 10):
        n = 2*k + odd
        term = term*t2 / ((n-1)*n)
        val += term
    # Remainder <= 1/20! < 42 fixed-point units.
    rad = max(m-x.l, x.u-m)+42
    return I.raw(val.l-rad, val.u+rad)


def sincos(x):
    return trig(x, True), trig(x, False)


def tan(x):
    si, co = sincos(x)
    return si/co


def profile(leg, start, end, vmax=1.95):
    """Floating point proposes rational v/delta knots; only the verifier accepts them."""
    x, y, _ = leg['start']
    xx, yy = leg['end']
    dx, dy = xx-x, yy-y
    def d(u):
        return 30*dy*u*u*(1-u)**2
    def speed(u):
        return math.hypot(dx, d(u))
    grid = 2048
    ss = [0.]
    for j in range(grid):
        ss.append(ss[-1]+(speed(j/grid)+4*speed((j+.5)/grid)+speed((j+1)/grid))/(6*grid))
    length = ss[-1]
    for n in range(max(2, int(length/vmax/.2)), 2000):
        cap = lambda j: end+.18*(n-j) if end or j==n else .1+.18*(n-j-1)
        ceiling = [min(vmax, start+.18*j, cap(j)) for j in range(n+1)]
        floor = [max(.1, start-.18*j, end-.18*(n-j)) for j in range(n+1)]
        floor[0], floor[-1] = start, end
        low = .2*(sum(floor[1:-1])+(start+end)/2)
        high = .2*(sum(ceiling[1:-1])+(start+end)/2)
        if low <= length <= high:
            break
    else:
        raise ValueError('No profile')
    mix = (length-low)/(high-low)
    vs = [F(round((a+mix*(b-a))*10**8), 10**8) for a, b in zip(floor, ceiling)]
    ds = [F(0)]
    s = 0.
    for j in range(1, n):
        s += .1*float(vs[j-1]+vs[j])
        k = min(grid-1, bisect.bisect_right(ss, s)-1)
        u = (k+(s-ss[k])/(ss[k+1]-ss[k]))/grid
        dd = 60*dy*u*(1-u)*(1-2*u)
        curvature = dx*dd/speed(u)**3
        ds.append(F(round(math.atan(2.7*curvature)*10**8), 10**8))
    return vs, ds+[F(0)]


def shoot(v, d, pose, end, s, target_heading=0.):
    """Correct endpoint drift in the candidate only; no float result serves as proof."""
    vf, df = list(map(float,v)), list(map(float,d))
    n = len(v)-1
    weights = [(math.sin(math.pi*j/n)**2, math.sin(math.pi*j/n),
                math.sin(2*math.pi*j/n)) for j in range(n+1)]
    weights[0] = weights[-1] = (0.,0.,0.)
    if v[-1] == 0:
        weights[-2] = (0.,*weights[-2][1:])

    def knots(k):
        return ([x+k[0]*w[0] for x,w in zip(vf,weights)],
                [x+k[1]*w[1]+k[2]*w[2] for x,w in zip(df,weights)])

    def endpoint(k):
        vs, ds = knots(k)
        x,y,th = (q.mid() for q in pose)
        dt = .025
        for j in range(n):
            def rate(t):
                v = vs[j]+(vs[j+1]-vs[j])*t
                d = ds[j]+(ds[j+1]-ds[j])*t
                return v, v*math.tan(d)/2.7
            for l in range(8):
                t = l/8
                v0, a = rate(t)
                vm, b = rate(t+1/16)
                v1, c = rate(t+1/8)
                m = th+dt*(a+4*rate(t+1/32)[1]+b)/12
                z = th+dt*(a+4*b+c)/6
                x += s*dt*(v0*math.cos(th)+4*vm*math.cos(m)+v1*math.cos(z))/6
                y += s*dt*(v0*math.sin(th)+4*vm*math.sin(m)+v1*math.sin(z))/6
                th = z
        return [x-end[0],y-end[1],th-target_heading]

    k = [0.,0.,0.]
    for _ in range(4):
        b = endpoint(k)
        if max(map(abs,b)) < 1e-8:
            break
        cols = []
        for i in range(3):
            z = k.copy()
            z[i] += 1e-6
            cols.append([(x-y)/1e-6 for x,y in zip(endpoint(z),b)])
        mat = [list(rr)+[-z] for rr,z in zip(zip(*cols),b)]
        for i in range(3):
            p = max(range(i,3),key=lambda j:abs(mat[j][i]))
            mat[p],mat[i] = mat[i],mat[p]
            row = mat[i]
            mat[i] = [x/row[i] for x in row]
            for j in range(3):
                if j != i:
                    mat[j] = [x-mat[j][i]*y for x,y in zip(mat[j],mat[i])]
        k = [x+rr[-1] for x,rr in zip(k,mat)]
    return tuple([F(round(x*10**8),10**8) for x in rr] for rr in knots(k))


def simpson(f0, fm, f1, dt, bound):
    e = I(bound*dt**5/2880).u
    q = (f0+4*fm+f1)*(dt/6)
    return I.raw(q.l-e, q.u+e)


def propagate(z, v0, v1, d0, d1, s, sub):
    """Return validated pose nodes on one actual ZOH tick."""
    a, w = (v1-v0)/H, (d1-d0)/H
    dt = H/sub
    fine = dt/2
    v = [v0+a*(j*fine/2) for j in range(4*sub+1)]
    rate = [I(v[j]/L)*tan(I(d0+w*(j*fine/2))) for j in range(4*sub+1)]
    x, y, th = z
    out = [z]
    for j in range(sub):
        i = 4*j
        thm = th+simpson(*rate[i:i+3], fine, 84)
        th1 = thm+simpson(*rate[i+2:i+5], fine, 84)
        parts = [(si*(s*v[k]), co*(s*v[k]))
                 for k, phi in ((i, th), (i+2, thm), (i+4, th1))
                 for si, co in [sincos(phi)]]
        x += simpson(*(co for si, co in parts), dt, 256)
        y += simpson(*(si for si, co in parts), dt, 256)
        th = th1
        out.append((x, y, th))
    return out


def corners(z, s):
    x, y, th = z
    si, co = sincos(th)
    return [(x+s*(a*co-b*si), y+s*(a*si+b*co))
            for a in (-1, F(7, 2)) for b in (F(-9, 10), F(9, 10))]


def axis_box(cs):
    return min(x.l for x,y in cs), max(x.u for x,y in cs), min(y.l for x,y in cs), max(y.u for x,y in cs)


def road(bb, roads):
    x, X, y, Y = bb
    return max(min(x-a, b-X, y-c, d-Y) for a,b,c,d in roads)


def rear(bb, s):
    return bb[0] if s == 1 else -bb[1]


def gap(ci, cj, bi, bj, axes):
    x,X,y,Y = bi
    a,A,b,B = bj
    best = max(a-X,x-A,b-Y,y-B)
    if best > 2*Q:
        return best
    for nx,ny in axes:
        p = [x*nx+y*ny for x,y in ci]
        q = [x*nx+y*ny for x,y in cj]
        raw = max(min(z.l for z in p)-max(z.u for z in q),
                  min(z.l for z in q)-max(z.u for z in p))
        # Each supplied direction has Euclidean norm <= 1.
        best = max(best, raw)
    return best


def normed_axes(z):
    # Float proposes integer axis; exact squared norm proves validity.
    a = z[2].mid()
    c, s = (round(.999999*math.cos(a)*10**6), round(.999999*math.sin(a)*10**6))
    assert c*c+s*s <= 10**12
    return [(F(c,10**6), F(s,10**6)), (F(-s,10**6),F(c,10**6))]


def increase(p, th, rv, rd):
    # Exact componentwise incremental bounds with endpoint-reset feedback.
    return p+H*rv/2+2*H*th+2*H**2*(rv+rd)/3, th+H*(rv+rd)/2


def main(verify=False):
    cfg = json.loads(CFG.read_text(), parse_float=F)
    legs = json.loads(ROUTE.read_text())['legs']
    assert len(legs) == 14 and cfg['h'] == H and cfg['N'] == 32
    assert cfg['s_min'] == F(1,2) and cfg['vehicle']['L'] == L
    params = cfg['vehicle']
    assert (params['v_fwd'],params['a_min'],params['a_max'],params['delta_max'],params['omega_max']) == (2,-1,1,F(1,2),1)
    assert (params['rear'],L+params['front'],params['width']/2) == (1,F(7,2),F(9,10))
    assert RB**2 > F(7,2)**2+F(9,10)**2
    assert cfg['admission_protection_abs_x'] == 18 and cfg['release_plane_abs_x']+cfg['release_margin_m'] == F(53,2)
    assert F(1,math.factorial(20)) < F(42,Q)
    assert (2*80+4*16)/L < 84 and 2*(17+4*5+3*4+6*2+1)+4*(5+3*2+1) < 256
    csv_path = HERE/'route_zoh_ticks.csv'
    summary_path = HERE/'route_zoh_summary.json'
    saved = {}
    if verify:
        with csv_path.open(encoding='utf-8-sig',newline='') as f:
            old = list(csv.DictReader(f))
        for j,r in enumerate(old):
            assert int(r['tick']) == j
            saved.setdefault(int(r['leg']), []).append(r)
        assert set(saved) == set(range(1,15))
    cars = tuple(cfg['initial'])
    pairs = list(itertools.combinations(cars, 2))
    pose = {c: tuple(I(q) for q in z[:2])+(I(0),) for c,z in cfg['initial'].items()}
    sign = {c: 1 if c[0]=='A' else -1 for c in cars}
    pi_lo = F('3.14159265358979323846264338327950288')
    pi_hi = pi_lo+F(1,10**35)
    for c in cars:
        angle = F(cfg['initial'][c][2])
        if sign[c] == 1:
            assert angle == 0
        else:
            pose[c] = (*pose[c][:2], I.raw(I(angle-pi_hi).l,I(angle-pi_lo).u))
    roads = [[I(x).l for x in rr] for rr in cfg['road_rectangles']]
    protect, release = 18*Q, 53*Q//2
    rad = {c: (F(1), F(1)) for c in cars}  # weights; multiply by rho below
    rho = F(5,10**6)
    released, rows, interfaces, summaries = set(), [], [], []
    sub = 4
    chord = I(15*(H/sub)**2/8).u
    run_begin = 0
    goals = {6,12,13,14}
    assert [legs[k-1]['car'] for k in (3,6,9,12)] == cfg['coordination_order']
    assert all(z[3:]==[0,0] for z in cfg['initial'].values())
    for ell, leg in enumerate(legs, 1):
        c = leg['car']
        before = ell > 1 and legs[ell-2]['car'] == c
        after = ell < len(legs) and legs[ell]['car'] == c
        if verify:
            rs = saved[ell]
            v,d = [[F(r[k+'0']) for r in rs]+[F(rs[-1][k+'1'])] for k in ('v','delta')]
            for j,r in enumerate(rs):
                assert r['active']==c and F(r['v1'])==v[j+1] and F(r['delta1'])==d[j+1]
        else:
            v, d = shoot(*profile(leg, 1.6 if before else 0, 1.6 if after else 0),
                         pose[c],leg['end'],sign[c])
        assert v[0] == (F(8,5) if before else 0) and d[0] == 0
        assert v[-1] == (F(8,5) if after else 0) and d[-1] == 0
        holder = c if c not in released else None
        begin = len(rows)
        others = {q: corners(pose[q],sign[q]) for q in cars if q != c}
        prev_corner = corners(pose[c],sign[c])
        for j in range(len(v)-1):
            v0,v1,d0,d1 = v[j],v[j+1],d[j],d[j+1]
            a,w = (v1-v0)/H,(d1-d0)/H
            rv = rho if v0 > 0 else F(0)
            rd = rv
            assert 0 <= v0-rv <= v0+rv <= 2 and 0 <= v1 <= 2
            assert max(abs(d0)+rd,abs(d1)) <= F(1,2)
            assert abs(a)+rv/H <= 1 and abs(w)+rd/H <= 1
            p0,t0 = rad[c]
            p1,t1 = increase(p0,t0,rv/rho,rd/rho)
            rad[c] = (p1,t1)
            err = {q: I(rho*(p+RB*t)).u for q,(p,t) in rad.items()}
            nodes = propagate(pose[c], v0,v1,d0,d1,sign[c],sub)
            if not after and j == len(v)-2 and ell in goals:
                gx,gy = leg['end']
                start_p = (nodes[0][0]-I(gx-F(sign[c],100))).abs_upper()+(nodes[0][1]-I(gy)).abs_upper()
                assert start_p+rho*p0 <= F(2,100), ('capture start',ell,float(start_p))
                assert nodes[0][2].abs_upper()+rho*t0 <= F(2,1000)
                assert F(5,100) <= v0-rv <= v0+rv <= F(15,100)
                assert abs(d0)+rd <= F(1,100)
            road_lb, permit_lb, released_lb = Q*100, Q*100, Q*100
            active_rear = Q*100
            gaps = {k: Q*100 for k in pairs}
            for ii, node in enumerate(nodes[1:]):
                cc = corners(node,sign[c])
                body = {**others,c: prev_corner+cc}
                boxes = {q: axis_box(cs) for q,cs in body.items()}
                budget = {q: err[q]+(chord if q == c else 0) for q in cars}
                active_rear = min(active_rear,rear(boxes[c],sign[c])-budget[c])
                axes = normed_axes(nodes[ii])
                for q in cars:
                    road_lb = min(road_lb, road(boxes[q],roads)-budget[q])
                    if q != holder:
                        z = boxes[q]
                        permit_lb = min(permit_lb,max(z[0]-protect,-protect-z[1])-budget[q])
                    if q in released:
                        released_lb = min(released_lb,rear(boxes[q],sign[q])-release-budget[q])
                for p,q in pairs:
                    gaps[p,q] = min(gaps[p,q],gap(body[p],body[q],boxes[p],boxes[q],
                                                   axes if c in (p,q) else [])-budget[p]-budget[q])
                prev_corner = cc
            assert road_lb > 0 and permit_lb > 0 and released_lb > 0 and min(gaps.values()) > Q//2, (
                ell,j,road_lb/Q,permit_lb/Q,released_lb/Q,min(gaps.values())/Q)
            pose[c] = nodes[-1]
            row = dict(tick=len(rows), leg=ell, active=c, holder=holder or '-', a=str(a), omega=str(w),
                       v0=str(v0), v1=str(v1), delta0=str(d0), delta1=str(d1),
                       road_m=road_lb/Q, protection_m=permit_lb/Q,
                       active_signed_rear_lower_Q=active_rear,
                       released_m=None if not released else released_lb/Q)
            for q in cars:
                pp,tt = (p0,t0) if q == c else rad[q]
                row.update({f'{q}_r_p':str(rho*pp), f'{q}_r_yaw':str(rho*tt),
                            f'{q}_r_v_delta':str(rv if q==c else 0),
                            f'{q}_x_center':nodes[0][0].mid() if q==c else pose[q][0].mid(),
                            f'{q}_y_center':nodes[0][1].mid() if q==c else pose[q][1].mid(),
                            f'{q}_local_yaw_center':nodes[0][2].mid() if q==c else pose[q][2].mid(),
                            f'{q}_pose_enclosure_l_u_Q':';'.join(f'{u.l}:{u.u}' for u in (nodes[0] if q==c else pose[q]))})
            row.update({f'gap_{p}_{q}_m':z/Q for (p,q),z in gaps.items()})
            row.update({f'gap_{p}_{q}_lower_Q':z for (p,q),z in gaps.items()})
            row.update(road_lower_Q=road_lb,protection_lower_Q=permit_lb,
                       released_lower_Q=released_lb if released else None)
            rows.append(row)
        cs = corners(pose[c],sign[c])
        p,t = rad[c]
        z = pose[c]
        e = rho*(p+RB*t)
        release_gap = F(rear(axis_box(cs),sign[c])-release,Q)-e
        gx,gy = leg['end']
        pos_err = (z[0]-I(gx)).abs_upper()+(z[1]-I(gy)).abs_upper()+rho*p
        yaw_err = z[2].abs_upper()+rho*t
        assert pos_err < F(2,100) and yaw_err < F(2,1000)
        exit_ticks = {}
        if not after:
            assert release_gap > 0 and v[-1] == d[-1] == 0
            if holder:
                for label,limit in [('clear_protection_tick',protect),('clear_release_plane_tick',release)]:
                    exit_ticks[label] = max((r['tick']+1 for r in rows[run_begin:]
                                            if r['active_signed_rear_lower_Q']<limit),default=run_begin)
                assert exit_ticks['clear_protection_tick'] < exit_ticks['clear_release_plane_tick'] <= len(rows)
            released.add(c)
            run_begin = len(rows)
        if ell in goals:
            assert list(leg['end']) == cfg['goals'][c][:2]
            assert pos_err <= cfg['terminal']['position_m'] and yaw_err <= cfg['terminal']['yaw_rad']
        interfaces.append(dict(leg=ell,tick=len(rows),active=c,next=legs[ell]['car'] if ell<len(legs) else 'hold',
                               v=str(v[-1]),delta=str(d[-1]), position_error_l1_plus_domain_m=float(pos_err),
                               yaw_error_plus_domain_rad=float(yaw_err),
                               release_margin_m=float(release_gap) if not after else None,**exit_ticks))
        rr = rows[begin:]
        summaries.append(dict(leg=ell,active=c,start=leg['start'][:2],end=leg['end'],ticks=len(rr),
                              road_m=min(r['road_m'] for r in rr),gap_m=min(
                                  r[f'gap_{p}_{q}_m'] for r in rr for p,q in pairs)))
        print(ell,c,len(rr),float(pos_err),float(yaw_err),summaries[-1]['gap_m'],flush=True)
    assert len(released)==4
    # Initial 32-tick witness and all adjacent box inclusions are checked explicitly.
    assert len(rows)>=cfg['N']
    last = {q: (rho*rad[q][0],rho*rad[q][1],F(0)) for q in cars}
    for j,r in enumerate(rows):
        c = r['active']
        nr = rows[j+1] if j+1<len(rows) else None
        for q in cars:
            p,t,w = (F(r[f'{q}_r_{key}']) for key in ('p','yaw','v_delta'))
            p1,t1 = increase(p,t,w,w) if q==c else (p,t)
            pp,tt,ww = tuple(F(nr[f'{q}_r_{key}']) for key in ('p','yaw','v_delta')) if nr else last[q]
            assert p1<=pp and t1<=tt and ww>=0
            if nr:
                vn = F(nr['v0']) if nr['active']==q else F(0)
                dn = F(nr['delta0']) if nr['active']==q else F(0)
                assert (F(r['v1']),F(r['delta1'])) == (vn,dn) if q==c else (vn,dn)==(0,0)
            else:
                assert F(r['v1'])==F(r['delta1'])==0
        if nr and nr['holder'] != r['holder']:
            assert r['leg'] in (3,6,9,12)
            assert interfaces[r['leg']-1]['release_margin_m'] > 0
            assert nr['holder'] in (nr['active'],'-')
    assert rows[0]['holder'] == cfg['coordination_order'][0]
    assert all(abs(z[0])-RB-rho*(1+RB)>18 for z in cfg['initial'].values())
    # Triangle inequality with 2 cm allowed at each endpoint, then rest-to-rest speed bound.
    length_lower = sum(F(math.isqrt(sum((b-a)**2 for a,b in zip(l['start'][:2],l['end']))*10**12),10**6)
                       -F(4,100) for l in legs)
    assert min(sum((b-a)**2 for a,b in zip(l['start'][:2],l['end'])) for l in legs) > 9**2
    min_time = length_lower/2+2*6
    assert min_time > cfg['timeout_s']
    output = dict(ticks=len(rows),duration_s=len(rows)/5,initial_witness_ticks=cfg['N'],
                  initial_pose_r_m_rad=float(rho),subintervals_per_tick=sub,legs=summaries,interfaces=interfaces,
                  final_radii_exact={q:list(map(str,v)) for q,v in last.items()},
                  final_pose_enclosures_Q={q:[[u.l,u.u] for u in zz] for q,zz in pose.items()},
                  same_waypoints_six_stop_runs_duration_lower_s=str(min_time),
                  exact_scale=Q)
    if verify:
        assert output == json.loads(summary_path.read_text(encoding='utf-8'))
        assert [{k:'' if v is None else str(v) for k,v in r.items()} for r in rows]==old
    else:
        with csv_path.open('w',encoding='utf-8-sig',newline='') as f:
            dw=csv.DictWriter(f,fieldnames=rows[0]); dw.writeheader(); dw.writerows(rows)
        summary_path.write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(f"PASS: {len(rows)} ticks, {len(legs)} legs, {len(pairs)} pairs per tick, {len(rows)/5} s")


if __name__ == '__main__':
    if not __debug__ or sys.argv[1:] not in ([],['--verify']):
        raise SystemExit('Usage: python route_zoh_certificate.py [--verify]; assertions must be enabled')
    main(bool(sys.argv[1:]))
