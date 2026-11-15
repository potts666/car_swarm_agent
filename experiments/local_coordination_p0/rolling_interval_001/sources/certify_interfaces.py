"""Model-based receiving-domain and derivative certificates; no controller run."""
import argparse
import csv
from fractions import Fraction as F
from functools import lru_cache
import gzip
import hashlib
import importlib.util
import itertools
import json
import math
from pathlib import Path
import sys

H, L, RB, C = F(1, 5), F(27, 10), F(1807, 500), F(3, 2)
CARS = ('A1', 'A2', 'B1', 'B2')
HERE = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text(encoding='utf8'))


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, default=str), encoding='utf8')


def up(x, grid=10**10):
    x = F(x)
    return F(-(-x.numerator*grid//x.denominator), grid)


@lru_cache(None)
def trig_bounds(d):
    assert 0 <= d <= F(1, 2)
    tan = (d-d**3/6+d**5/120)/(1-d*d/2)
    return up(tan/L), up((1+tan*tan)/L), up(2*tan*(1+tan*tan)/L)


@lru_cache(None)
def derivative_bounds_one(vmax, dmax):
    k, kd, kdd = trig_bounds(dmax)
    c, d = k+vmax*kd, 2*kd+vmax*kdd
    gpsi = 1+c*(H+H*H/2)
    hpsi = d*(H+H*H+H**3/3)
    gp = 1+H+H*H/2+vmax*(H+c*(H*H/2+H**3/6))
    # Positive polynomial coefficients of 2*(1+t)*P1+vmax*(P2+P1**2).
    p1, p2 = [F(1), c, c/2], [F(0), d, d, d/3]
    coeff = [F(0)]*5
    for i, x in enumerate([F(1), F(1)]):
        for j, y in enumerate(p1): coeff[i+j] += 2*x*y
    for i, x in enumerate(p2): coeff[i] += vmax*x
    for i, x in enumerate(p1):
        for j, y in enumerate(p1): coeff[i+j] += vmax*x*y
    hp = sum(x*H**(i+1)/(i+1) for i, x in enumerate(coeff))
    mc = 1+vmax*vmax*k+RB*(k+vmax*kd+vmax*vmax*k*k)
    return dict(G_position=up(gp), G_yaw=up(gpsi), H_position=up(hp),
                H_yaw=up(hpsi), G_corner=up(gp+RB*gpsi),
                H_corner=up(hp+RB*(hpsi+gpsi*gpsi)), Mc=up(mc),
                v_upper=vmax, delta_abs_upper=dmax)


def derivative_bounds_horizon(knots, cut, initial_rv=F(0), initial_rd=F(0)):
    # Euclidean norm on 64 input coordinates; h*ceil(sqrt(32)) bounds both v/delta gradients.
    beta, trust = F(6, 5), F(1, 1000)
    g = t = gp = hp = F(0)
    for j in range(cut, cut+32):
        row = knots[j] if j < len(knots) else (F(0),)*4
        v = min(F(2), up(max(row[:2])+initial_rv+beta*trust, 1000))
        d = min(F(1, 2), up(max(map(abs, row[2:]))+initial_rd+beta*trust, 1000))
        k, kd, kdd = trig_bounds(d)
        c, z = k+v*kd, 2*kd+v*kdd
        inc = g*H+beta*c*H*H/2
        hp = up(hp+2*beta*inc+v*(t*H+beta*beta*z*H*H/2
                       +g*g*H+g*beta*c*H*H+beta*beta*c*c*H**3/3))
        gp = up(gp+beta*H+v*inc)
        g = up(g+beta*c*H)
        t = up(t+beta*beta*z*H)
    return dict(G_position=gp, G_yaw=g, H_position=hp, H_yaw=t,
                G_corner=up(gp+RB*g), H_corner=up(hp+RB*(t+g*g)),
                input_l2_trust_radius=trust, horizon_ticks=32,
                initial_speed_radius=initial_rv,initial_steering_radius=initial_rd,
                physical_affine_rows_required=True)


def domain(cert, k):
    """Box about rational midpoint, plus inherited exact-flow component."""
    Q = cert['scale']; poses = cert['pose_nodes_Q']; n = cert['ticks']
    if k in (0, n):
        return dict(kind='coarse_stopped_domain', pose_radius=cert['source_radius' if k == 0 else 'receiver_radius'],
                    velocity_error='0', steering_error='0')
    v0, v1, d0, d1 = map(F, cert['knots'][k])
    a, w = (v1-v0)/H, (d1-d0)/H
    P, T = map(F, cert['source_radius'])
    nominal = poses[k]
    erp = sum(F(hi-lo, 2*Q) for lo, hi in nominal[:2])
    ert = F(nominal[2][1]-nominal[2][0], 2*Q)
    origin = list(map(F, cert['waypoints'][0]))
    distance = sum(max(abs(F(lo, Q)-origin[j]), abs(F(hi, Q)-origin[j]))
                   for j, (lo, hi) in enumerate(nominal[:2]))
    distance += C*H*(v0+v1)/2
    vcap = min(v0, 2-v0, H*(1-abs(a)))/2
    dcap = min(F(1,2)-abs(d0), H*(1-abs(w)))/2
    assert vcap > 0 and dcap > 0, 'moving interface has an active physical limit'
    dmax = max(abs(d0)+dcap, abs(d1))
    kap, kd, _ = trig_bounds(dmax)
    vr = max(v0, v1)
    qv, qd = kap, vr*kd
    At = C*vr*H+C*distance
    Av = C*H/2+C*vr*H*H*qv/3+C*distance*H*qv/2
    Ad = C*vr*H*H*qd/3+C*distance*H*qd/2
    Bv, Bd = H*qv/2, H*qd/2
    prem, trem = P-erp-At*ert, T-ert
    assert min(prem, trem, At, Av, Ad, Bv, Bd) > 0
    rp = prem/4
    rt = min(prem/(4*At), trem/3)
    rv = min(prem/(4*Av), trem/(3*Bv), vcap)
    rd = min(prem/(4*Ad), trem/(3*Bd), dcap)
    assert min(rp, rt, rv, rd) > 0
    q = qv*rv+qd*rd
    theta = rt+ert+H*q/2
    pos = rp+erp+C*H*rv/2+C*vr*H*(rt+ert)+C*vr*H*H*q/3
    pslack, tslack = P-pos-C*distance*theta, T-theta
    assert min(pslack, tslack) >= 0
    assert abs(a)+rv/H <= 1 and abs(w)+rd/H <= 1
    assert 0 <= v0-rv <= v0+rv <= 2
    assert max(abs(d0-rd), abs(d0+rd), abs(d1)) <= F(1,2)
    return dict(kind='flow_union_five_state_reset_box',
        position_l1_radius=rp, yaw_radius=rt, speed_radius=rv, steering_radius=rd,
        center_pose_Q=[[(lo+hi), 2*Q] for lo, hi in nominal],
        center_v=v0, center_delta=d0, feedback_a_ref=a, feedback_omega_ref=w,
        feedback_rule='a=a_ref-(v-v_ref)/h; omega=omega_ref-(delta-delta_ref)/h',
        nominal_position_uncertainty=erp, nominal_yaw_uncertainty=ert,
        displacement_l1_upper=distance, kappa_upper=kap, kappa_delta_upper=kd,
        reset_pose_source_slack=pslack, reset_yaw_source_slack=tslack,
        next_component='inherited_exact_flow', next_phase=k+1)


def interval_jacobian(m, cert, k, box, subdivisions=8):
    """Range integration of variational ODE; interval rectangles enclose integrals."""
    Q, I = cert['scale'], m.I
    zero, one = I(0), I(1)
    v0, v1, d0, d1 = map(F, cert['knots'][k])
    a, w = (v1-v0)/H, (d1-d0)/H
    rv = F(box.get('speed_radius', 0)); rd = F(box.get('steering_radius', 0))
    rt = F(box.get('yaw_radius', box.get('pose_radius', [0,0])[1]))
    def radius(x, r): return I.raw(I(x-r).l, I(x+r).u)
    vi, di, ai, wi = radius(v0,rv), radius(d0,rd), radius(a,rv/H), radius(w,rd/H)
    lo,hi = cert['pose_nodes_Q'][k][2]
    theta = I.raw(lo-I(rt).u, hi+I(rt).u)
    yaw = [zero for _ in range(7)];yaw[2] = one
    px = [zero for _ in range(7)]; px[0] = one
    py = [zero for _ in range(7)]; py[1] = one
    sign = cert['direction_sign']
    dt = H/subdivisions; short = I.raw(0, I(dt).u)
    for j in range(subdivisions):
        ti = I.raw(I(j*dt).l, I((j+1)*dt).u)
        t0,t1=j*dt,(j+1)*dt
        moment1=(t1*t1-t0*t0)/2
        moment2=(t1**3-t0**3)/3
        v, d = vi+ai*ti, di+wi*ti
        tan = m.tan(d); kd = (one+tan*tan)/L
        rate = v*tan/L
        yrange = theta+short*rate
        si,co = m.sincos(yrange)
        dv = [zero,zero,zero,one,zero,ti,zero]
        dd = [zero,zero,zero,zero,one,zero,ti]
        dyaw = [tan/L*dv[t]+v*kd*dd[t] for t in range(7)]
        yaw_range = [yaw[t]+short*dyaw[t] for t in range(7)]
        dv_integral=[zero,zero,zero,I(dt),zero,I(moment1),zero]
        px = [px[t]+sign*(co*dv_integral[t]-vi*si*yaw_range[t]*dt
                         -ai*si*yaw_range[t]*moment1) for t in range(7)]
        py = [py[t]+sign*(si*dv_integral[t]+vi*co*yaw_range[t]*dt
                         +ai*co*yaw_range[t]*moment1) for t in range(7)]
        yaw[3] += tan/L*dt
        yaw[5] += tan/L*moment1
        yaw[4] += kd*vi*dt+kd*ai*moment1
        yaw[6] += kd*vi*moment1+kd*ai*moment2
        theta += tan/L*vi*dt+tan/L*ai*moment1
    velocity = [zero,zero,zero,one,zero,I(H),zero]
    steering = [zero,zero,zero,zero,one,zero,I(H)]
    rows = [px,py,yaw,velocity,steering]
    scale = 10**15
    packed = []
    for r,row in enumerate(rows):
        for col,x in enumerate(row):
            low = x.l*scale//Q
            high = -(-x.u*scale//Q)
            if low or high: packed.append([r,col,low,high])
    return dict(shape=[5,7], columns=['px','py','psi','v','delta','a','omega'],
                scale=scale, nonzero_intervals=packed, subintervals=subdivisions,
                method='interval ranges with exact polynomial time moments',
                initial_and_feedback_input_rectangle=True,initial_yaw_radius=rt)


def budgets(graph, motions):
    outgoing = {n:[] for n in graph['goal_levels']}
    for i,e in enumerate(graph['winning_edges']): outgoing[e['source']].append((i,e))
    Cc,R,policy = {graph['goal']:0},{graph['goal']:0},{}
    def visit(node):
        if node in Cc:return
        for _,e in outgoing[node]: visit(e['target'])
        values = [(motions[e['motion']]['ticks']+Cc[e['target']],i) for i,e in outgoing[node]]
        Cc[node],policy[node] = min(values)
        R[node] = max(motions[e['motion']]['ticks']+R[e['target']] for _,e in outgoing[node])
    for node in outgoing: visit(node)
    return Cc,R,policy,outgoing


def station(cfg, car, stage):
    if stage==0:return tuple(map(F,cfg['initial'][car][:2]))
    if stage==3:return tuple(map(F,cfg['goals'][car][:2]))
    x = F(cfg['release_plane_abs_x'])+F(cfg['release_margin_m'])+F(cfg['vehicle']['rear'])+F(cfg['s_min'])
    ys = sorted({F(z[1]) for z in cfg['goals'].values()}, reverse=True)
    return (x if car[0]=='A' else -x), ys[stage-1]


def static_node(m,cfg,node,radii):
    footprints,bb,errors = {},{},{}
    roads = [[m.I(F(x)).l for x in row] for row in cfg['road_rectangles']]
    release = F(cfg['release_plane_abs_x'])+F(cfg['release_margin_m'])
    road, protection, released = {},{},{}
    for i,c in enumerate(CARS):
        stage=int(node[i]);p,t=map(F,radii[stage]);x,y=station(cfg,c,stage)
        footprints[c]=m.corners((m.I(x),m.I(y),m.I(0)),1 if c[0]=='A' else -1)
        bb[c]=m.axis_box(footprints[c]);errors[c]=m.I(p+RB*t).u
        road[c]=F(m.road(bb[c],roads)-errors[c],m.Q)
        protection[c]=F(max(bb[c][0]-18*m.Q,-18*m.Q-bb[c][1])-errors[c],m.Q)
        if stage:released[c]=F(m.rear(bb[c],1 if c[0]=='A' else -1)-errors[c],m.Q)-release
    pairs = {a+'-'+b:F(m.gap(footprints[a],footprints[b],bb[a],bb[b],[])-errors[a]-errors[b],m.Q)
             for a,b in itertools.combinations(CARS,2)}
    assert min(road.values())>0 and min(protection.values())>0
    assert not released or min(released.values())>=0
    assert min(pairs.values())>=F(1,2)
    return dict(road_lower=road,pair_lower=pairs,protection_lower=protection,
                released_lower=released,all_v_delta_zero=True)


def analytic_self_checks(m):
    k,kd,kdd=trig_bounds(F(1,2))
    # Fourth derivatives of yaw rate and Cartesian velocity for archived Simpson error constants.
    tan,sec=F(11,20),F(64,49)
    f2=2*tan*sec;f3=2*sec*(1+3*tan*tan);f4=8*tan*sec*(2+3*tan*tan)
    rate4=(4*f3+2*f4)/L
    p1=2*tan/L;p2=(tan+2*sec)/L;p3=(2*sec+2*f2)/L;p4=(3*f2+2*f3)/L
    velocity4=2*(p1**4+6*p1*p1*p2+3*p2*p2+4*p1*p3+p4)+4*(p1**3+3*p1*p2+p3)
    assert rate4<84 and velocity4<256
    assert 1+4*k+RB*(k+2*kd+4*k*k)<7
    # Interval variational inclusion checked against the exact straight-reference Jacobian.
    cert=dict(scale=m.Q,car='A1',knots=[['1/2','1/2','0','0']],pose_nodes_Q=[[[0,0],[0,0],[0,0]]])
    result=interval_jacobian(m,cert,0,{})
    exact=[[F(0) for _ in range(7)] for _ in range(5)]
    exact[0][0]=exact[1][1]=exact[2][2]=exact[3][3]=exact[4][4]=F(1)
    exact[0][3]=H; exact[0][5]=H*H/2
    exact[1][2]=H/2;exact[1][4]=H*H/(8*L);exact[1][6]=H**3/(24*L)
    exact[2][4]=H/(2*L);exact[2][6]=H*H/(4*L)
    exact[3][5]=exact[4][6]=H
    entries={(r,c):(F(lo,result['scale']),F(hi,result['scale'])) for r,c,lo,hi in result['nonzero_intervals']}
    for r in range(5):
        for c in range(7):
            lo,hi=entries.get((r,c),(F(0),F(0)))
            assert lo<=exact[r][c]<=hi,(r,c,lo,exact[r][c],hi)
    return dict(yaw_rate_fourth_upper=str(rate4),archived_bound_84_valid=True,
                cartesian_velocity_fourth_upper=str(velocity4),archived_bound_256_valid=True,
                straight_Jacobian_exact_inclusion=True,Mc_7_valid=True)


def recheck_geometry(source,cfg,motions):
    graph_path=source/'results/source_rebuild_005/work/NTU_DMPC_Dissertation/certificate_graph.py'
    spec=importlib.util.spec_from_file_location('current_map_geometry',graph_path)
    graph=importlib.util.module_from_spec(spec);spec.loader.exec_module(graph)
    graph.CFG=cfg
    graph.STAGE_RADII=[tuple(map(F,row)) for row in next(iter(motions.values()))['stage_radii']]
    graph.ROADS=[[graph.model.I(F(x)).l for x in row] for row in cfg['road_rectangles']]
    graph.RELEASE=F(cfg['release_plane_abs_x'])+F(cfg['release_margin_m'])
    graph.STAGING=graph.RELEASE+F(cfg['vehicle']['rear'])+F(cfg['s_min'])
    graph.YBAYS=sorted({F(z[1]) for z in cfg['goals'].values()},reverse=True)
    spec=importlib.util.spec_from_file_location('current_map_cross_stop',source/'cross_stop.py')
    verifier=importlib.util.module_from_spec(spec);spec.loader.exec_module(verifier)
    for key,cert in sorted(motions.items()):
        checked=verifier.verify(graph,cert['car'],cert['source'],cert['target'],
                                [tuple(map(F,row)) for row in cert['knots']])
        for name,value in checked.items():assert value==cert[name],(key,name)
        print('geometry rechecked',key,flush=True)
    return [graph_path,source/'cross_stop.py',graph.model.CFG]


def run(source, output):
    data=source/'results/pilot_objects_011'; case=data/'asymmetric_baseline'
    model_path=source/'results/source_rebuild_005/work/M2b_首段自由度与递归可行性/route_zoh_certificate.py'
    spec=importlib.util.spec_from_file_location('cert_interval',model_path)
    model=importlib.util.module_from_spec(spec);spec.loader.exec_module(model)
    assert model.Q==10**20 and model.H==H and model.L==L
    checks=analytic_self_checks(model)
    cfg,graph,mapping=read(case/'config.json'),read(case/'M_graph.json'),read(case/'M_motions.json')
    assert F(cfg['vehicle']['L']) == L and F(cfg['h']) == H
    assert F(cfg['vehicle']['L'])+F(cfg['vehicle']['front']) == F(7,2)
    assert F(cfg['vehicle']['rear']) == 1 and F(cfg['vehicle']['width']) == F(9,5)
    motions={key:read(data/'objects'/(digest+'.json')) for key,digest in mapping.items()}
    inputs={str(p.relative_to(source)).replace('\\','/'):sha(p) for p in
            [model_path,case/'config.json',case/'M_graph.json',case/'M_motions.json',case/'coordination.json']}
    for key,digest in mapping.items():
        path=data/'objects'/(digest+'.json'); assert sha(path)==digest
        inputs[str(path.relative_to(source)).replace('\\','/')]=digest
        assert motions[key]['ticks']==len(motions[key]['knots'])==len(motions[key]['inputs'])
    assert len(motions)==20 and len(graph['goal_levels'])==141 and len(graph['winning_edges'])==402
    dependencies=recheck_geometry(source,cfg,motions)
    for path in dependencies:inputs[str(path.relative_to(source)).replace('\\','/')]=sha(path)
    Cc,R,policy,outgoing=budgets(graph,motions)
    assert Cc[graph['initial']]==read(case/'coordination.json')['C_M'][graph['initial']]
    radii=next(iter(motions.values()))['stage_radii']
    nodes={node:{**static_node(model,cfg,node,radii),'C':Cc[node],'R':R[node],
                 'default_C_edge':policy.get(node)} for node in Cc}
    edges=[];incoming={n:[] for n in nodes}
    for i,e in enumerate(graph['winning_edges']):
        cert=motions[e['motion']];Q=cert['scale']
        assert F(cert['endpoint_position_upper'])<=F(cert['receiver_radius'][0])
        assert F(cert['endpoint_heading_upper'])<=F(cert['receiver_radius'][1])
        assert list(map(F,cert['knots'][-1]))[1::2]==[F(0),F(0)]
        peer={c:F(cert['peer_gap_lower_Q'][c+':'+e['source'][j]],Q)
              for j,c in enumerate(CARS) if c!=e['actor']}
        assert min(peer.values())>=F(1,2) and cert['road_lower_Q']>0
        if cert['source']>0:assert cert['released_lower_Q']>=0
        assert cert['ticks']+R[e['target']]<=R[e['source']]
        edges.append(dict(id=i,**e,ticks=cert['ticks'],road_lower=F(cert['road_lower_Q'],Q),
            pair_lower=peer,required_C_budget=cert['ticks']+Cc[e['target']],
            required_R_budget=cert['ticks']+R[e['target']],
            C_optimal_edge=cert['ticks']+Cc[e['target']]==Cc[e['source']],
            coarse_receiver_contains_entire_source=True))
        incoming[e['target']].append(i)
    switches=[]
    for node in nodes:
        pairs=[(i,j) for i in incoming[node] for j,_ in outgoing[node]
               if edges[i]['actor']!=edges[j]['actor']]
        if pairs:switches.append(dict(node=node,incoming_outgoing_edge_pairs=pairs,
            receiver_all_v_delta_zero=True,protection_lower=nodes[node]['protection_lower'],
            following_required_budget='see edge required_C_budget/required_R_budget',
            R_all_outgoing_admissible=True))
    output.mkdir(parents=True,exist_ok=False)
    write(output/'input_manifest.json',inputs);write(output/'joint_nodes.json',nodes)
    write(output/'edge_certificates.json',edges);write(output/'permission_handoffs.json',switches)
    records=positive=total=0;landmarks=[];templates=[];minima={}
    compressed=output/'interfaces.jsonl.gz'
    with compressed.open('wb') as raw,gzip.GzipFile(fileobj=raw,mode='wb',mtime=0) as zipped:
        for key,cert in sorted(motions.items()):
            knots=[tuple(map(F,row)) for row in cert['knots']];n=cert['ticks']
            selected={0,n}; labels={0:['source'],n:['coarse_receiver']}
            last=0
            for join in cert['joins']:
                cut=join['cut'];selected.add(cut);labels.setdefault(cut,[]).append('physical_join')
                indices=range(last,cut)
                for phase in {max(indices,key=lambda j:knots[j][2]),min(indices,key=lambda j:knots[j][2])}:
                    selected.add(phase);labels.setdefault(phase,[]).append('curve_extremum')
                last=cut
            release_phase=None
            for k,pose in enumerate(cert['pose_nodes_Q']):
                origin=list(map(F,cert['waypoints'][0]));Q=cert['scale']
                z=tuple(model.I.raw(lo,hi) for lo,hi in pose)
                bb=model.axis_box(model.corners(z,1 if cert['car'][0]=='A' else -1))
                D=sum(max(abs(F(lo,Q)-origin[j]),abs(F(hi,Q)-origin[j])) for j,(lo,hi) in enumerate(pose[:2]))
                P,T=map(F,cert['source_radius'])
                rear=F(model.rear(bb,1 if cert['car'][0]=='A' else -1),Q)-(P+T*(D+RB))
                release=F(cfg['release_plane_abs_x'])+F(cfg['release_margin_m'])
                if rear>=release:
                    release_phase=k;selected.add(k);labels.setdefault(k,[]).append('full_source_exit_release');break
            for k in range(n+1):
                box=domain(cert,k);isbox=box['kind'].startswith('flow_union')
                if isbox:
                    positive+=1
                    for name in ('position_l1_radius','yaw_radius','speed_radius','steering_radius'):
                        value=box[name];minima[name]=min(minima.get(name,value),value)
                row=knots[k] if k<n else (F(0),)*4
                bounds1=derivative_bounds_one(F(2),F(1,2))
                record=dict(motion=key,object_sha256=mapping[key],phase=k,
                    nominal_pose_Q=cert['pose_nodes_Q'][k],domain=box,
                    nominal_v=row[0],nominal_delta=row[2],remaining_template_ticks=n-k,
                    continuation='first-step reset then inherited suffix' if isbox else 'certified stopped resource continuation',
                    bounds_one_step_7parameters=bounds1,
                    bounds_32step_64inputs=derivative_bounds_horizon(knots,k,
                        F(box.get('speed_radius',0)),F(box.get('steering_radius',0))),tags=labels.get(k,[]))
                if k<n:
                    record['jacobian_one_step']=interval_jacobian(model,cert,k,box)
                    total+=1
                zipped.write((json.dumps(record,default=str,separators=(',',':'))+'\n').encode('utf8'))
                records+=1
                if k in selected:
                    landmarks.append(dict(motion=key,phase=k,tags=';'.join(labels[k]),
                        x=float(F(sum(cert['pose_nodes_Q'][k][0]),2*cert['scale'])),
                        y=float(F(sum(cert['pose_nodes_Q'][k][1]),2*cert['scale'])),
                        v=float(row[0]),delta=float(row[2]),
                        rp=float(box['position_l1_radius'] if isbox else F(box['pose_radius'][0])),
                        rpsi=float(box['yaw_radius'] if isbox else F(box['pose_radius'][1])),
                        rv=float(box.get('speed_radius',0)),rdelta=float(box.get('steering_radius',0)),
                        G32=float(record['bounds_32step_64inputs']['G_corner']),
                        H32=float(record['bounds_32step_64inputs']['H_corner'])))
            templates.append(dict(motion=key,ticks=n,source=cert['source'],target=cert['target'],
                waypoints=cert['waypoints'],joins=cert['joins'],
                first_full_source_release_phase=release_phase,
                release_to_coarse_stop_ticks=None if release_phase is None else n-release_phase,
                source_radius=cert['source_radius'],receiver_radius=cert['receiver_radius']))
            print('certified',key,n,'ticks',flush=True)
    with (output/'landmarks.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(landmarks[0]));writer.writeheader();writer.writerows(landmarks)
    write(output/'templates.json',templates)
    report=dict(status='PASS',map_case='asymmetric_baseline',templates=len(motions),
        joint_nodes=len(nodes),joint_edges=len(edges),interface_records=records,
        one_step_jacobians=total,positive_five_state_boxes=positive,
        coarse_endpoint_records=records-positive,
        permission_switch_nodes=len(switches),
        permission_switch_edge_pairs=sum(len(x['incoming_outgoing_edge_pairs']) for x in switches),
        initial_C=Cc[graph['initial']],initial_R=R[graph['initial']],
        minimum_box_radii={k:dict(exact=str(v),decimal=float(v)) for k,v in minima.items()},
        derivative_tests=checks,input_files_verified=len(inputs),
        inherited_source_geometry='20 full-source interval motion certificates freshly recomputed field by field; all joint node static geometry recomputed',
        new_certificate='full 5-state midpoint boxes, one-step feedback reset into exact-flow source image; union includes inherited component',
        scope='model and interval certificates; no optimization run, no new shorter-duration template, no controller experiment')
    write(output/'checks.json',report)
    print(json.dumps(report,indent=2))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=HERE.parents[1]/'reStart/local_coordination_p0')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    run(args.source.resolve(),args.output.resolve())


if __name__=='__main__':main()
