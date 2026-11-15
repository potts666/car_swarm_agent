"""Mode-indexed straight/circular path projection atlas with interval bounds."""
from common import *
ETA=F(1,50);MOTION=F(1,10**8)

def distance(v,a):
    v1=v+H*a
    if v*v1<0:return (v*v+v1*v1)/(2*abs(a))
    return H*(abs(v)+abs(v1))/2

def chart(w,c,k):
    route=get(HERE/'objects',read(HERE/'rational_inputs.json')['routes'][c]);r=k-w['starts'][c]
    if r<0:
        prep=read(HERE/'rational_inputs.json')['preparation'][c]
        return dict(kind='straight',mode='preparation' if k<len(prep) else 'wait',origin=[[m.I(INITIAL[c][0]).l]*2,[m.I(INITIAL[c][1]).l]*2,[0,0]],curvature='0',task_sign=1,car=c)
    if r>=route['ticks']:return dict(kind='hold',mode='goal_hold',car=c)
    label=route['labels'][r];start=r
    while start>0 and route['labels'][start-1]==label:start-=1
    d0,d1=map(F,route['knots'][r][2:]);v0,v1=map(F,route['knots'][r][:2]);task=-1 if 'staging' in label or 'reverse' in label else 1
    if v0==v1==0 or 'turn' in label or label=='waiting_feedback':return dict(kind='hold',mode=label,car=c)
    assert d0==d1,'missing varying-curvature path atlas'
    return dict(kind='arc' if d0 else 'straight',mode=label,origin=route['poses_Q'][start],curvature=str(d0),task_sign=task,car=c)

def atan_interval(x):
    radius=x.abs_upper();assert radius<F(9,10),'atlas angle ratio outside certified branch'
    total=m.I(0);power=x;square=x*x
    for j in range(160):total+=power*F((-1)**j,2*j+1);power=power*square
    error=radius**321/321;pad=m.I(error).u;return m.I.raw(total.l-pad,total.u+pad)

def coordinate(p,ch):
    if ch['kind']=='hold':return m.I(0)
    origin=[m.I.raw(*r) for r in ch['origin']];theta=origin[2];si,co=(m.I(0),m.I(1)) if theta.l==theta.u==0 else m.sincos(theta);sg=direction(ch['car']);dx=p[0]-origin[0];dy=p[1]-origin[1];t=sg*(dx*co+dy*si);n=sg*(-dx*si+dy*co)
    if ch['kind']=='straight':assert n.abs_upper()<=F(1,5),'straight projection outside tubular atlas';return ch['task_sign']*t
    kap=m.tan(m.I(F(ch['curvature'])))/m.L;den=m.I(1)-kap*n;assert den.l>0,'ambiguous circular projection';angle=atan_interval(kap*t/den);radius_residual=2*n-kap*(t*t+n*n)
    assert radius_residual.abs_upper()<F(1,2),'circular projection outside tubular atlas'
    sign=1 if F(ch['curvature'])>0 else -1
    return ch['task_sign']*sign*angle/(sign*kap)

def step_progress(state,commands,nodes,w):
    result={};k=state['cycle']
    for c in CARS:
        v=state['v'][c];a,omega=commands[c];ch=chart(w,c,k);travel=distance(v,a)
        if ch['kind']=='hold':lower=upper=F(0)
        elif state['poses'][c][2].l==state['poses'][c][2].u==0 and state['delta'][c]==omega==0 and ch['kind']=='straight':lower=upper=ch['task_sign']*(H*v+H*H*a/2)
        else:
            inc=coordinate(nodes[c][-1],ch)-coordinate(nodes[c][0],ch);lower=F(inc.l,m.Q);upper=F(inc.u,m.Q)
        done=k+1==w['goal_times'][c]
        result[c]=dict(progress_lower=str(lower),progress_upper=str(upper),travel=str(travel),motion=travel>MOTION,active=(lower>=ETA or done),completed_this_tick=done,chart_id=content_id(ch),mode=ch['mode'])
    return result

def atlas_identity(w):
    return content_id(dict(rule='mode_path_projection_v1',routes=read(HERE/'rational_inputs.json')['routes'],starts=w['starts'],ETA=ETA,tube_radius='1/5',atan_terms=160))

def add_rows(anchor,cert,rows):
    from derivatives import unpack
    contract=get(HERE/'objects',anchor['contract_id']);c=anchor['actor'];s=restore(m,anchor['state']);w=get(HERE/'objects',s['witness']);k=s['cycle']
    for phase,actors in contract['obligations'].items():
        if c not in actors or not k<=int(phase)<k+32:continue
        j=int(phase)-k;ch=chart(w,c,int(phase));assert ch['kind'] in ('straight','arc'),'missing unambiguous progress chart'
        source=cert['singles'][j]['source'];v=F(source['bounds']['v'][0]);a=F(anchor['plan']['remaining'][j][0]);sign=ch['task_sign'];d0=F(source['bounds']['delta'][0]);d1=d0+H*F(anchor['plan']['remaining'][j][1]);frozen=F(anchor['contracts'][j]['delta'][0])==F(anchor['contracts'][j]['delta'][1])==0 and F(anchor['contracts'][j]['omega'][0])==F(anchor['contracts'][j]['omega'][1])==0
        linear=[F(0)]*64
        if frozen and source['bounds']['pose_Q'][2]==[0,0]:
            for t in range(j):linear[2*t]=-sign*H*H
            linear[2*j]=-sign*H*H/2;g=F(actors[c])-sign*(H*v+H*H*a/2);err=hh=F(0);rule='exact_affine_signed_straight'
        else:
            # Chain the path projection with the already certified input-to-position map.
            end=cert['singles'][j+1]['source']['bounds']['pose_Q'] if j<31 else cert['terminal_state']['poses_Q'][c]
            begin=source['bounds']['pose_Q'];p0=tuple(m.I.raw(*x) for x in begin);p1=tuple(m.I.raw(*x) for x in end)
            bound=projection_bounds(ch);grad1=projection_gradient(p1,ch);grad0=projection_gradient(p0,ch)
            J1=unpack(m,cert['prefix_jacobians'][j]);J0=unpack(m,cert['prefix_jacobians'][j-1]) if j else None
            intervals=[]
            for t in range(64):
                value=sum((grad1[r]*J1[r][t] for r in range(2)),m.I(0)) if t<2*(j+1) else m.I(0)
                if j and t<2*j:value-=sum((grad0[r]*J0[r][t] for r in range(2)),m.I(0))
                intervals.append(-value)
            linear=[F(x.l+x.u,2*m.Q) for x in intervals];err=norm_upper([F(x.u-x.l,2*m.Q) for x in intervals]);inc=coordinate(p1,ch)-coordinate(p0,ch);g=F(actors[c])-F(inc.l,m.Q)
            pe=cert['local_profiles'][j];ps=cert['local_profiles'][j-1] if j else dict(Gp='0',Hp='0');hh=F(bound['gradient_upper'])*3*(F(pe['Hp'])+F(ps['Hp']))+F(bound['hessian_upper'])*9*(F(pe['Gp'])**2+F(ps['Gp'])**2);rule='certified_path_projection_Taylor'
        rows.append(dict(name=f'progress_{phase}_{c}',g_upper=g,linear=linear,gradient_error=err,hessian=hh,kind='progress',rule=rule,atlas_id=contract['atlas_id']))
    return rows


def projection_bounds(ch):
    if ch['kind']!='arc':return dict(gradient_upper='1',hessian_upper='0',tubular_radius='1/5')
    k=m.tan(m.I(F(ch['curvature'])))/m.L;K=k.abs_upper();e=F(1,5);den=1-K*e;assert den>0
    return normal(dict(gradient_upper=2/den,hessian_upper=8*K/(den*den),tubular_radius=e,curvature_abs=K,unique_projection_radius_lower=1/K))

def projection_gradient(p,ch):
    origin=[m.I.raw(*x) for x in ch['origin']];si,co=(m.I(0),m.I(1)) if origin[2].l==origin[2].u==0 else m.sincos(origin[2]);sg=direction(ch['car']);tangent=[sg*co,sg*si];normal=[-sg*si,sg*co]
    if ch['kind']=='straight':return [ch['task_sign']*x for x in tangent]
    dx=[p[r]-origin[r] for r in (0,1)];t=sum((x*y for x,y in zip(dx,tangent)),m.I(0));n=sum((x*y for x,y in zip(dx,normal)),m.I(0));k=m.tan(m.I(F(ch['curvature'])))/m.L;den=m.I(1)-k*n;D=den*den+(k*t)*(k*t);assert D.l>0
    return [ch['task_sign']*(den*tangent[r]+k*t*normal[r])/D for r in (0,1)]
