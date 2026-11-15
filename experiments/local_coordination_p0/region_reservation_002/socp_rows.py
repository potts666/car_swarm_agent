from common import *
from derivatives import unpack

def rows_for(data,m,anchor,cert,method):
    assert method=='B-JL' and cert['anchor_id']==content_id(anchor)
    u=[tuple(map(F,x)) for x in anchor['plan']['remaining'][:32]];c=anchor['actor'];state=restore(m,anchor['state'])
    v,d=state['v'][c],state['delta'][c];rows=[]
    def add(name,g,linear=None,error=0,hessian=0,kind='physical'):
        rows.append(dict(name=name,g_upper=F(g),linear=[F(0)]*64 if linear is None else list(map(F,linear)),gradient_error=F(error),hessian=F(hessian),kind=kind))
    for j,(a,w) in enumerate(u):
        for component,value in enumerate((a,w)):
            for sign in (-1,1):
                gradient=[F(0)]*64;gradient[2*j+component]=F(sign)
                add(f'input_{j}_{component}_{sign}',sign*value-1,gradient)
        v+=H*a;d+=H*w
        for component,value,lower,upper in ((0,v,F(0),F(2)),(1,d,F(-1,2),F(1,2))):
            for sign,bound in ((1,upper),(-1,-lower)):
                gradient=[F(0)]*64
                for k in range(j+1):gradient[2*k+component]=sign*H
                add(f'state_{j}_{component}_{sign}',sign*value-bound,gradient)
    end=cert['terminal_state'];r=data.row(anchor['receiver_phase'])['domain'];pose=[m.I.raw(*p) for p in end['poses_Q'][c]];center=[F(*p) for p in r['center_pose_Q']]
    G={k:F(x) for k,x in cert['local_profiles'][-1].items()}
    matrix=unpack(m,cert['prefix_jacobians'][-1])
    terminal=[]
    for sx in (-1,1):
        for sy in (-1,1):terminal.append((f'position_{sx}_{sy}',(sx*(pose[0]-center[0])+sy*(pose[1]-center[1])).u,F(r['position_l1_radius']),[sx*matrix[0][j]+sy*matrix[1][j] for j in range(64)],F(3,2)*G['Gp'],F(3,2)*G['Hp']))
    for sign in (-1,1):terminal.append((f'yaw_{sign}',(sign*(pose[2]-center[2])).u,F(r['yaw_radius']),[sign*x for x in matrix[2]],G['P1'],G['P2']))
    for name,upper,radius,J,gradient_bound,hessian in terminal:
        if method=='B-JL':
            mid=[F(x.l+x.u,2*m.Q) for x in J];epsilon=norm_upper([F(x.u-x.l,2*m.Q) for x in J])
        else:mid=[F(0)]*64;epsilon=gradient_bound
        add(name,F(upper,m.Q)-radius,mid,epsilon,hessian,'terminal')
    for component,key,radius_name,refname in ((0,'v','speed_radius','center_v'),(1,'delta','steering_radius','center_delta')):
        value=F(end[key][c])-F(r[refname]);radius=F(r[radius_name])
        for sign in (-1,1):
            gradient=[F(0)]*64
            for j in range(32):gradient[2*j+component]=sign*H
            add(f'terminal_{key}_{sign}',sign*value-radius,gradient)
    for j,(profile,margin) in enumerate(zip(cert['local_profiles'],cert['geometry_margins'])):
        add(f'geometry_{j+1}',-F(margin),error=F(profile['Gc']),hessian=F(profile['Hc']),kind='geometry')
    add('hard_trust',-RHO,error=1,kind='trust')
    return rows


def evaluate(rows,delta):
    delta=list(map(F,delta));t=norm_upper(delta);s=t*t;checks=[]
    for row in rows:
        constant=F(row['g_upper']);linear=sum(F(a)*b for a,b in zip(row['linear'],delta));error=F(row['gradient_error'])*t;second=F(row['hessian'])*s/2
        checks.append(dict(name=row['name'],kind=row['kind'],constant=constant,linear=linear,gradient_uncertainty=error,second_order=second,slack=-(constant+linear+error+second)))
    return dict(passed=all(x['slack']>=0 for x in checks),norm_upper=t,rows=checks,violations=[x['name'] for x in checks if x['slack']<0])
