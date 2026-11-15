"""Rational commands; exact constant-curvature integration against analytic intervals."""
from common import *
from physics import propagate,trig
ALPHA_LO,ALPHA_HI=F('0.722734247813'),F('0.722734247814')
DELTA_LO,DELTA_HI=F('0.422853926132'),F('0.422853926134')
ALPHA=(ALPHA_LO+ALPHA_HI)/2;DELTA=(DELTA_LO+DELTA_HI)/2
SQ=m.I.raw(math.isqrt(7*m.Q*m.Q),math.isqrt(7*m.Q*m.Q)+1);DS=3*SQ
D=F(round(F(DS.l+DS.u,2*m.Q)*10**12),10**12)

def trapezoid(length):
    M=5;n=M+max(M,math.ceil(abs(length)/H));V=F(length)/(H*(n-M));v=F(0);u=[]
    for j in range(n):
        a=V/(M*H) if j<M else -V/(M*H) if j>=n-M else F(0);u.append((a,F(0)));v+=H*a
    assert v==0 and max(abs(a) for a,w in u)<=1
    assert H*sum(F(n-1-j)*H*a+H*a/2 for j,(a,w) in enumerate(u))==length
    return u

def reference(car,name,progress):
    s=direction(car);yl=s*WAIT[car][1];pre=m.I(-23)-DS;endpre=m.I(46)-DS
    if name in ('staging','approach','crossing','exit'):
        xx0,xx1,yy={'staging':(m.I(-30),pre,m.I(yl)),'approach':(m.I(-23),m.I(-22),m.I(0)),'crossing':(m.I(-22),m.I(34),m.I(0)),'exit':(m.I(34),endpre,m.I(0))}[name]
        return xx0+(xx1-xx0)*progress,yy,m.I(0)
    parking=name.startswith('parking');first=name.endswith('first');sign=(1 if yl>0 else -1)*(1 if parking else -1)
    f=progress if first else 1-progress;phi=m.I.raw(m.I(ALPHA_LO*f).l,m.I(ALPHA_HI*f).u);si,co=m.sincos(phi)
    if first:
        x0,y0=(endpre,m.I(0)) if parking else (pre,m.I(yl));x=x0+6*si;y=y0+sign*6*(m.I(1)-co)
    else:
        x1,y1=(m.I(46),m.I(yl)) if parking else (m.I(-23),m.I(0));x=x1-6*si;y=y1-sign*6*(m.I(1)-co)
    return x,y,sign*phi

def make_route(car):
    sign=direction(car);yl=sign*WAIT[car][1];merge=-1 if yl>0 else 1;parking=-merge
    commands=[];labels=[];references=[];phase_ranges={};current_d=F(0)
    def add(label,u):
        phase_ranges[label]=[len(commands),len(commands)+len(u)];commands.extend(u);labels.extend([label]*len(u))
    def turn(label,target):
        nonlocal current_d
        add(label,[(F(0),(target-current_d)/(5*H))]*5);current_d=target
    add('waiting_feedback',[(F(0),F(0))]*2)
    add('staging',trapezoid(-(D-7)));turn('merge_turn_first',merge*DELTA)
    add('merge_first',trapezoid(6*ALPHA));turn('merge_turn_second',-merge*DELTA)
    add('merge_second',trapezoid(6*ALPHA));turn('merge_turn_zero',F(0))
    add('approach',[(F(1,8),F(0))]*20)
    add('crossing',[(F(0),F(0))]*558+[(F(-5,8),F(0))]*4)
    add('exit',trapezoid(12-D));turn('parking_turn_first',parking*DELTA)
    add('parking_first',trapezoid(6*ALPHA));turn('parking_turn_second',-parking*DELTA)
    add('parking_second',trapezoid(6*ALPHA));turn('parking_turn_zero',F(0))
    assert len(commands)==758 and phase_ranges['crossing']==[101,663] and phase_ranges['exit'][0]==663
    p=(m.I(WAIT[car][0]),m.I(WAIT[car][1]),m.I(0));v=d=F(0);poses=[pose_rows(p)];knots=[];errors=[];distance=F(0)
    for j,(a,w) in enumerate(commands):
        label=labels[j];nv,nd=v+H*a,d+H*w;knots.append([v,nv,d,nd]);p=propagate(p,v,nv,d,nd,sign,4)[-1]
        if label in ('staging','approach','crossing','exit','merge_first','merge_second','parking_first','parking_second'):
            if j==phase_ranges[label][0]:distance=F(0)
            distance+=H*(v+nv)/2
            length={'staging':-(D-7),'approach':F(1),'crossing':F(56),'exit':12-D}.get(label,6*ALPHA)
            ref=reference(car,label,distance/length);ref=(sign*ref[0],sign*ref[1],ref[2])
            ep=sum((p[k]-ref[k]).abs_upper() for k in (0,1));et=(p[2]-ref[2]).abs_upper();errors.append(dict(phase=j+1,segment=label,position_l1=str(ep),yaw=str(et),body=str(ep+RB*et)))
        v,d=nv,nd;poses.append(pose_rows(p))
        assert -1<=v<=2 and abs(d)<=F(1,2) and max(abs(a),abs(w))<=1
    assert v==d==0 and max(F(x['body']) for x in errors)<EXTRA
    return dict(car=car,controls=commands,knots=knots,poses_Q=poses,phase_ranges=phase_ranges,labels=labels,analytic_error=errors,ticks=len(commands))

def main():
    assert m.sincos(m.I(ALPHA_LO))[1].l>m.I(F(3,4)).u and m.sincos(m.I(ALPHA_HI))[1].u<m.I(F(3,4)).l
    assert m.tan(m.I(DELTA_LO)).u<m.I(F(9,20)).l and m.tan(m.I(DELTA_HI)).l>m.I(F(9,20)).u
    results={};maxerr=F(0)
    for c in CARS:
        route=make_route(c);key=put(HERE/'objects',route);results[c]=key;maxerr=max(maxerr,max(F(x['body']) for x in route['analytic_error']))
    prepare={c:trapezoid(F(abs(WAIT[c][0]-INITIAL[c][0]))) for c in CARS}
    assert max(map(len,prepare.values()))==55
    write(HERE/'rational_inputs.json',dict(routes=results,preparation=prepare,alpha_q=ALPHA,delta_q=DELTA,D_q=D,maximum_body_quantization_error=maxerr,reserved_quantization_error=EXTRA,body_tube=TUBE,approach=101,crossing=562,tail=95,overlapped_ticks=55+4*663+95,serial_ticks=55+4*758))
    print(json.dumps(dict(passed=True,routes=4,max_body_error=float(maxerr),reserved=float(EXTRA),actual_ticks=2802)))
if __name__=='__main__':main()
