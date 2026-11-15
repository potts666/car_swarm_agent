"""Signed ZOH simultaneous execution with continuous corner-error envelopes."""
from functools import lru_cache
import itertools
from common import *

@lru_cache(32768)
def cached_trig(lo,hi):
    if lo==hi==0:return m.I(0),m.I(1)
    return m.sincos(m.I.raw(lo,hi))
def trig(z):return cached_trig(z.l,z.u)

def propagate(p,v0,v1,d0,d1,sign,sub):
    if v0==v1==0:return [p]*(sub+1)
    if d0!=d1 or 0<abs(d0)<F(1,10**6):
        # Refine only numerical integration, preserving requested geometry nodes.
        return m.propagate(p,v0,v1,d0,d1,sign,sub*4)[::4]
    x,y,theta=p;a=(v1-v0)/H;nodes=[p]
    if d0:
        kap=m.tan(m.I(d0))/m.L;ksign=1 if d0>0 else -1;den=kap*ksign;s0,c0=trig(theta)
    for j in range(1,sub+1):
        t=H*j/sub;s=v0*t+a*t*t/2
        if d0:
            tt=theta+kap*s;ss,cc=trig(tt);nodes.append((x+(ss-s0)/den*(sign*ksign),y+(c0-cc)/den*(sign*ksign),tt))
        else:
            si,co=trig(theta);nodes.append((x+sign*s*co,y+sign*s*si,theta))
    return nodes

def body(p,c):
    si,co=trig(p[2]);s=direction(c)
    return [(p[0]+s*(x*co-y*si),p[1]+s*(x*si+y*co)) for x,y in itertools.product((F(-1),F(7,2)),(F(-9,10),F(9,10)))]

def predicted_nodes(state,commands,sub):
    result={}
    for c in CARS:
        a,w=map(F,commands[c]);v,d=state['v'][c],state['delta'][c];nv,nd=v+H*a,d+H*w
        if max(abs(a),abs(w))>1 or not -1<=v<=2 or not -1<=nv<=2 or max(abs(d),abs(nd))>F(1,2):raise ValueError('signed input/state bound: '+c)
        result[c]=propagate(state['poses'][c],v,nv,d,nd,direction(c),sub)
    return result


def geometry(nodes,owner,sub,tube=TUBE):
    road=pair=F(100);active={c:any(pose_rows(p)!=pose_rows(nodes[c][0]) for p in nodes[c][1:]) for c in CARS};epsilon=F(15)*(H/sub)**2/8
    static={c:body(nodes[c][0],c) for c in CARS if not active[c]}
    for j in range(sub):
        bodies={c:body(nodes[c][j],c)+body(nodes[c][j+1],c) if active[c] else static[c] for c in CARS};boxes={c:m.axis_box(b) for c,b in bodies.items()};errors={c:tube+(epsilon if active[c] else 0) for c in CARS}
        for c,b in boxes.items():
            margin=F(m.road(b,[[m.I(x).l for x in r] for r in ROADS]),m.Q)-errors[c];road=min(road,margin)
            if margin<0:raise ValueError('continuous road: '+c)
            if c!=owner and not (F(b[1],m.Q)+errors[c]<-16 or F(b[0],m.Q)-errors[c]>16):raise ValueError('unreserved body enters resource: '+c)
        for a,b in itertools.combinations(CARS,2):
            axes=m.normed_axes(nodes[a][j])+m.normed_axes(nodes[b][j]);gap=F(m.gap(bodies[a],bodies[b],boxes[a],boxes[b],axes),m.Q)-errors[a]-errors[b];pair=min(pair,gap)
            if gap<SAFE:raise ValueError('continuous six-pair separation: '+a+'/'+b)
    return dict(road=road,pair=pair,active=[c for c in CARS if active[c]],chord_error=epsilon,body_tube=tube)


def step(state,commands,owner,sub=4):
    commands={c:tuple(map(F,commands[c])) for c in CARS};nodes=predicted_nodes(state,commands,sub);bounds=geometry(nodes,owner,sub)
    for c in CARS:
        state['poses'][c]=nodes[c][-1];state['v'][c]+=H*commands[c][0];state['delta'][c]+=H*commands[c][1]
    state['history']=content_id(dict(parent=state['history'],commands=commands,owner=owner));state['cycle']+=1
    return dict(commands={c:list(map(str,u)) for c,u in commands.items()},poses_Q={c:pose_rows(state['poses'][c]) for c in CARS},v={c:str(v) for c,v in state['v'].items()},delta={c:str(d) for c,d in state['delta'].items()},history=state['history'],bounds=bounds)
