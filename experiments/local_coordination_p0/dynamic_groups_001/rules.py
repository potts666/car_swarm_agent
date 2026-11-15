"""Pure signed boxes and stage intersections; identifiers do not select formulas."""
from common import *

PHYSICAL=dict(v=[F(-1),F(2)],delta=[F(-1,2),F(1,2)],a=[F(-1),F(1)],omega=[F(-1),F(1)])

def intersect(a,b):
    lo,hi=max(map(F,(a[0],b[0]))),min(map(F,(a[1],b[1])))
    if lo>hi:raise ValueError('phase_or_permission: empty intersection')
    return [lo,hi]

def contract(mode):
    b=copy.deepcopy(PHYSICAL)
    if mode=='reverse':b['v']=intersect(b['v'],[-1,0])
    elif mode=='forward':b['v']=intersect(b['v'],[0,2])
    elif mode in ('stationary','hold'):
        b['v']=[F(0),F(0)];b['a']=[F(0),F(0)]
        if mode=='hold':b['delta']=[F(0),F(0)];b['omega']=[F(0),F(0)]
    elif mode!='signed':raise ValueError('unknown stage contract')
    return dict(mode=mode,**b)

def contracts_for(state,actor,length=34):
    """Decode saved witness contracts, not car-name or absolute-time branches."""
    w=get(HERE/'objects',state['witness']);k=state['cycle'];route=get(HERE/'objects',read(HERE/'rational_inputs.json')['routes'][actor]);start=w['starts'][actor];out=[]
    for t in range(k,k+length):
        # A certified free prefix inherited its original nonnegative rectangle.
        free=False;ancestor=w
        while ancestor.get('optimization_origin'):
            if ancestor['optimization_actor']==actor and ancestor['optimization_phase']<=t<ancestor['optimization_phase']+33:
                free=True;break
            ancestor=get(HERE/'objects',ancestor['parent_witness'])
        if free:mode='forward'
        elif 0<=t-start<len(route['labels']):
            label=route['labels'][t-start]
            if 'reverse' in label or label=='staging':mode='reverse'
            elif 'turn' in label:mode='stationary'
            elif label=='waiting_feedback':mode='stationary' if all(F(v)==0 for v in route['knots'][t-start][:2]) else 'forward'
            else:mode='forward'
        elif t>=start+len(route['labels']):mode='hold'
        else:mode='forward'  # preparation permits forward motion and a stopped hold
        out.append(contract(mode))
    # Boundary states must satisfy both adjacent contracts.
    for j,row in enumerate(out[:-1]):
        row['end_v']=intersect(row['v'],out[j+1]['v'])
        row['end_delta']=intersect(row['delta'],out[j+1]['delta'])
    out[-1]['end_v']=out[-1]['v'];out[-1]['end_delta']=out[-1]['delta']
    return normal(out)

def envelope(knots,box,beta=None):
    beta=norm_upper([H]*32) if beta is None else F(beta)
    v0,v1,d0,d1=map(F,knots)
    vv=intersect([min(v0,v1)-beta*RHO,max(v0,v1)+beta*RHO],box['v'])
    dd=intersect([min(d0,d1)-beta*RHO,max(d0,d1)+beta*RHO],box['delta'])
    return dict(v=vv,delta=dd,V_abs=max(map(abs,vv)),D_abs=max(map(abs,dd)))

def assert_step(v,d,u,box):
    a,w=map(F,u)
    for key,value in [('v',v),('delta',d),('a',a),('omega',w),('end_v',v+H*a),('end_delta',d+H*w)]:
        lo,hi=map(F,box.get(key,box.get(key.replace('end_',''))))
        if not lo<=value<=hi:raise ValueError('phase_or_permission: '+key+'='+str(value)+' outside '+str((lo,hi)))

def equality_blocks(boxes):
    frozen=set();blocks=[]
    for c,(key,ikey) in enumerate((('end_v','a'),('end_delta','omega'))):
        last=0
        for j,b in enumerate(boxes[:32]):
            if F(b[ikey][0])==F(b[ikey][1]):frozen.add(2*j+c)
            if F(b[key][0])==F(b[key][1]):
                blocks.append(list(range(2*last+c,2*(j+1)+c,2)));last=j+1
    return frozen,blocks

def quantize(raw,alpha,boxes):
    raw=[F(alpha)*F(1,10000)*F.from_float(float(x)) for x in raw[:64]];projected=raw.copy();frozen,blocks=equality_blocks(boxes);q=F(1,10**12)
    for i in frozen:projected[i]=0
    for block in blocks:
        active=[i for i in block if i not in frozen]
        if active:
            mean=sum((projected[i] for i in active),F(0))/len(active)
            for i in active:projected[i]-=mean
    delta=[q*round(x/q) for x in projected]
    for block in blocks:
        active=[i for i in block if i not in frozen]
        floors={i:projected[i]//q for i in active};needed=-sum(floors.values());assert 0<=needed<=len(active)
        upper=set(sorted(active,key=lambda i:(-(projected[i]/q-floors[i]),i))[:needed])
        for i in active:delta[i]=q*(floors[i]+int(i in upper))
    error=norm_upper([a-b for a,b in zip(delta,raw)])
    return delta,dict(error=str(error),budget=str(4*q),passed=error<=4*q,frozen=sorted(frozen),zero_sum_blocks=blocks)
