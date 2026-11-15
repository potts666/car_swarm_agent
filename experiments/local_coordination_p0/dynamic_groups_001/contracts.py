"""Finite reference envelopes; pair separation covers arbitrary allowed products."""
import itertools,time
from common import *
from physics import step,predicted_nodes,geometry,body
from certify_domains import check_domain

def build(state):
    began=time.perf_counter();w=get(HERE/'objects',state['witness']);s=copy.deepcopy(state);limits=[];cap=F(100)
    for t in range(state['cycle'],w['ticks']):
        u={c:tuple(map(F,w['controls'][c][t])) for c in CARS};nodes=predicted_nodes(s,u,4);g=geometry(nodes,w['owners'][t],4);resource=F(100)
        for c in CARS:
            if c!=w['owners'][t]:
                for j in range(4):
                    b=m.axis_box(body(nodes[c][j],c)+body(nodes[c][j+1],c))
                    resource=min(resource,max(F(-16)-F(b[1],m.Q),F(b[0],m.Q)-16)-TUBE-F(15)*(H/4)**2/8)
        margin=min(g['road'],g['pair']-SAFE,resource);assert margin>0,('no positive contract margin',t)
        cap=min(cap,margin/4);limits.append([t,str(g['road']),str(g['pair']-SAFE),str(resource)])
        step(s,u,w['owners'][t],4)
        for c,kind in w['events'].get(str(t+1),[]):check_domain(s,c,kind,read(HERE/'domain_checks.json')['domains'])
    for c in CARS:check_domain(s,c,'target',read(HERE/'domain_checks.json')['domains']);assert s['v'][c]==s['delta'][c]==0
    edges={a+':'+b:content_id(dict(pair=[a,b],reference=state['witness'],cap=str(cap),phase=state['cycle'])) for a,b in itertools.combinations(CARS,2)}
    value=normal(dict(rule='finite_Hausdorff_product_v1',source=state_record(state),reference=state['witness'],phase=state['cycle'],stop=w['ticks'],cap={c:str(cap) for c in CARS},edge_ids=edges,limits_id=put(HERE/'objects',limits),owner_calendar_id=content_id(w['owners']),exit_actions_id=content_id({c:[w['starts'][c]+663,w['goal_times'][c]] for c in CARS}),goals=w['goal_times'],terminal_hold=True,wall_s=time.perf_counter()-began))
    value['discovery_id']=content_id({k:value[k] for k in ('cap','edge_ids','owner_calendar_id')})
    return put(HERE/'objects',value)

def bind(state,cid):
    c=get(HERE/'objects',cid);assert c['reference']==state['witness'] and c['phase']==state['cycle']
    state['contract_id']=cid;return state

def freedom(gid):
    import numpy as np
    g=get(HERE/'objects',gid);assert g['passed'];anchor=get(HERE/'objects',g['anchor_id']);rows=get(HERE/'objects',g['physical_rows_id']);from rules import equality_blocks
    frozen,blocks=equality_blocks(anchor['contracts']);ref=[F(x) for u in anchor['plan']['remaining'][:32] for x in u];direction=[-2*H*x for x in ref]
    for i in frozen:direction[i]=F(0)
    for block in blocks:
        active=[i for i in block if i not in frozen]
        if active:
            mean=sum((direction[i] for i in active),F(0))/len(active)
            for i in active:direction[i]-=mean
    # Retain every active affine row as a rational equality as required by section 11.
    active_rows=[list(map(F,r['linear'])) for r in rows if F(r['g_upper'])==0]
    basis=[]
    for row in active_rows:
        for b in basis:
            q=sum(x*y for x,y in zip(row,b))/sum(x*x for x in b);row=[x-q*y for x,y in zip(row,b)]
        if any(row):basis.append(row)
    for row in basis:
        q=sum(x*y for x,y in zip(direction,row))/sum(x*x for x in row);direction=[x-q*y for x,y in zip(direction,row)]
    norm=norm_upper(direction);assert norm>0,'zero projected gradient'
    # Rational lower bound of the positive roots, followed by exact row checks.
    radius=RHO;limiting='hard_trust'
    for r in rows:
        margin=-F(r['g_upper']);L=norm_upper(r['linear'])+F(r['gradient_error']);hh=F(r['hessian'])
        if margin==0:
            assert not F(r['gradient_error']) and not hh,'active nonlinear restriction'
            assert sum(F(a)*b for a,b in zip(r['linear'],direction))==0
        elif L or hh:
            bound=min(margin/(2*L) if L else RHO,norm_upper([margin/(hh+1)])/2 if hh else RHO,RHO)
            while L*bound+hh*bound*bound/2>margin:bound/=2
            if bound<radius:radius=bound;limiting=r['name']
    alpha=min(radius/(2*norm),F(1));delta=[alpha*x for x in direction]
    from socp_rows import evaluate
    proof=evaluate(rows,delta);assert proof['passed']
    gain=H*sum((u+d)**2-u*u for u,d in zip(ref,delta));assert gain<0
    return normal(dict(passed=True,rho_i=radius,limiting_row=limiting,projected_gradient=direction,projected_norm_upper=norm,alpha=alpha,descent_delta=delta,exact_cost_change=gain,active_rank=len(basis),row_proof_id=put(HERE/'objects',proof)))
