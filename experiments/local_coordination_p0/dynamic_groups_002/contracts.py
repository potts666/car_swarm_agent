"""Explicit normalized physical separation planes and causally valid resource calendars."""
import itertools,time
from common import *
from physics import predicted_nodes,geometry,step,body
from certify_domains import check_domain
from progress import atlas_identity
PAIRS=list(itertools.combinations(CARS,2))

def support(rect,n):
    values=[x*n[0]+y*n[1] for x,y in rect];return F(min(x.l for x in values),m.Q),F(max(x.u for x in values),m.Q)

def build(state,obligations=None,base=None):
    began=time.perf_counter();w=get(HERE/'objects',state['witness']);s=copy.deepcopy(state);limits=[];faces={};cap=F(100);prior=get(HERE/'objects',base) if base else None;oldfaces=get(HERE/'objects',prior['faces_id']) if prior else None
    for t in range(state['cycle'],w['ticks']):
        u={c:tuple(map(F,w['controls'][c][t])) for c in CARS};nodes=predicted_nodes(s,u,4);g=geometry(nodes,w['owners'][t],4);resource=F(100);minimum_face=F(100);rows=[]
        for j in range(4):
            rect={c:body(nodes[c][j],c)+body(nodes[c][j+1],c) for c in CARS};bounds={c:m.axis_box(rect[c]) for c in CARS};pad=TUBE+F(15)*(H/4)**2/8
            for c in CARS:
                if c!=w['owners'][t]:b=bounds[c];resource=min(resource,max(-16-F(b[1],m.Q),F(b[0],m.Q)-16)-pad)
            for pairno,(a,b) in enumerate(PAIRS):
                if prior:
                    row=oldfaces[str(t)][j*len(PAIRS)+pairno];assert row[:2]==[j,pairno];nx,ny,beta=map(F,row[2:]);n=(nx,ny)
                else:
                    aa,bb=bounds[a],bounds[b];axis=[(F(bb[0]-aa[1],m.Q),(F(1),F(0))),(F(aa[0]-bb[1],m.Q),(F(-1),F(0))),(F(bb[2]-aa[3],m.Q),(F(0),F(1))),(F(aa[2]-bb[3],m.Q),(F(0),F(-1)))];gap,n=max(axis,key=lambda x:x[0])
                    if gap<F(2):
                        for n0 in m.normed_axes(nodes[a][j])+m.normed_axes(nodes[b][j]):
                            for sign in (1,-1):
                                nn=tuple(sign*x for x in n0);al,ah=support(rect[a],nn);bl,bh=support(rect[b],nn)
                                if bl-ah>gap:gap,n=bl-ah,nn
                    al,ah=support(rect[a],n);bl,bh=support(rect[b],n);beta=(ah+bl)/2
                assert 0<n[0]*n[0]+n[1]*n[1]<=1,'invalid normalized direction'
                al,ah=support(rect[a],n);bl,bh=support(rect[b],n);slack=min(beta-SAFE/2-ah-pad,bl-pad-beta-SAFE/2);assert slack>0,('physical separation face',t,j,a,b,str(slack))
                minimum_face=min(minimum_face,slack);rows.append([j,pairno,str(n[0]),str(n[1]),str(beta)])
        margin=min(g['road'],minimum_face,resource);assert margin>0,('contract margin',t,str(margin));cap=min(cap,margin/4);faces[str(t)]=rows;limits.append([t,str(g['road']),str(g['pair']-SAFE),str(resource),str(minimum_face)])
        step(s,u,w['owners'][t],4)
        for c,kind in w['events'].get(str(t+1),[]):check_domain(s,c,kind,read(HERE/'domain_checks.json')['domains'])
    for c in CARS:check_domain(s,c,'target',read(HERE/'domain_checks.json')['domains']);assert s['v'][c]==s['delta'][c]==0
    fid=prior['faces_id'] if prior else put(HERE/'objects',faces);edges=prior['edge_ids'] if prior else {a+':'+b:content_id(dict(pair=[a,b],planes=fid,safety=str(SAFE))) for a,b in PAIRS}
    value=normal(dict(rule='explicit_finite_planes_v1',source=state_record(state),reference=state['witness'],phase=state['cycle'],stop=w['ticks'],cap={c:str(cap) for c in CARS},edge_ids=edges,faces_id=fid,limits_id=put(HERE/'objects',limits),owner_calendar_id=content_id(w['owners']),exit_actions_id=content_id({c:[w['starts'][c]+663,w['goal_times'][c]] for c in CARS}),goals=w['goal_times'],obligations=obligations or {},atlas_id=atlas_identity(w),terminal_hold=True,base_physical_contract=base,wall_s=time.perf_counter()-began))
    value['discovery_id']=content_id({k:value[k] for k in ('cap','edge_ids','owner_calendar_id')});return put(HERE/'objects',value)

def bind(state,cid):
    c=get(HERE/'objects',cid);assert c['reference']==state['witness'] and c['phase']==state['cycle'];state['contract_id']=cid;return state

def physical_diff(oldid,newid):
    old,new=(get(HERE/'objects',c) for c in (oldid,newid));a,b=(get(HERE/'objects',c['faces_id']) for c in (old,new));changed=[];maximum=F(0)
    for t in b:
        for r,s in zip(a[t],b[t]):
            if r!=s:
                delta=[F(y)-F(x) for x,y in zip(r[2:],s[2:])];maximum=max(maximum,abs(delta[-1]));changed.append(dict(cycle=int(t),sub=r[0],pair=PAIRS[r[1]],old=r[2:],new=s[2:],delta=list(map(str,delta))))
    wa,wb=(get(HERE/'objects',c['reference']) for c in (old,new));calendar=[dict(cycle=k,old=wa['owners'][k],new=wb['owners'][k]) for k in range(new['phase'],wb['ticks']) if wa['owners'][k]!=wb['owners'][k]]
    return normal(dict(old_contract=oldid,new_contract=newid,changed_face_count=len(changed),max_beta_change_m=maximum,face_changes_id=put(HERE/'objects',changed),resource_changes=calendar,physical_changed=bool(changed or calendar),closure=sorted({c for row in changed for c in row['pair']}|{x for row in calendar for x in (row['old'],row['new']) if x})))
