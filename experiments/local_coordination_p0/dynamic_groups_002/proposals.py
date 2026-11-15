"""Finite waiting-advance pool with exact, no-optimizer launch repair."""
from common import *
from progress import ETA,distance
DELTAS=(1,2,4,8);WIDTH=4;DEPTH=2;MAX_POOL=16
_LAUNCH={};_NEXT={}

def launch_controls(route,repair):
    key=(id(route),repair)
    if key in _LAUNCH:return _LAUNCH[key]
    u=copy.deepcopy(route['controls']);indices=[j for j,label in enumerate(route['labels']) if label=='staging'];proof=None
    if repair and indices:
        first=indices[0];last=indices[-1];middle=indices[(len(indices)-1)//2];old=F(u[first][0]);sign=1 if old>0 else -1;target=sign*2*ETA/(H*H);d0=target-old;dmid=-F(last-first,last-middle)*d0;dlast=F(middle-first,last-middle)*d0
        changes={first:d0,middle:dmid,last:dlast}
        assert sum(changes.values())==0 and sum(j*d for j,d in changes.items())==0
        for j,d in changes.items():u[j][0]=str(F(u[j][0])+d);assert abs(F(u[j][0]))<=1
        proof=normal(dict(rule='three_point_impulse_first_moment_v1',indices=[first,middle,last],changes=changes,old_first_progress=H*H*abs(old)/2,new_first_progress=ETA,terminal_speed_change=sum(changes.values())*H,terminal_position_change=sum((H*H*(last-j+F(1,2))*d for j,d in changes.items()),F(0))))
    _LAUNCH[key]=(u,proof)
    return u,proof

def candidate(old,state,shifts,repair=True):
    w=copy.deepcopy(old);k=state['cycle'];repairs={}
    for c,shift in shifts.items():
        route=get(HERE/'objects',read(HERE/'rational_inputs.json')['routes'][c]);u,proof=launch_controls(route,repair);start=old['starts'][c]-shift;first=next(j for j,v in enumerate(u) if any(F(x) for x in v))
        assert state['v'][c]==0 and state['delta'][c]==0,'advance source not stationary'
        assert start+first>=k,'advance would rewrite an executed movement'
        assert all(all(F(x)==0 for x in old['controls'][c][t]) for t in range(k,old['starts'][c]+first)),'not a constant waiting segment'
        for t in range(k,w['ticks']):w['controls'][c][t]=u[t-start] if 0<=t-start<len(u) else ['0','0']
        w['starts'][c]=start;w['goal_times'][c]=old['goal_times'][c]-shift
        for t,entries in list(w['events'].items()):
            if int(t)>k:
                kept=[x for x in entries if x[0]!=c]
                if kept:w['events'][t]=kept
                else:del w['events'][t]
        for offset,kind in ((0,'W_in'),(2,'W_hold'),(101,'G_in'),(663,'G_out'),(758,'target')):
            t=max(k,start+offset);w['events'].setdefault(str(t),[]).append([c,kind])
        for t in range(k,w['ticks']):
            if w['owners'][t]==c:w['owners'][t]=None
        for t in range(max(k,start),min(w['ticks'],start+663)):
            assert w['owners'][t] in (None,c),'resource overlap: '+c+' at '+str(t)
            w['owners'][t]=c
        repairs[c]=proof
    for t in w['events']:w['events'][t]=sorted({tuple(x) for x in w['events'][t]})
    assert all(w['controls'][c][:k]==old['controls'][c][:k] for c in CARS) and w['owners'][:k]==old['owners'][:k]
    w.update(parent_witness=state['witness'],optimization_origin=None,advance=dict(shifts=shifts,repairs=repairs,source=state_record(state),rule='bounded_wait_advance_v1'))
    return normal(w)

def pool_for(old,s):
    k=s['cycle'];available=[]
    key=id(old)
    if key not in _NEXT:
        table={}
        for car in CARS:
            next_value=None;values=[None]*(old['ticks']+1)
            for t in range(old['ticks']-1,-1,-1):
                if any(F(x) for x in old['controls'][car][t]):next_value=t
                values[t]=next_value
            table[car]=values
        _NEXT[key]=table
    for c in CARS:
        if s['v'][c]!=0 or s['delta'][c]!=0 or k>=old['goal_times'][c]:continue
        future=_NEXT[key][c][k]
        if future is not None and future>k:available.append((future,c))
    available.sort();beam=[{c:d} for _,c in available[:1] for d in reversed(DELTAS)];expanded=[]
    for p in beam:
        for _,c in available[:2]:
            for d in reversed(DELTAS):
                q=p.copy();q[c]=q.get(c,0)+d;expanded.append(q)
    # The full emitted pool is bounded before any physical certification.
    expanded.sort(key=lambda p:(-max(p.values()),len(p),sorted(p.items())))
    out=[{}];seen={content_id({})}
    for p in beam+expanded:
        if content_id(p) not in seen:out.append(p);seen.add(content_id(p))
        if len(out)==MAX_POOL:break
    return out
