"""Directed line/arc atlas, reused interval atan projection and certified seams."""
from common import *
from curve_bounds import sqrt_upper

def atan_interval(x):
 radius=x.abs_upper();assert radius<F(9,10),'projection branch ratio exceeded'
 total=m.I(0);power=x;square=x*x
 for j in range(96):total+=power*F((-1)**j,2*j+1);power=power*square
 error=radius**193/193;pad=m.I(error).u;return m.I.raw(total.l-pad,total.u+pad)

def chart(plan,c,phase):
 policy=plan['per_car_policies'][c];route=plan['paths'][c];end=route['ranges']['approach_initial'][1]
 if phase<policy['service_start']:
  return {'kind':'line','mode':'initial_approach_or_hold','origin':[[m.I(F(TASKS[c]['initial'][j])).l]*2 for j in range(3)],'car':c,'offset':'0','curvature':'0'}
 local=min(route['ticks']-1,phase-policy['service_start']+end);name=route['labels'][local]
 if 'turn' in name:
  # Terminal objectives may not sit on a mode seam; selection rejects these.
  return {'kind':'line','mode':name,'car':c,'origin':route['poses_Q'][local],'offset':str(sum(H*(F(v0)+F(v1))/2 for v0,v1,d0,d1 in route['knots'][:local])),'curvature':'0','report_only_stopped_tangent':True}
 start=route['ranges'][name][0];origin=route['poses_Q'][start];delta=F(route['knots'][start][2]);distance=sum(H*(F(v0)+F(v1))/2 for v0,v1,d0,d1 in route['knots'][:start])
 return {'kind':'arc' if delta else 'line','mode':name,'origin':origin,'curvature':str(delta),'car':c,'offset':str(distance),'entry_phase':policy['service_start']+start-end,'end_phase':policy['service_start']+route['ranges'][name][1]-end,'radius':'6' if delta else None,'seam_rule':'oriented tangent-half-plane selects adjacent C1 line/arc patch; both charts give equal offset on shared normal line; objective domain stays strictly inside one patch'}

def frame(ch):
 p=[m.I.raw(*q) for q in ch['origin']];si,co=m.sincos(p[2]);sg=direction(ch['car']);return p,[sg*co,sg*si],[-sg*si,sg*co]
def coordinate(p,ch):
 assert ch['kind'] in ('line','arc'),'objective chart falls on stopped curvature-change seam'
 origin,tangent,normal=frame(ch);dx=[p[r]-origin[r] for r in (0,1)];t=sum((x*y for x,y in zip(dx,tangent)),m.I(0));n=sum((x*y for x,y in zip(dx,normal)),m.I(0));offset=m.I(F(ch['offset']))
 if ch['kind']=='line':return offset+t
 kap=m.tan(m.I(F(ch['curvature'])))/m.L;den=m.I(1)-kap*n;assert den.l>0
 radius_residual=2*n-kap*(t*t+n*n);assert radius_residual.abs_upper()<F(1,5),'outside certified tubular arc'
 angle=atan_interval(kap*t/den);sign=1 if F(ch['curvature'])>0 else -1
 return offset+sign*angle/(sign*kap)
def gradient(p,ch):
 origin,tangent,normal=frame(ch)
 if ch['kind']=='line':return tangent
 dx=[p[r]-origin[r] for r in (0,1)];t=sum((x*y for x,y in zip(dx,tangent)),m.I(0));n=sum((x*y for x,y in zip(dx,normal)),m.I(0));kap=m.tan(m.I(F(ch['curvature'])))/m.L;den=m.I(1)-kap*n;D=den*den+(kap*t)*(kap*t);assert D.l>0
 return [(den*tangent[r]+kap*t*normal[r])/D for r in (0,1)]
def bounds(ch,tube=F(1,100)):
 if ch['kind']=='line':return {'r_min':None,'G':F(1),'H':F(0),'tube':tube}
 kap=m.tan(m.I(F(ch['curvature'])))/m.L;K=kap.abs_upper();radius_lo=1/K;radius_hi=1/F(min(abs(kap.l),abs(kap.u)),m.Q);rmin=radius_lo-tube;assert rmin>0
 return {'r_min':rmin,'radius_upper':radius_hi,'G':radius_hi/rmin,'H':radius_hi/rmin**2,'tube':tube}
