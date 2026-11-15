"""Zero-call first actual curve source, real interval derivatives and moments."""
from common import *
from plan_io import load,source,save,record
from derivatives import build,matrix
from progression import chart,coordinate,gradient,bounds
from quantization import kernel
from curve_bounds import sqrt_upper

def run():
 p=load('results/R_original.json.gz');rows=[]
 for k in range(p['B0']):
  s=source(p,k);moving=[c for c in CARS if s['v'][c]!=0];curved=[c for c in moving if s['delta'][c]!=0]
  if not curved or len(moving)<2:rows.append({'cycle':k,'reason':'no moving curved actor plus moving peer'});continue
  c=curved[0];peer=next(x for x in moving if x!=c);actors=[c,peer];write(HERE/'results/source_probe.json',{'cycle':k,'actors':actors,'source':record(s)});print('source',k,actors,flush=True)
  for actor in actors:
   cert=build(p,s,actor,64);save(f'results/derivative_probe_{actor}.json.gz',cert)
   ne=next((j+1 for j,row in enumerate(cert['knots']) if F(row[1])==0),32);assert ne>=3
   J=matrix(cert);weights=[F(J[2][2*j+1].l+J[2][2*j+1].u,2*m.Q) for j in range(ne)];L=kernel(ne,weights)
   endpoint=tuple(m.I.raw(*q) for q in cert['poses_Q'][-1]);ch=chart(p,actor,k+32);sourcechart=chart(p,actor,k);g=gradient(endpoint,ch);gb=[sum((g[r]*J[r][t] for r in (0,1)),m.I(0)) for t in range(64)];mid=[F(x.l+x.u,2*m.Q) for x in gb];eps=sqrt_upper(sum(F(x.u-x.l,2*m.Q)**2 for x in gb));w=[F(0)]*64
   if actor==c:
    count=ne//3;Q=[sum(weights[b*count:(b+1)*count]) for b in range(3)];rr=[F(1),-(Q[0]-Q[2])/(Q[1]-Q[2]),(Q[0]-Q[1])/(Q[1]-Q[2])];sg=1 if s['delta'][actor]>0 else -1
    for j in range(3*count):w[2*j+1]=sg*rr[j//count]
    # Orthogonally remove acceleration sum/time moments from negative energy gradient.
    ref=[F(u[0]) for u in p['controls'][actor][k:k+ne]];n=F(ne);j1=sum(map(F,range(ne)));j2=sum(F(j*j) for j in range(ne));b0=sum(ref);b1=sum(j*x for j,x in enumerate(ref));det=n*j2-j1*j1;aa=(b0*j2-b1*j1)/det;bb=(n*b1-j1*b0)/det;wa=[aa+bb*j-x for j,x in enumerate(ref)];norm=max(map(abs,wa))
    for j,x in enumerate(wa):w[2*j]=x/norm
    assert all(sum(F(x)*y for x,y in zip(e,w))==0 for e in L['E'])
   wn=sqrt_upper(sum(x*x for x in w));cs=sum(x*y for x,y in zip(mid,w))-eps*wn
   out={'actor':actor,'cycle':k,'free_motion_ticks':ne,'endpoint_chart':ch,'source_chart':sourcechart,'nominal_progress_Q':[(coordinate(endpoint,ch)-coordinate(s['poses'][actor],sourcechart)).l,(coordinate(endpoint,ch)-coordinate(s['poses'][actor],sourcechart)).u],'bounds_end':cert['bounds'][-1],'gradient':mid,'gradient_epsilon':eps,'direction':w,'direction_norm':wn,'c_cert':cs,'lattice':L,'constant32_available':False,'basis':'actual mixed-segment C4-C21 and rational weighted moment; C24 constant-speed formula not claimed'}
   write(HERE/f'results/probe_{actor}.json',out);print('actor',actor,'ne',ne,'eps',float(eps),'c_cert',float(cs),'direction_norm',float(wn),flush=True)
  write(HERE/'results/source_scan.initial.json',rows);break
if __name__=='__main__':run()
