"""CURVE C6-C15 outward rational per-prefix constants, no example constants."""
from common import *

def up(q,scale=10**15):
 q=F(q);return F(-(-q.numerator*scale//q.denominator),scale)
def sqrt_upper(q,scale=10**15):
 q=F(q);n=math.isqrt(q.numerator*scale*scale//q.denominator)
 if F(n*n,scale*scale)<q:n+=1
 return F(n,scale)
def polynomial_moment(cut,q):
 total=F(0)
 for j in range(cut):
  for r in range(q+1):total+=math.comb(q,r)*(j*H)**(q-r)*(j*H*H*H**(r+1)/F(r+1)+H**(r+3)/F(r+3))
 return total

def profiles(knots):
 result=[];V=D=F(0)
 for j,(v0,v1,d0,d1) in enumerate(knots,1):
  perturb=H*sqrt_upper(j)*F(1,1000);V=max(V,min(F(2),max(abs(v0),abs(v1))+perturb));D=max(D,min(F(1,2),max(abs(d0),abs(d1))+perturb))
  T=F(m.tan(m.I(D)).u,m.Q);Q=1+T*T;K1=sqrt_upper(T*T+(V*Q)**2)/m.L;D2=2*V*Q*T;K2=(D2+sqrt_upper(D2*D2+4*Q*Q))/(2*m.L)
  t=j*H;S0,S1,S2=[polynomial_moment(j,q) for q in (0,1,2)];Gpsi=K1*sqrt_upper(t*S0);Bpsi=K2*S0
  Gp=sqrt_upper(t*S0)+V*K1*sqrt_upper(t**3*S0/3)
  Bp=K1*t*S0+V*K2*(t*S0-S1)+V*K1*K1*(t*t*S0-S2)/2
  Gc=Gp+RB*Gpsi;Bc=Bp+RB*(Bpsi+K1*K1*t*S0)
  Ac=1+V*V*T/m.L+RB*(T/m.L+V*Q/m.L+V*V*(T/m.L)**2)
  assert min(S0,S1,S2,Gp,Bp,Gc,Bc,Ac)>0 and D<F(1,2)
  result.append({k:up(v) for k,v in dict(t=t,V=V,D=D,tan_D=T,sec2_D=Q,K1=K1,K2=K2,S0=S0,S1=S1,S2=S2,Gp=Gp,Bp=Bp,Gpsi=Gpsi,Bpsi=Bpsi,Gc=Gc,Bc=Bc,Ac=Ac).items()})
 return result
