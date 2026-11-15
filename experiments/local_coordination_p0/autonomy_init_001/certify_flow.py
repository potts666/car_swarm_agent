"""Uniform geometric-template vs rational-ZOH error and phase-flow audit."""
import gzip
from common import *
from initialize import ALPHA,DELTA,D

def run():
 lo,hi=F('0.722734247813'),F('0.722734247814')
 assert m.sincos(m.I(lo))[1].l>m.I(F(3,4)).u and m.sincos(m.I(hi))[1].u<m.I(F(3,4)).l
 R=m.I(F(27,10))/m.tan(m.I(DELTA));dr=(R-6).abs_upper();angle=6*m.I(ALPHA)/R;da=max((angle-lo).abs_upper(),(angle-hi).abs_upper())
 sq=math.isqrt(7*m.Q*m.Q);idealD=3*m.I.raw(sq,sq+1);dd=(m.I(D)-idealD).abs_upper()
 w=json.loads(gzip.decompress((HERE/'results/init_0.json.gz').read_bytes()));rows=[]
 for c,path in w['paths'].items():
  L=F(path['edge_certificate']['length']);uniform=4*(2*dr+6*da)+4*da*(L+RB)+4*dd
  assert uniform<EXTRA
  rP=F(path['edge_certificate']['source_rP']);rPsi=F(path['edge_certificate']['source_rPsi'])
  assert rP+(L+RB)*rPsi+uniform<F(1,20)
  rows.append({'car':c,'radius_error':dr,'angle_error':da,'D_error':dd,'uniform_reference_body_error':uniform,'domain_body_error_bound':rP+(L+RB)*rPsi+uniform,'reserved_body_tube':'1/20','all_phases_flow':'SE(2) same-input equivariance with exact v/delta; rP_l=rP0+L_prefix*rPsi','terminal_v_delta_exact':True})
 write(HERE/'results/flow_certificate.json',{'passed':True,'rows':rows,'optimizer_calls':0})
if __name__=='__main__':run()
