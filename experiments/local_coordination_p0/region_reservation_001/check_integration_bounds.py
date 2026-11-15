"""Absolute derivative bounds remain valid with v in [-1,2]."""
from common import *

def main():
    t=F(1,2)/(1-F(1,2)**2/2);k0=t/m.L;k1=(1+t*t)/m.L;k2=2*t*(1+t*t)/m.L;k3=2*(1+t*t)*(1+3*t*t)/m.L;k4=8*t*(1+t*t)*(2+3*t*t)/m.L
    p1=2*k0;p2=k0+2*k1;p3=2*k1+2*k2;p4=3*k2+2*k3;p5=4*k3+2*k4
    f4=2*(p1**4+6*p1*p1*p2+3*p2*p2+4*p1*p3+p4)+4*(p1**3+3*p1*p2+p3)
    Mc=1+4*k0+RB*(k0+2*k1+4*k0*k0)
    assert p5<84 and f4<256 and Mc<15
    values=dict(passed=True,domain=dict(v=['-1','2'],abs_a='1',abs_omega='1',abs_delta='1/2'),yaw_rate_fourth=str(p5),position_integrand_fourth=str(f4),corner_acceleration=str(Mc),implemented_caps=dict(rate_Simpson=84,position_Simpson=256,corner_chord=15),basis='Absolute-value derivatives; sign of v does not enter upper polynomials. Simpson error M*dt^5/2880; corner chord error Mc*dt^2/8.',model_sha256=sha(MODEL_PATH))
    write(HERE/'integration_bounds.json',values);print(json.dumps(values,indent=2))
if __name__=='__main__':main()
