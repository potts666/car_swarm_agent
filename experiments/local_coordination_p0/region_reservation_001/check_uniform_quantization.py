"""All-time input-quantization bound, separate from endpoint interval diagnostics."""
from common import *
from construct_inputs import ALPHA,ALPHA_LO,ALPHA_HI,DELTA,DS,D


def main():
    k=m.tan(m.I(DELTA))/m.L;dk=max(abs(F(k.l,m.Q)-F(1,6)),abs(F(k.u,m.Q)-F(1,6)))
    length_error=6*max(abs(ALPHA-ALPHA_LO),abs(ALPHA_HI-ALPHA));Derror=max(abs(D-F(DS.l,m.Q)),abs(D-F(DS.u,m.Q)))
    arc_heading_error=6*ALPHA*dk+length_error/6;theta=4*arc_heading_error
    length_total=2*Derror+4*length_error;assert 62+24*ALPHA_HI<80
    position_l1=F(3,2)*(length_total+80*theta);body=position_l1+RB*theta;assert body<EXTRA
    # The real and rational lengths use the same ramp/coast counts, so each
    # within-segment traveled length is the same fraction of the total length.
    for lo,hi,q,expected in ((F(DS.l,m.Q)-7,F(DS.u,m.Q)-7,D-7,10),(6*ALPHA_LO,6*ALPHA_HI,6*ALPHA,27),(12-F(DS.u,m.Q),12-F(DS.l,m.Q),12-D,26)):
        assert 5+max(5,math.ceil(lo/H))==5+max(5,math.ceil(hi/H))==5+max(5,math.ceil(q/H))==expected
    value=dict(passed=True,scope='uniform over continuous time, all four routes, including negative staging and stationary steering; endpoint-only sampling is not the proof',curvature_error=str(dk),arc_length_error=str(length_error),D_error=str(Derror),yaw_error=str(theta),position_l1_error=str(position_l1),body_error=str(body),reserved_extra=str(EXTRA),proof='For matched trapezoidal profiles: integral |v_q-v| <= sum length errors; |yaw_q-yaw| <= sum(|k_q-k|*ell_q + |ell_q-ell|/6); L1 position <= 3/2*(length_error + 80*yaw_error); body <= position_L1 + rB*yaw_error. All real/rational counts agree.',rational_inputs_id=content_id(read(HERE/'rational_inputs.json')),domain_certificate_id=read(HERE/'domain_checks.json')['certificate_id'],model_sha256=sha(MODEL_PATH))
    key=put(HERE/'objects',value);write(HERE/'uniform_quantization_check.json',dict(certificate_id=key,**value));print(json.dumps(dict(passed=True,body_error=float(body),reserve=float(EXTRA))))
if __name__=='__main__':main()
