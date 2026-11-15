"""One-tick-later receiving polytope with a proved K-to-existing-G_out suffix."""
from common import *

def main():
    old=read(HERE/'domain_checks.json');P=F(21,1000);T=F(119,500000);rv=F(1,200);rd=F(1,100000);vmax=F(1,2)+rv
    kap=rd/(1-rd*rd/2)/m.L;postT=T+vmax*H*kap;rot=postT+postT*postT/2
    postP=P+H*rv/2*(1+rot)+vmax*H*rot;endP=postP+F(559,10)*rot
    assert postT<F(3,12500) and endP<F(7,200) and endP+RB*postT<TUBE
    assert rv/H<=1 and rd/H<=1
    proof=dict(passed=True,source_domain_certificate=old['certificate_id'],source_kind='G_in_shifted',center_local_x='-219/10',position_l1=str(P),yaw=str(T),v=['99/200','101/200'],delta=['-1/100000','1/100000'],K='a=(1/2-v)/h; omega=-delta/h',nominal_remaining_distance='559/10',remaining_ticks=561,after_K_yaw=str(postT),after_K_position_l1=str(postP),G_out_position_l1_upper=str(endP),G_out_yaw_limit='3/12500',G_out_position_limit='7/200',completion_ticks=2802,proof='K normalizes v/delta exactly. Position L1 drift <= h*rv/2*(1+theta+theta^2/2) + vmax*h*(theta+theta^2/2); saved straight/braking suffix adds <=55.9*(theta+theta^2/2). This deliberately double-counts the first nominal tick and is conservative.')
    pid=put(HERE/'objects',proof);write(HERE/'shifted_receiver_certificate.json',dict(certificate_id=pid,**proof))
    value={k:v for k,v in old.items() if k!='certificate_id'};value['domains']['G_in_shifted']={k:proof[k] for k in ('position_l1','yaw','v','delta','center_local_x')};value['shifted_receiver_certificate']=pid;key=put(HERE/'objects',value);write(HERE/'domain_checks.json',dict(certificate_id=key,**value))
    print(json.dumps(dict(passed=True,certificate_id=pid,yaw_upper=float(postT),G_out_position_upper=float(endP),margin_m=float(F(7,200)-endP))))
if __name__=='__main__':main()
