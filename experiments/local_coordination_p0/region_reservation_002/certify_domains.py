"""Actual L1 receiving polytopes, rational quantization reserve and whole-curve margins."""
from common import *

def main():
    source=read(HERE/'inputs/domain_certificate.json');rebuilt=read(HERE/'results/analytic_rebuild/domain_certificate.json')
    for key in ('domain_bounds','ticks','preparation_profiles'):assert source[key]==rebuilt[key]
    assert read(HERE/'inputs/geometry_certificate.json')==read(HERE/'results/analytic_rebuild/geometry_certificate.json')
    for name,digest in read(HERE/'inputs/SHA256SUMS.json').items():assert sha(HERE/'inputs'/name)==digest
    rational=read(HERE/'rational_inputs.json');quant=F(rational['maximum_body_quantization_error']);assert quant<EXTRA
    kw=F(1,100)/(1-F(1,100)**2/2)/m.L;tw=F(1,10000)+kw/500
    pw=F(1,100)+F(1,500)*(1+tw+tw*tw/2);assert pw<F(12001,10**6) and tw<F(108,10**6)
    holdP,holdT=F(12001,10**6),F(108,10**6)
    entry=holdP+F(3,2)*11*holdT+EXTRA;whole=holdP+(F(3,2)*80+RB)*holdT+EXTRA
    assert entry<F(1,50) and whole<F(1,20)
    ke=F(1,1000)/(1-F(1,1000)**2/2)/m.L;te=F(1,5000)+F(51,100)*H*ke;rot=te+te*te/2
    pout=F(1,50)+F(1,1000)*(1+rot)+(F(51,100)*H+56)*rot
    assert te<F(24,100000) and pout<F(35,1000)
    goalP=F(35,1000)+F(3,2)*13*F(24,100000)+EXTRA;goalBody=F(35,1000)+(F(3,2)*13+RB)*F(24,100000)+EXTRA
    assert goalP<F(2,25) and goalBody<F(1,20)
    from construct_inputs import DS
    overlap=33-(24+F(DS.u,m.Q))-2*TUBE;assert overlap>SAFE
    # Corner acceleration is bounded using absolute signed speed, hence reverse is covered.
    tanhalf=F(1,2)/(1-F(1,2)**2/2);kap=tanhalf/m.L;kd=(1+tanhalf*tanhalf)/m.L
    Mc=1+4*kap+RB*(kap+2*kd+4*kap*kap);assert Mc<15
    cert=read(HERE/'results/analytic_rebuild/geometry_certificate.json');leaves=[]
    for segment in cert['segments']:
        for leaf in segment['leaves']:
            road=F(leaf['road_lower'])-EXTRA;pair=F(leaf['pair_lower'])-2*EXTRA;assert road>=0 and pair>=SAFE
            leaves.append(dict(car=segment['car'],segment=segment['segment'],lo=leaf['lo'],hi=leaf['hi'],road_lower=road,pair_lower=pair))
    domains=dict(W_in=dict(position_l1='1/100',yaw='1/10000',v=['0','1/50'],delta=['-1/100','1/100']),W_hold=dict(position_l1=str(holdP),yaw=str(holdT),v=['0','0'],delta=['0','0']),G_in=dict(position_l1='1/50',yaw='1/5000',v=['49/100','51/100'],delta=['-1/1000','1/1000']),G_out=dict(position_l1='7/200',yaw='3/12500',v=['0','0'],delta=['0','0']),target=dict(position_l1='2/25',yaw='1/100',v=['0','0'],delta=['0','0']))
    value=dict(passed=True,domains=domains,source_domain_change='W_in and receiving position domains use L1 balls, subsets of original L2 balls; new L1 inclusions proved explicitly',quantization_body_bound=quant,quantization_reserve=EXTRA,total_body_tube=TUBE,bounds=dict(wait_position=pw,wait_yaw=tw,entry_position=entry,wait_full_body=whole,entry_reset_yaw=te,exit_position=pout,goal_position=goalP,goal_body=goalBody,overlap_gap=overlap,Mc_signed=Mc),timing=source['ticks'],geometry_leaves=leaves,minimum_road=min(F(x['road_lower']) for x in leaves),minimum_pair=min(F(x['pair_lower']) for x in leaves),source_sha256=sha(MODEL_PATH),feedback_laws=dict(wait_brake='a=-v/h; omega=0',wait_steer='v=0; omega=-delta/h',entry='a=(1/2-v)/h; omega=-delta/h'),flow_images='F_j = exact ZOH image of the certified source domain with saved feedback/commands; immutable command object and phase identify every image')
    key=put(HERE/'objects',value);write(HERE/'domain_checks.json',dict(certificate_id=key,**value));print(json.dumps(dict(passed=True,certificate_id=key,minimum_road=float(value['minimum_road']),minimum_pair=float(value['minimum_pair']),exit_position=float(pout),Mc_signed=float(Mc))))

def check_domain(state,car,kind,domains):
    sign=direction(car)
    domain=domains[kind];local=F(domain.get('center_local_x',-22 if kind=='G_in' else 34 if kind=='G_out' else 46))
    center=WAIT[car] if kind in ('W_in','W_hold') else (sign*local,0 if kind in ('G_in','G_in_shifted','G_out') else WAIT[car][1])
    d=domains[kind];p=state['poses'][car];error=sum((p[j]-F(center[j])).abs_upper() for j in (0,1))
    if error>F(d['position_l1']) or p[2].abs_upper()>F(d['yaw']):raise ValueError('receiving pose/yaw: '+car+'/'+kind)
    if not F(d['v'][0])<=state['v'][car]<=F(d['v'][1]) or not F(d['delta'][0])<=state['delta'][car]<=F(d['delta'][1]):raise ValueError('receiving v/delta: '+car+'/'+kind)
    return dict(car=car,kind=kind,position_l1=str(error),yaw=str(p[2].abs_upper()),v=str(state['v'][car]),delta=str(state['delta'][car]))
if __name__=='__main__':main()
