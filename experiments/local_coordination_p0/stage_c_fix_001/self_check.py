"""Exact cone equivalence and trust-boundary regression, no new solves."""
from fix_socp import *
from probe import strict_interior
from validate_fix import check

def main():
    count=0
    for beta in (F(0),F(1,17),F(10),F(1000)):
        for tau in (F(-2),F(-1,3),F(0),F(1,3),F(2)):
            boundary=1-beta*tau*tau/4
            for w in (-2,F(0),F(1),F(3,2),boundary,boundary-F(1,100),boundary+F(1,100)):
                lhs=(2-w)**2-w*w-beta*tau*tau;rhs=4*(1-w-beta*tau*tau/4)
                assert lhs==rhs
                assert (2-w>=0 and lhs>=0)==(rhs>=0);count+=1
    # Actual assembly branches, including unavailable anchors, both signs of tau.
    branch_tests=[]
    for h in (F(0),F(2)):
        for g in (F(-1),F(0),F(1)):
            row=dict(name='edge',linear=[F(1)]+[F(0)]*63,g_upper=g,gradient_error=F(1,2),hessian=h,kind='test')
            linear=dict(row,name='linear_anchor',g_upper=-1,hessian=0)
            fixture=[linear,row]
            old=old_assembly(fixture,np.zeros(64),F(1,10000))[-1];new=assembly(fixture,np.zeros(64),F(1,10000))[-1]
            if not h or g>=0:
                assert old['A']==new['A'] and old['b']==new['b']
            else:
                assert new['b'][-3:]==[2,0,0]
                assert all(v==0 for v in new['A'][-2][:-1])
            branch_tests.append(dict(h=str(h),g=str(g),branch=new['branches'][-1],zero_anchor_legal=g<=0))
    q=F(1,10**12)
    for i in range(-64,65):
        raw=[F(i,3)*q+q/2]*64;assert norm_upper([a-b for a,b in zip(nearest(raw),raw)])<=4*q
    data,m=setup();summary=read(HERE/'results/probe/summary.json');rejections=[];independent=[]
    for name in ('curve','straight_acceleration'):
        source=get(HERE/'objects',summary[name+'_centered']['result_id']);anchor=get(HERE/'objects',source['anchor_id']);cert=get(HERE/'objects',source['derivative_id'])
        for status in ('NumericalError','InsufficientProgress','MaxTime','PrimalInfeasible'):
            bad=copy.deepcopy(source);bad['status']=status
            assert not admit(data,m,anchor,cert,bad,'certificate_suboptimal_v1')['accepted'];rejections.append(name+':'+status)
        bad=copy.deepcopy(source);bad['raw_solution'][0]=float('nan');assert not admit(data,m,anchor,cert,bad,'certificate_suboptimal_v1')['accepted'];rejections.append(name+':NaN')
        accepted=get(HERE/'objects',summary[name+'_centered']['chosen_id']);accepted['interface']=name
        assert check(data,m,anchor,cert,accepted)
        for key in ('raw_solution','anchor_id','derivative_id'):
            bad=copy.deepcopy(accepted)
            if key=='raw_solution':bad[key][0]+=1
            else:bad[key]='0'*64
            try:check(data,m,anchor,cert,bad,recompute=False)
            except (AssertionError,ValueError,KeyError):rejections.append(name+':'+key)
            else:raise AssertionError('bad provenance admitted')
        model=get(HERE/'objects',source['assembly_id']);strict_interior(model)
        old=anchor['plan'];state=restore(m,anchor['state'])
        for kind in ('K','phase','object','domain','deadline','witness','budget'):
            bad=copy.deepcopy(accepted['accepted_proposal'])
            if kind=='K':bad['pending_reset']['input'][0]=str(F(bad['pending_reset']['input'][0])+1)
            elif kind=='phase':bad['phase']+=1
            elif kind=='object':bad['object']='0'*64
            elif kind=='domain':bad['receiver']['domain']['speed_radius']='1'
            elif kind=='deadline':bad['receiver']['deadline']+=1
            elif kind=='witness':bad['witness']='0'*64
            elif kind=='budget':bad['budget']+=1
            bad=A.seal(bad)
            try:A.commit(m,data,state,old,bad,old['phase'],old['phase'])
            except ValueError as e:rejections.append(name+':'+kind+':'+str(e))
            else:raise AssertionError('damaged proposal committed')
        independent.append(name)
    original=HERE.parent
    preservation={name:sha(original/name)==digest for name,digest in read(HERE/'historical_hashes.json').items()};assert all(preservation.values())
    inputs={name:sha(HERE/'instructions'/name)==digest for name,digest in read(HERE/'instructions/SHA256SUMS.json').items()};assert all(inputs.values())
    manifest=read(HERE/'base_c/DELIVERY_MANIFEST.json')['files'];assert all(sha(HERE/'base_c'/name)==v['sha256'] for name,v in manifest.items())
    result=dict(passed=True,exact_identity_cases=count,branch_tests=branch_tests,quantization_cases=129,rejected_cases=rejections,independent_numeric_certificates=independent,history_files_unchanged=len(preservation),instructions_hashes_verified=inputs,original_C_manifest_verified=len(manifest))
    write(HERE/'results/self_checks.json',result);print('Exact equivalence, quantization, provenance, damage rejection and preservation checks passed',flush=True)
if __name__=='__main__':main()
