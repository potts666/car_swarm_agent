"""External numerical provenance and certificate checks, independent terminal rows."""
from common import *
from derivatives import verify,unpack
from socp import evaluate
from socp_anchor import derivative_bounds as global_bounds


def check(data,m,anchor,cert,result,proposal=None,recompute=True):
    verify(data,m,cert,anchor,recompute)
    assert result['anchor_id']==content_id(anchor) and result['method'] in ('B-ZG','B-ZL','B-JL')
    rows=result['rows'];terminal=[r for r in rows if r['kind']=='terminal'];assert len(terminal)==6
    end=cert['terminal_state'];actor=anchor['actor'];domain=data.row(anchor['receiver_phase'])['domain'];pose=[m.I.raw(*p) for p in end['poses_Q'][actor]];center=[F(*q) for q in domain['center_pose_Q']]
    local={k:F(v) for k,v in cert['local_profiles'][-1].items()};bounds=global_bounds() if result['method']=='B-ZG' else local
    S=unpack(m,cert['prefix_jacobians'][-1]);expected={}
    for x in (-1,1):
        for y in (-1,1):expected[f'position_{x}_{y}']=(F((x*(pose[0]-center[0])+y*(pose[1]-center[1])).u,m.Q)-F(domain['position_l1_radius']),[x*S[0][j]+y*S[1][j] for j in range(64)],F(3,2)*bounds['Gp'],F(3,2)*bounds['Hp'])
    for sign in (-1,1):expected[f'yaw_{sign}']=(F((sign*(pose[2]-center[2])).u,m.Q)-F(domain['yaw_radius']),[sign*x for x in S[2]],bounds['P1'],bounds['P2'])
    for row in terminal:
        g,J,bound,hess=expected[row['name']]
        if result['method']=='B-JL':gradient=[F(q.l+q.u,2*m.Q) for q in J];error=norm_upper([F(q.u-q.l,2*m.Q) for q in J])
        else:gradient=[F(0)]*64;error=bound
        assert F(row['g_upper'])==g and list(map(F,row['linear']))==gradient
        assert F(row['gradient_error'])==error,'gradient error corrupted'
        assert F(row['hessian'])==hess,'Hessian source corrupted'
    geometry=[r for r in rows if r['kind']=='geometry'];assert len(geometry)==32
    for j,row in enumerate(geometry):
        assert F(row['g_upper'])==-F(cert['geometry_margins'][j]) and F(row['gradient_error'])==F(cert['local_profiles'][j]['Gc']) and F(row['hessian'])==F(cert['local_profiles'][j]['Hc'])
        assert not any(map(F,row['linear']))
    trust=next(r for r in rows if r['name']=='hard_trust');assert F(trust['g_upper'])==-RHO and F(trust['gradient_error'])==1 and F(trust['hessian'])==0
    if not result['accepted']:return True
    assert result['status']=='Solved' and result['solver_called'] and result['variables']==66 and result['input_variables']==64
    assert F(result['physical_trust_radius'])==RHO and result['solver_tolerance']<=1e-10
    raw=result['raw_solution'];scale=F(result['numerical_variable_scale']);alpha=F(result['accepted_alpha']);assert alpha in (1,F(1,2),F(1,4))
    physical=[float(scale)*float(x) for x in raw[:64]]
    delta=[F(round(float(alpha)*x*10**12),10**12) for x in physical]
    assert list(map(F,result['accepted_delta']))==delta,'input did not come from raw Clarabel point'
    cone=evaluate(rows,delta);assert cone['passed'] and F(cone['norm_upper'])<=RHO,'rational cone rejection'
    accepted=result['accepted_proposal'];assert proposal is None or proposal==accepted
    assert accepted['id']==A.seal(accepted)['id']
    state=restore(m,anchor['state']);old=anchor['plan'];commands=[tuple(F(u)+delta[2*j+k] for k,u in enumerate(row)) for j,row in enumerate(old['remaining'][:32])]
    assert [tuple(map(F,u)) for u in accepted['remaining'][:32]]==commands
    assert accepted['parent']==old['id'] and accepted['witness']==data.witness_id and accepted['budget']==old['budget']
    r=accepted['receiver'];row=data.row(anchor['receiver_phase']);rr=accepted['pending_reset'];domain=row['domain']
    assert r['phase']==anchor['phase']+32 and r['deadline']==32 and r['domain_object']==data.cert['_object_id']
    assert r['row_sha256']==content_id(row) and r['domain']==domain and r['component']=='box'
    assert rr['phase']==r['phase'] and rr['next_phase']==r['phase']+1 and rr['row_sha256']==content_id(row)
    v=state['v'][anchor['actor']]+H*sum(a for a,w in commands);d=state['delta'][anchor['actor']]+H*sum(w for a,w in commands)
    K=(F(domain['feedback_a_ref'])-(v-F(domain['center_v']))/H,F(domain['feedback_omega_ref'])-(d-F(domain['center_delta']))/H)
    assert tuple(map(F,rr['input']))==tuple(map(F,accepted['remaining'][32]))==K,'corrupted K'
    assert F(result['accepted_prefix_energy'])==H*sum(sum(x*x for x in row) for row in commands)
    return True
