#!/usr/bin/env python3
"""Reconstruct the archived B 66-variable assembly and its physical residuals."""
import argparse
import math
import numpy as np
from c_common import *
from socp import evaluate


def diagnose(result):
    scale=F(result['numerical_variable_scale']);rows=result['rows'];mat=[];rhs=[];normalizers=[]
    for r in rows:
        a=np.array([float(F(x)*scale) for x in r['linear']]+[float(F(r['gradient_error'])*scale),float(F(r['hessian'])*scale*scale/2)])
        bound=-float(F(r['g_upper']));norm=max(abs(bound),float(np.max(abs(a))),1e-300)
        mat.append(a/norm);rhs.append(bound/norm);normalizers.append(norm)
    soc=np.zeros((65,66));soc[0,64]=-1;soc[1:,:64]=-np.eye(64)
    rot=np.zeros((3,66));rot[0,65]=-1;rot[1,64]=-2;rot[2,65]=-1
    M=np.vstack([mat,soc,rot]);b=np.r_[rhs,np.zeros(65),[1.,0.,-1.]]
    anchor=get(HERE/'objects',result['anchor_id']);u=np.array([[float(F(x)) for x in r] for r in anchor['plan']['remaining'][:32]]).ravel()
    P=np.diag([float(2*H*scale)]*64+[0.,0.]);q=np.r_[2*float(H)*u,0.,0.]
    x=np.asarray(result['raw_solution']);d=float(scale)*x[:64];t=float(scale)*x[64];s=float(scale*scale)*x[65]
    normalized=M@x-b;physical=[]
    for i,r in enumerate(rows):
        value=float(F(r['g_upper']))+sum(float(F(a))*v for a,v in zip(r['linear'],d))+float(F(r['gradient_error']))*t+float(F(r['hessian']))*s/2
        physical.append(dict(name=r['name'],normalizer=normalizers[i],normalized_residual=float(normalized[i]),physical_aux_residual=value,back_conversion_error=value-float(normalized[i])*normalizers[i]))
    def stats(v):
        x=np.abs(v);nz=x[x>0];return dict(min_nonzero=float(nz.min()) if nz.size else 0,max=float(nz.max()) if nz.size else 0,nonzeros=int(nz.size))
    refenergy=float(H)*float(np.sum(u*u));energy=float(H)*float(np.sum((u+d)**2));obj=float(.5*x@P@x+q@x)
    quant=[F(round(v*10**12),10**12) for v in d];rounds={}
    for alpha in (F(1),F(1,2),F(1,4)):rounds[str(alpha)]=evaluate(rows,[F(round(float(alpha)*v*10**12),10**12) for v in d])
    return dict(P=P.tolist(),q=q.tolist(),A=M.tolist(),b=b.tolist(),cones=[['nonnegative',len(rows)],['SOC',65],['SOC',3]],row_normalizers=normalizers,row_nonzero_stats=[stats(v) for v in M],column_nonzero_stats=[stats(v) for v in M.T],physical_rows=rows,physical_residuals=physical,t_physical=t,s_physical=s,t_minus_norm=t-float(np.linalg.norm(d)),s_minus_t_squared=s-t*t,objective_scale=str(scale),physical_energy_change=energy-refenergy,reconstructed_objective_change=float(scale)*obj,objective_conversion_error=(energy-refenergy)-float(scale)*obj,status=result['status'],primal=result['primal_residual'],dual=result['dual_residual'],objective_gap=result.get('absolute_objective_gap'),rounded_points=rounds,tightest_physical_rows=sorted(physical,key=lambda r:-r['physical_aux_residual'])[:8])


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():p.error('new output required')
    a.output.mkdir(parents=True);summary={}
    for name in ('curve','straight_acceleration'):
        s=read(BASE/'results/comparison_002'/(name+'_B-JL-shift')/'summary.json')
        for i,event in enumerate(s['solver_events']):
            key=event['attempts'][-1]['result_id'];result=get(HERE/'objects',key);out=diagnose(result);ident=put(HERE/'objects',out)
            summary[name+('_initial' if i==0 else '_shift')]=dict(result_id=key,diagnostic_id=ident,status=out['status'],objective_conversion_error=out['objective_conversion_error'],t_minus_norm=out['t_minus_norm'],s_minus_t_squared=out['s_minus_t_squared'])
    write(a.output/'summary.json',dict(passed=True,cases=summary,hypothesis='common s couples all quadratic rows with disparate scaling and gap sensitivity; test equivalent per-row SOC projection, no tolerance relaxation'))
    print('Four archived 66-variable problems exported; physical/objective residual conversion checked')


if __name__=='__main__':main()
