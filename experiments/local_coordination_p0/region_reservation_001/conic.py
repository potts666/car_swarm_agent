import numpy as np
from scipy import sparse
import clarabel,math
from common import *

def old_assembly(rows,reference,scale):
    nonneg=[];rhs=[];socblocks=[];socrhs=[];cones=[];normalizers=[];conversions=[]
    for row in rows:
        a=list(map(F,row['linear']));g=F(row['g_upper']);e=F(row['gradient_error']);Hess=F(row['hessian'])
        L=max(abs(g),*(abs(x*scale) for x in a),abs(e*scale),Hess*scale*scale/2,F(1,10**100))
        linear=[x*scale/L for x in a]+[e*scale/L];rconst=-g/L;normalizers.append(str(L))
        errors=[abs(F.from_float(float(x))-x) for x in linear]
        if Hess==0:
            nonneg.append(list(map(float,linear)));rhs.append(float(rconst))
            conversions.append(dict(name=row['name'],normalizer=str(L),maximum_coefficient_error=str(max(errors)),constant_error=str(abs(F.from_float(float(rconst))-rconst))))
        else:
            K=2*Hess*scale*scale/L;root=math.sqrt(float(K));block=np.zeros((3,65));block[0]=list(map(float,linear));block[2]=block[0];block[1,64]=-root
            socblocks.append(block);socrhs.extend([float(rconst+1),0.,float(rconst-1)])
            conversions.append(dict(name=row['name'],normalizer=str(L),root_squared_error=str(abs(F.from_float(root)**2-K)),maximum_coefficient_error=str(max(errors)),constant_plus_error=str(abs(F.from_float(float(rconst+1))-(rconst+1))),constant_minus_error=str(abs(F.from_float(float(rconst-1))-(rconst-1)))))
    norm=np.zeros((65,65));norm[0,64]=-1;norm[1:,:64]=-np.eye(64)
    blocks=[np.asarray(nonneg),norm,*socblocks];b=np.r_[rhs,np.zeros(65),socrhs]
    cones=[clarabel.NonnegativeConeT(len(nonneg)),clarabel.SecondOrderConeT(65)]+[clarabel.SecondOrderConeT(3) for _ in socblocks]
    P=sparse.diags([float(2*H*scale)]*64+[0.],format='csc');q=np.r_[2*float(H)*reference,0.]
    return P,q,sparse.csc_matrix(np.vstack(blocks)),b,cones,dict(row_normalizers=normalizers,coefficient_conversion=conversions,cone_dimensions=[len(nonneg),65]+[3]*len(socblocks),P=P.toarray().tolist(),q=q.tolist(),A=np.vstack(blocks).tolist(),b=b.tolist(),representation='R=-f_linear/L; (R+1,sqrt(2*Hrow*scale^2/L)*tau,R-1) in SOC')


def assembly(rows,reference,scale):
    P,q,a,b,cones,export=old_assembly(rows,reference,scale)
    a=a.toarray();b=b.copy();offset=export['cone_dimensions'][0]+65;branches=[];errors=[]
    for row in rows:
        g=F(row['g_upper']);h=F(row['hessian'])
        if not h:branches.append('linear');continue
        if g<0:
            ell=[scale*F(x)/(-g) for x in row['linear']]+[scale*F(row['gradient_error'])/(-g)]
            beta=2*h*scale**2/(-g);root=math.sqrt(float(beta))
            a[offset]=list(map(float,ell));a[offset+1]=0;a[offset+1,64]=-root;a[offset+2]=a[offset];b[offset:offset+3]=(2,0,0)
            errors.append(dict(name=row['name'],maximum_coefficient_error=str(max(abs(F.from_float(float(x))-x) for x in ell)),root_squared_error=str(F.from_float(root)**2-beta)))
            branches.append('centered')
        else:branches.append('old_valid_cone_nonpositive_margin')
        offset+=3
    assert offset==len(b)
    export.update(A=a.tolist(),b=b.tolist(),representation='centered_equivalent_v1',branches=branches,centered_conversion=errors)
    return P,q,sparse.csc_matrix(a),b,cones,export
