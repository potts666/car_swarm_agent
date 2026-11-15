from common import *
from canonical import reference,point,pose_error

def synthesize(plan,state,c,cert):
 phase=state['cycle']+32;refs=reference(plan);ref=refs[phase];nominal=[m.I.raw(*q) for q in cert['poses_Q'][-1]];target=[m.I.raw(*q) for q in ref['poses_Q'][c]];Lprefix=sum(abs(H*(F(refs[t]['v'][c])+F(refs[t+1]['v'][c]))/2) for t in range(phase));rp=F(1,400)+Lprefix/F(100000);rt=F(1,100000);ep=sum((nominal[j]-target[j]).abs_upper() for j in (0,1));et=(nominal[2]-target[2]).abs_upper();pos=(rp-ep)/2;yaw=(rt-et)/2;assert min(pos,yaw)>0,('receiver positive',c,phase,ep,et)
 L=sum(abs(H*(F(refs[t]['v'][c])+F(refs[t+1]['v'][c]))/2) for t in range(phase,plan['per_car_policies'][c]['goal']));bound=rp+(L+RB)*rt;assert bound<F(1,40)
 return {'version':'SR-positive-pose-v1','phase':phase,'parent':content_id(plan['controls'][c]),'center_pose_Q':cert['poses_Q'][-1],'center_pose':[F(z.l+z.u,2*m.Q) for z in nominal],'center_v':cert['knots'][-1][1],'center_delta':cert['knots'][-1][3],'rP':pos,'rPsi':yaw,'rv':0,'rdelta':0,'common_reference_phase_pose':ref['poses_Q'][c],'common_rP':rp,'common_rPsi':rt,'reference_position_error':ep,'reference_yaw_error':et,'L_remaining':L,'suffix_body_error_upper':bound,'K_id':content_id(plan['controls'][c][phase:]),'deadline':plan['per_car_policies'][c]['goal'],'inclusion':'positive receiver into canonical selected phase; exact v/delta same-input SE2 all future body error <.025, all mode and exit Q/goal hold'}
def check_domain(plan,state,c):
 p,v,d=point(c,state['cycle'],plan['per_car_policies'][c]['service_start']);error=pose_error(state['poses'][c],p);assert error<=F(1,40) and state['v'][c]==v and state['delta'][c]==d,('phase Q',c,state['cycle'],error);return {'actor':c,'phase':state['cycle'],'error':error}
