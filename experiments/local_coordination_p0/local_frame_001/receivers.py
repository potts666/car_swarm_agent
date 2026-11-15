"""Branch-indexed positive pose receivers; own-plan identity and SE2 tail."""
from common import *
from plan_io import load
CACHE={}
def reference(plan=None):
 name=plan.get('frame_branch','new') if plan else 'new'
 if name not in CACHE:CACHE[name]=load(f'results/references/{name}_states.json.gz')
 return CACHE[name]
def radii(car,phase,plan):
 refs=reference(plan);length=sum(abs(H*(F(refs[t]['v'][car])+F(refs[t+1]['v'][car]))/2) for t in range(1,phase));return F(1,400)+length*F(1,100000),F(1,100000)
def synthesize(plan,state,c,cert):
 phase=state['cycle']+32;refs=reference(plan);ref=refs[phase];nominal=[m.I.raw(*q) for q in cert['poses_Q'][-1]];target=[m.I.raw(*q) for q in ref['poses_Q'][c]];rp,rt=radii(c,phase,plan);ep=sum((nominal[j]-target[j]).abs_upper() for j in (0,1));et=(nominal[2]-target[2]).abs_upper();pos=(rp-ep)/2;yaw=(rt-et)/2;assert min(pos,yaw)>0,('receiver',c,phase,ep,et)
 Lrem=sum(abs(H*(F(refs[t]['v'][c])+F(refs[t+1]['v'][c]))/2) for t in range(phase,plan['per_car_policies'][c]['goal']));body=rp+(Lrem+RB)*rt;assert body<F(1,40) and rp<F(1,25) and rt<F(1,100)
 return {'version':'local_frame_pose_domain_v1','phase':phase,'parent':content_id(plan['controls'][c]),'center_pose_Q':cert['poses_Q'][-1],'center_pose':[F(z.l+z.u,2*m.Q) for z in nominal],'center_v':cert['knots'][-1][1],'center_delta':cert['knots'][-1][3],'rP':pos,'rPsi':yaw,'rv':0,'rdelta':0,'common_reference_phase_pose':ref['poses_Q'][c],'common_rP':rp,'common_rPsi':rt,'reference_position_error':ep,'reference_yaw_error':et,'L_remaining':Lrem,'suffix_body_error_upper':body,'K_id':content_id(plan['controls'][c][phase:]),'deadline':plan['per_car_policies'][c]['goal'],'inclusion':'positive local ball subset of common branch phase ball; exact v/delta and same-input SE2 full tail body error <.025; stopped and resource guards use branch-indexed domains'}
def check_domain(state,car,phase,plan,cap=F(1,40)):
 r=reference(plan)[phase];p=state['poses'][car];z=[m.I.raw(*q) for q in r['poses_Q'][car]];error=sum((p[j]-z[j]).abs_upper() for j in (0,1))+RB*(p[2]-z[2]).abs_upper();assert error<=cap and state['v'][car]==F(r['v'][car]) and state['delta'][car]==F(r['delta'][car]),('mode_domain',phase,car,error)
 return {'phase':phase,'actor':car,'body_pose_error':error,'cap':cap,'v_delta_exact':True}
