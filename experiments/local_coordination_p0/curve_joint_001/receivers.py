"""Positive pose domains and same-input full-tail SE2 inclusion, exact v/delta."""
from common import *
from plan_io import load,source,record,save
from physics import propagate
from curve_bounds import sqrt_upper
CACHE=None

def reference():
 global CACHE
 if CACHE is not None:return CACHE
 p=HERE/'results/reference_states.json.gz'
 if p.exists():CACHE=load('results/reference_states.json.gz');return CACHE
 plan=load('results/R_original.json.gz');s=initial_state();rows=[record(s)];from physics import step
 for k in range(plan['B0']):step(s,{c:plan['controls'][c][k] for c in CARS},plan['owners'][k],8);rows.append(record(s))
 CACHE=rows;save('results/reference_states.json.gz',rows);return rows

def radii(car,phase):
 # One global contract corridor surrounds all four common-reference trajectories.
 refs=reference();start=read(HERE/'results/source_probe.json')['cycle'];plan=load('results/R_original.json.gz');length=sum(abs(H*(F(refs[t]['v'][car])+F(refs[t+1]['v'][car]))/2) for t in range(start,phase))
 return F(1,400)+length*F(1,100000),F(1,100000)

def synthesize(plan,state,c,cert):
 phase=state['cycle']+32;ref=reference()[phase];nominal=[m.I.raw(*z) for z in cert['poses_Q'][-1]];target=[m.I.raw(*z) for z in ref['poses_Q'][c]];rp,rt=radii(c,phase)
 ep=sum((nominal[j]-target[j]).abs_upper() for j in (0,1));et=(nominal[2]-target[2]).abs_upper();pos=(rp-ep)/2;yaw=(rt-et)/2;assert min(pos,yaw)>0,('receiver source margin',c,phase,ep,et,rp,rt)
 tail=plan['controls'][c][phase:];Lrem=sum(abs(H*(F(reference()[t]['v'][c])+F(reference()[t+1]['v'][c]))/2) for t in range(phase,plan['per_car_policies'][c]['goal']))
 body=rp+(Lrem+RB)*rt;assert body<F(1,40) and rp<F(1,25) and rt<F(1,100)
 return {'version':'curve_pose_domain_v1','phase':phase,'parent':content_id(plan),'center_pose_Q':cert['poses_Q'][-1],'center_pose':[F(z.l+z.u,2*m.Q) for z in nominal],'center_v':cert['knots'][-1][1],'center_delta':cert['knots'][-1][3],'rP':pos,'rPsi':yaw,'rv':0,'rdelta':0,'common_reference_phase_pose':ref['poses_Q'][c],'common_rP':rp,'common_rPsi':rt,'reference_position_error':ep,'reference_yaw_error':et,'L_remaining':Lrem,'suffix_body_error_upper':body,'K_id':content_id(tail),'deadline':plan['per_car_policies'][c]['goal'],'inclusion':'local ball/interval lies in common phase domain; same-input v/delta equality yields constant yaw error and SE2 flow; entire remaining body envelope <.025; all stopped/mode guards use .025 pose-body domain, exact v/delta'}

def check_domain(state,car,phase,cap=F(1,40)):
 r=reference()[phase];p=state['poses'][car];z=[m.I.raw(*q) for q in r['poses_Q'][car]];error=sum((p[j]-z[j]).abs_upper() for j in (0,1))+RB*(p[2]-z[2]).abs_upper()
 assert error<=cap and state['v'][car]==F(r['v'][car]) and state['delta'][car]==F(r['delta'][car]),('mode_domain',phase,car,error)
 return {'phase':phase,'actor':car,'body_pose_error':error,'cap':cap,'v_delta_exact':True}
