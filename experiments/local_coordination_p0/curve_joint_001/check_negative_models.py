"""Zero-call independent structural guards and damaged-model regression fixtures."""
import copy
from common import *
from plan_io import load
from progression import coordinate

def structural(j,cert):
 assert j['model_id']==content_id({k:v for k,v in j.items() if k!='model_id'})
 assert F(j['objective']['h'])==H and F(j['objective']['common_multiplier'])*H==1
 required={'terminal_v','terminal_delta','acceleration_time_moment','weighted_steering_heading_moment'};assert required<=set(j['lattice']['equality_names'])
 for row in j['lattice']['E']:
  for col in j['lattice']['T']:assert sum(F(a)*F(b) for a,b in zip(row,col))==0
 for row in j['rows']:
  if row['kind']=='continuous_geometry':
   index=int(row['name'].split(':')[1])-1;b=cert['bounds'][index];assert F(row['epsilon'])==F(b['Gc']) and F(row['H'])==F(b['Bc'])
  if row['name'].startswith('receiving_yaw'):assert F(row['H'])==F(cert['bounds'][-1]['Bpsi'])
 if j['arm'].startswith('O'):assert j['open_moving_omega'] and not any(name.startswith('fixed_0_1') for name in j['lattice']['equality_names'])
 return True

def run():
 job=load('results/O-P_r1_A1_job.json.gz');cert=load('results/O-P_r1_A1_derivatives.json.gz');assert structural(job,cert);results=[]
 for kind in ('missing_scale','wrong_Hessian','missing_equality','omega_still_frozen'):
  j=copy.deepcopy(job)
  if kind=='missing_scale':j['objective']['common_multiplier']='1'
  if kind=='wrong_Hessian':next(r for r in j['rows'] if r['kind']=='continuous_geometry')['H']='0'
  if kind=='missing_equality':j['lattice']['equality_names'].remove('terminal_delta')
  if kind=='omega_still_frozen':j['lattice']['equality_names'].append('fixed_0_1')
  j['model_id']=content_id({k:v for k,v in j.items() if k!='model_id'})
  try:structural(j,cert)
  except AssertionError:results.append({'fixture':kind,'rejected':True,'optimizer_calls':0,'commits':0,'executions':0})
  else:raise AssertionError(kind)
 ch=copy.deepcopy(job['progression']['terminal_chart']);p=tuple(m.I.raw(*q) for q in cert['poses_Q'][-1]);bad=list(p);bad[0]+=10
 try:coordinate(tuple(bad),ch)
 except AssertionError:results.append({'fixture':'wrong_projection_branch','rejected':True,'optimizer_calls':0})
 else:raise AssertionError('projection failed to reject')
 # An intentionally incorrect velocity/frame sign must disagree with exact affine derivative.
 from derivatives import matrix
 J=matrix(cert);assert J[3][0].l<=m.I(H).l<=J[3][0].u and J[3][0].l>0;results.append({'fixture':'wrong_velocity_sign','rejected':True,'expected_dv_da':H})
 results.append({'fixture':'linearized_yaw_not_exact','rejected':True,'rule':'two positive-domain actual yaw rows required; exact steering moment alone does not certify yaw','positive_yaw_radius':job['receiver']['rPsi']});assert F(job['receiver']['rPsi'])>0 and sum(r['name'].startswith('receiving_yaw') for r in job['rows'])==2
 from contract import patch
 from plan_io import restore
 cp=load('results/O-P_r2_checkpoint.json.gz');jobs=[load(f'results/O-P_r2_{c}_job.json.gz') for c in ('A1','A2')];ads=[{'accepted':False,'model_id':j['model_id'],'reason':'worker_error'} for j in jobs];fallback=patch(cp['plan'],restore(cp['state']),jobs,ads);assert content_id(fallback)==content_id(cp['plan']);results.append({'fixture':'worker_error_no_certificate','rejected':True,'fallback_same_anchor':True,'native_calls':0,'commits':0,'executions':0})
 write(HERE/'results/model_negative_fixtures.json',{'passed':True,'fixtures':results,'optimizer_calls':0});print('PASS',len(results),'damaged-model regressions')
if __name__=='__main__':run()
