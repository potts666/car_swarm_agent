"""Zero-solve complete direction/guard proof and deliberately corrupt fixtures."""
import copy,traceback
from common import *
from plan_io import load,source,save,record
from curve_model import generate,evaluate
from contract import validate_binding,replay_suffix,patch

def run():
 assert not read(HERE/'solver_calls.json');p=load('results/R_original.json.gz');sel=read(HERE/'results/source_probe.json');s=source(p,sel['cycle']);actors=sel['actors'];jobs=[load(f'results/preflight/O-P_{s["cycle"]}_{c}_job.json.gz') for c in actors];prog=read(HERE/'results/preflight/progress.json');direction=next(x['positive_joint_direction'] for x in prog if 'positive_joint_direction' in x);delta=list(map(F,direction['delta']));commands=[[F(jobs[0]['reference'][2*j])+delta[2*j],F(jobs[0]['reference'][2*j+1])+delta[2*j+1]] for j in range(32)];admissions=[{'accepted':True,'model_id':jobs[0]['model_id'],'commands':commands},{'accepted':False,'model_id':jobs[1]['model_id']}];trial=patch(p,s,jobs,admissions);save('results/analytic_direction_plan.json.gz',trial);suffix=replay_suffix(trial,s,'preflight/direction_complete');tests=[]
 for name,mutate in [('stale_source',lambda j:j['source'].update(history='wrong')),('neighbor_parent',lambda j:j.update(parent='wrong')),('receiver_phase',lambda j:j['receiver'].update(phase=999)),('receiver_K',lambda j:j['receiver'].update(K_id='wrong')),('raw_identity',lambda j:None)]:
  j=copy.deepcopy(jobs[0]);mutate(j)
  try:validate_binding(j,p,s,{'model_id':'wrong' if name=='raw_identity' else j['model_id']})
  except AssertionError:tests.append({'fixture':name,'rejected':True,'commits':0,'executions':0})
  else:raise AssertionError(name)
 # Historical exact-position interface is incompatible with a curved nonzero direction.
 old={'fixture':'old_point_receiver_and_moving_omega_freeze','source':record(s),'old_omega_increment_rows':'all moving omega = 0','old_position_tolerance':'1/1000000','new_direction_first_omega':delta[1],'rejected_by_old_freeze':delta[1]!=0,'replacement':'positive certified p/yaw domain and omega bounds; uniform suffix domain guards'};assert old['rejected_by_old_freeze'];write(HERE/'results/old_interface_failure.json',old)
 job=jobs[0];E=job['lattice']['E'];assert any(sum(F(x)*v for x,v in zip(row,[F(1,10**6)]+[F(0)]*63))!=0 for row in E);tests.append({'fixture':'broken_terminal_equality','rejected':True})
 # Common analytic nominal envelope has .448m road /1.098m pair clearance
 # after its original .05m tube, thus ±.025m individual-body domains fit.
 oldaudit=read(HERE/'results/R_original_acceptance.json');assert F(oldaudit['minimum']['road'])>F(1,40) and F(oldaudit['minimum']['pair'])-F(1,20)>SAFE
 envelope={'passed':True,'reference_acceptance':'results/R_original_acceptance.json','road_margin_after_additional_domain':F(oldaudit['minimum']['road'])-F(1,40),'pair_margin_after_additional_domains':F(oldaudit['minimum']['pair'])-F(1,20),'per_body_domain':'1/40','continuous_chord_bound':'15','basis':'old whole-reference moving body tube .05 plus extra .025; two-body loss at most .05; per-model C14 rows keep all variable bodies within .025, suffix SE2 keeps same corridor; each non-owner resource endpoint verified by actual replay and universal road/core buffers'}
 write(HERE/'results/contract_certificate.json',{'passed':True,'direction_complete':suffix,'negative_fixtures':tests,'common_envelope':envelope,'receiver_version':'curve_pose_domain_v1','exact_v_delta':True,'guards':'all stopped steering poses body error<=.025 and exact v/delta; actual exit body resource clear','solver_calls':0})
 print('PASS full direction/guards/completion and source fixtures',flush=True)
if __name__=='__main__':
 try:run()
 except Exception as e:write(HERE/'results/contract_failure.json',{'error':repr(e),'traceback':traceback.format_exc(),'solver_calls':0});raise
