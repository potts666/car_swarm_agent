import shutil,sys
from common import *
from plan_io import load

def run():
 assert read(HERE/'solver_calls.json')==[];zero=read(HERE/'results/preflight_certificate.json');assert zero['passed'];assert read(HERE/'results/negative_fixtures.json')['passed']
 for n in ('R-fixed','E-anchor'):assert read(HERE/f'results/{n}_independent.json')['passed']
 windows=[x for x in zero['windows'] if 'models' in x];assert len(windows)==2 and all(len(x['models'])==4 for x in windows);assert windows[1]['cycle']>windows[0]['cycle']+1
 sources={str(q.relative_to(HERE)):sha(q) for q in HERE.rglob('*.py') if 'source_archive' not in q.parts and 'results' not in q.parts};archive=HERE/'source_archive/numerical_v1';archive.mkdir(parents=True,exist_ok=False)
 for n in sources:
  target=archive/n;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(HERE/n,target)
 write(HERE/'executed_source_versions.json',sources)
 import clarabel,numpy,scipy
 write(HERE/'pre_registration.json',{'passed':True,'task':'slack_release_001','limit':8,'event_windows':windows,'first_two_successful_events_only':True,'rounds_each':2,'actors_each':2,'selection':read(HERE/'event_selection_rule.json'),'constants':{'h':'1/5','N':32,'rho':'1/1000','eta':'1/50','R':[1,1],'S':[1,1],'Delta':'1/1000000000000','JE_unit':'normalized input square * seconds (U_J), not Joules','objective_multiplier':5},'settings':{'max_iter':200,'tolerance':1e-10,'solver_time_limit':'infinity existing default','channel_seconds':30},'python':sys.version,'versions':{'clarabel':clarabel.__version__,'numpy':numpy.__version__,'scipy':scipy.__version__},'failures_counted':True,'retry_solver':False,'forecast_scope':'anchor event windows; actual E-MPC sources after earlier updates always regenerated and actual byte assembly checked before solve','baseline':content_id(load('results/initial_bound.json.gz')),'initial_Phi':sum(load('results/R_initial.json.gz')['b_i'].values()),'source_hashes':sources})
 print('PASS freeze, budget8, attempts0',flush=True)
if __name__=='__main__':run()
