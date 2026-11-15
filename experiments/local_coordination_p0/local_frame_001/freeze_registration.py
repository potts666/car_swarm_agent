"""Freeze the12-attempt same-physical-model experiment only after zero gates."""
import shutil,sys,platform
from common import *
from plan_io import load

def run():
 assert not read(HERE/'solver_calls.json');zero=read(HERE/'results/preflight_certificate.json');assert zero['passed'];assert read(HERE/'results/outer_frame_certificate.json')['passed'];assert read(HERE/'results/zero_joint_direction_certificate.json')['passed'];sel=read(HERE/'results/source_selection.json');assert len(sel['D'])==2 and sel['X'] not in sel['D']
 for branch in ('old','new','extended'):assert read(HERE/f'results/references/{branch}_acceptance.json')['passed']
 sources={str(p.relative_to(HERE)):sha(p) for p in sorted(HERE.rglob('*.py')) if 'source_archive' not in p.parts};archive=HERE/'source_archive/numerical_v1';archive.mkdir(parents=True,exist_ok=False)
 for n in sources:
  dest=archive/n;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(HERE/n,dest)
 write(HERE/'executed_source_versions.json',sources)
 import clarabel,numpy,scipy
 write(HERE/'pre_registration.json',{'passed':True,'task':'local_frame_001','limit':12,'organizations':['C','L'],'rounds':2,'actors':sel['D']+[sel['X']],'D':sel['D'],'X':sel['X'],'source_cycle':sel['cycle'],'first_source':sel['source'],'objective':'JE=h sum normalized(a²+omega²), U_J','arm_for_model':'O-E for both C/L','h':'1/5','N':32,'rho':'1/1000','eta':'1/50','Delta':'1/1000000000000','S':[1,1],'R':[1,1],'multiplier':5,'outer_frame_id':zero['outer_frame_id'],'failures_counted':True,'retry':False,'forecast_unique_models':len(zero['models']),'actual_models_reassembled':True,'sources':sources,'python':sys.version,'platform':platform.platform(),'dependencies':{'clarabel':clarabel.__version__,'numpy':numpy.__version__,'scipy':scipy.__version__},'settings':{'max_iter':200,'tol_gap_abs':1e-10,'tol_gap_rel':1e-10,'tol_feas':1e-10,'solver_time_limit':'infinity existing default','channel_timeout':30},'scope':'union frame may read joint map/trajectories at initialization. Online worker own inputs and fixed outer dependency, global source history digest. Serialized in-process app dispatch, real spawned independent solver workers; sequential execution, no measured parallel-speed claim'})
 print('PASS frozen, limit12, attempts0',flush=True)
if __name__=='__main__':run()
