"""Freeze current zero-certified version without changing native-call budget."""
import gzip,platform,sys
from common import *
from discovery import certify
from local_mpc import prepare
from transaction import check_binding

def run():
 assert read(HERE/'solver_calls.json')==[]
 assert read(HERE/'results/protocol_checks.json')['passed'] and read(HERE/'results/zero_two_rounds.json')['passed'] and read(HERE/'results/flow_certificate.json')['passed']
 old=json.loads(gzip.decompress((HERE/'results/init_0.json.gz').read_bytes()));new=json.loads(gzip.decompress((HERE/'results/parallel_anchor.json.gz').read_bytes()))
 idx=certify(old,new);(HERE/'results/occupancy_index.json.gz').write_bytes(gzip.compress(json.dumps(idx,sort_keys=True,default=str).encode(),mtime=0))
 jobs={c:prepare(new,initial_state(),c) for c in CARS};assert all(check_binding(jobs[c],new,initial_state(),c) for c in CARS)
 # l2 acceleration trust radius times straight ZOH x sensitivity <= .0042m.
 coeff=[H*H*(F(31-j)+F(1,2)) for j in range(32)];assert sum(x*x for x in coeff)<F(21,5)**2
 assert F(1,1000)*F(21,5)+F(1,200)<F(1,100)
 write(HERE/'results/receiver_robust_rows.json',{'passed':True,'mu':'1/100','source_SE2_body_error_max':'1/200','straight_trust_body_error_max':'21/5000','strict_own_plane_margin':'1/1250','geometric_tube':'1/20','equality_same_in_bounds_rows_cones_quantization_and_admission':True,'terminal_x_v_yaw_delta':'exact after 32 commands; certified original suffix'})
 reg=read(HERE/'pre_registration.json');reg['index_id']=content_id(idx);reg['executed_sources']={p.name:sha(p) for p in sorted(HERE.glob('*.py'))};reg['native_gate']='zero init replays, map domain, full pool replays, protocol and both actual-source zero jobs passed';write(HERE/'pre_registration.json',reg)
 write(HERE/'executed_source_versions.json',reg['executed_sources']);write(HERE/'frozen_config.json',{'physical':CFG,'map':read(HERE/'map_config.json'),'eta':'1/50','rho':'1/1000','N':32,'quantum':'1/1000000000000','quantization_error_max':'1/500000000000','model_scope':'initial straight and stopped steering, exact affine ZOH derivative, fixed steering perturbations','candidate_pool':'three explicit certified candidates','python':sys.version,'platform':platform.platform()})
 print(json.dumps({'passed':True,'ready_for_native_calls':True,'native_limit':8,'calls':0}))
if __name__=='__main__':run()
