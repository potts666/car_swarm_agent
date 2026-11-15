"""Zero-native-call proposal generation, independent replay, and preregistration."""
import copy,gzip,time
from common import *
from audit import audit
from discovery import certify

def load(n):return json.loads(gzip.decompress((HERE/n).read_bytes()))
def save(n,w):(HERE/n).write_bytes(gzip.compress(json.dumps(w,sort_keys=True,default=str).encode(),mtime=0))
def simultaneous(old,launch):
 w=copy.deepcopy(old);paths=w['paths'];length={c:paths[c]['ranges']['approach_initial'][1] for c in CARS};prep=length[old['order'][0]];schedules={};rest={};starts={};elapsed=prep
 for c in old['order']:
  starts[c]=elapsed;rest[c]=paths[c]['controls'][length[c]:];elapsed+=len(rest[c])
 for c in CARS:
  u=[tuple(map(F,q)) for q in paths[c]['controls'][:length[c]]]
  if launch:
   correction=1-u[0][0]
   for j,factor in ((0,F(1)),(4,F(-9,5)),(9,F(4,5))):u[j]=(u[j][0]+factor*correction,u[j][1])
   assert all(abs(a)<=1 for a,b in u)
   assert sum(a for a,b in u)==0
  assert starts[c]>=length[c]
  schedules[c]=u+[(F(0),F(0))]*(starts[c]-length[c])+rest[c]+[(F(0),F(0))]*(elapsed-starts[c]-len(rest[c]))
  w['per_car_policies'][c]={'start':0,'goal':starts[c]+len(rest[c]),'approach_end':length[c],'service_start':starts[c],'edge_id':content_id(schedules[c])}
 w.update(controls=schedules,owners=[None]*prep+sum(([c]*len(rest[c]) for c in old['order']),[]),B0=elapsed,b_i={c:w['per_car_policies'][c]['goal'] for c in CARS},goal_permanent={c:[w['per_car_policies'][c]['goal'],'infinity'] for c in CARS},parent=content_id(old),proposal_rule='parallel independent straight-to-certified-hold, serial resource by request order; three-point moment-preserving launch' if launch else 'parallel approach')
 assert all(w['per_car_policies'][c]['goal']<=old['per_car_policies'][c]['goal'] for c in CARS),'individual absolute deadline increased'
 return w

def run():
 assert all(read(HERE/f'results/init_{k}_acceptance.json')['passed'] for k in (0,1))
 old=load('results/init_0.json.gz');plain=simultaneous(old,False);new=simultaneous(old,True)
 save('results/parallel_plain.json.gz',plain);save('results/parallel_anchor.json.gz',new)
 for name,w in [('parallel_plain',plain),('parallel_anchor',new)]:audit(w,output=HERE/f'results/{name}_acceptance.json')
 pool=[]
 for name,w in [('init_0',old),('parallel_plain',plain),('parallel_anchor',new)]:
  activity=[c for c in CARS if abs(H*H*F(w['controls'][c][0][0])/2)>=F(1,50)];J=sum(F(a)**2+F(b)**2 for c in CARS for a,b in w['controls'][c]);pool.append({'id':name,'A':len(activity),'P':len(activity)*F(1,50),'B':w['B0'],'J':J,'activity':activity,'certified':True})
 ranked=sorted(pool,key=lambda r:(-r['A'],-r['P'],r['B'],r['J'],r['id']));assert ranked[0]['id']=='parallel_anchor'
 write(HERE/'results/candidate_pool.json',{'pool':pool,'ranked':[p['id'] for p in ranked],'scope':'maximum only over declared three certified candidates'})
 index=certify(old,new);save('results/occupancy_index.json.gz',index)
 # Explicit AUT receiver radius for a minimum normalized own-plane margin.
 mu=F(1,100);gamma=F(1,100);receivers={}
 for c in CARS:
  L=F(new['paths'][c]['edge_certificate']['length']);rp=mu/4;rt=min(gamma/2,mu/(4*(L+RB)))
  assert rp+(L+RB)*rt<=mu/2
  receivers[c]={'mu':mu,'gamma':gamma,'L':L,'rP0':rp,'rPsi':rt,'rv':0,'rdelta':0,'phase_radius_rule':'rP_l=rP0+L_prefix*rPsi','remaining_tail':'same certified inputs to target; fixed v/delta exact'}
 write(HERE/'results/phase_receivers.json',receivers)
 # Historical whole witness is first opened here, after both new initial replays.
 hist=HERE.parent/'dynamic_groups_001/objects/2b5c52a0f55fd22744a2d6697da489b0d3e67e4b67307b695fdb468529c92f52.json.gz'
 historical=json.loads(gzip.decompress(hist.read_bytes()))
 write(HERE/'results/historical_comparison.json',{'read_after_two_independent_initial_replays':True,'historical_file':str(hist.resolve()),'sha256':sha(hist),'historical_ticks':len(historical['owners']),'generated_B0':old['B0'],'generated_orders':[read(HERE/f'results/init_{k}_summary.json')['order'] for k in (0,1)],'controls_equal':historical['controls']==old['controls'],'reason':'new single-car graph paths start at true initial states, full serial service; historic simultaneous preparation and pipelined exit not inherited'})
 registration={'passed':True,'source_plan':content_id(old),'selected':content_id(new),'candidate_pool':content_id(pool),'index_id':content_id(index),'source_cycle':0,'rounds':[0,1],'actors':[list(CARS),list(CARS)],'limit':8,'variables_per_actor':64,'horizon':32,'eta':'1/50','trust_l2':'1/1000','quantum':'1/1000000000000','cost':'sum(a^2+omega^2), fixed suffix','submission':'one all-four endpoint/resource transaction; first-layer new-contract anchor; missing ACK restores old full plan','source_files':{p.name:sha(p) for p in sorted(HERE.glob('*.py'))}}
 write(HERE/'pre_registration.json',registration)
 print(json.dumps({'passed':True,'B0':old['B0'],'new_B':new['B0'],'first_activity':4,'optimizer_calls':0}))
if __name__=='__main__':run()
