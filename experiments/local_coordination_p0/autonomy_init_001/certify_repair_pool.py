"""Complete three-candidate repaired pool certification with no native calls."""
import gzip
from common import *
from preflight import simultaneous
from audit import audit
from summarize import metrics

def run():
 old=json.loads(gzip.decompress((HERE/'results/init_0.json.gz').read_bytes()));plain=simultaneous(old,False);new=json.loads(gzip.decompress((HERE/'results/repair_v2/anchor.json.gz').read_bytes()))
 (HERE/'results/repair_v2/plain.json.gz').write_bytes(gzip.compress(json.dumps(plain,sort_keys=True,default=str).encode(),mtime=0));audit(plain,output=HERE/'results/repair_v2/plain_acceptance.json')
 pool=[]
 for name,w in [('initial',old),('parallel_plain',plain),('parallel_launch',new)]:
  m0=metrics(w);pool.append({'id':name,'A':len(m0['first_two'][0]),'P':sum(abs(H*H*F(w['controls'][c][0][0])/2) for c in m0['first_two'][0]),'B':w['B0'],'J':m0['input_cost'],'all_deadlines_nonincrease':all(w['per_car_policies'][c]['goal']<=old['per_car_policies'][c]['goal'] for c in CARS),'physical_replay':True})
 ranked=sorted(pool,key=lambda p:(-p['A'],-p['P'],p['B'],p['J'],p['id']));assert ranked[0]['id']=='parallel_launch'
 write(HERE/'results/repair_v2/candidate_pool.json',{'passed':True,'pool':pool,'ranked':[p['id'] for p in ranked],'scope':'only this finite three-candidate pool','native_attempts':0})
if __name__=='__main__':run()
