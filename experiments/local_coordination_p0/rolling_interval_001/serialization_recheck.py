"""Replay the failed serialization from saved raw; zero new native calls."""
from common import *
from plan_io import load,restore,save
from rolling_recover import nodes,objects,plan_at
from local_mpc import admit
from contracts import binding

def run():
 ledger=sha(HERE/'budget_ledger.json');out=[]
 n=next(n for n in nodes() if n['cycle']==413);plan=plan_at(413)
 for c in ('A1','B1'):
  row=n['rows'][c];job=objects(row['model_object']);raw=objects(row['raw_object']);binding(job,plan,restore(n['source']),raw);ad=admit(job,raw,restore(n['source']));assert ad['accepted'];encoded=json.dumps(ad,default=str,sort_keys=True);assert max(len(part) for part in encoded.replace('\"',' ').replace('/',' ').split())>4300;key=content_id(ad);save(f'results/serialization_regression/{c}_{key}.json.gz',ad);assert content_id(load(f'results/serialization_regression/{c}_{key}.json.gz'))==key;out.append({'actor':c,'saved_raw_id':row['raw_object'],'admission_id':key,'serialized_bytes':len(encoded.encode()),'accepted_but_not_retroactively_adopted':True})
 assert sha(HERE/'budget_ledger.json')==ledger;write(HERE/'results/serialization_regression.json',{'passed':True,'samples':out,'new_optimizer_calls':0,'fix':'locally produced exact Fraction certificates use unlimited integer decimal conversion in common process entry; external arbitrary request parsing is not part of this closed experiment'})
 print('PASS saved-raw serialization regression, no new solves',flush=True)
if __name__=='__main__':run()
