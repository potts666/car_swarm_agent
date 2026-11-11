"""Streaming restored prediction changes, aligned by absolute simulation time."""
import csv,gzip,json,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent

def components(a,b):
 d=np.asarray(a)-np.asarray(b);yaw=np.arctan2(np.sin(d[:,2]),np.cos(d[:,2]))
 return [float(np.linalg.norm(d[:,:2],axis=1).max()),float(abs(yaw).max()),float(abs(d[:,3]).max()),float(abs(d[:,4]).max())]
def run(directory):
 cfg=json.loads((directory/'config.json').read_text());n=round(cfg['scene']['h']/cfg['scene']['sample_s']);previous={};seeds={};data=[];max_adj=np.zeros(4);max_round=np.zeros(4)
 with gzip.open(directory/'plans.jsonl.gz','rt') as f:
  for line in f:
   p=json.loads(line);c=p['car'];body=p['states']
   if isinstance(body,dict):
    old=previous[c];assert old['id']==body['$shift_states'];p['states']=old['states'][body['skip']:]+body['tail']
   if p['kind']=='seed':seeds[c]=p;continue
   adjacent=components(np.asarray(previous[c]['states'])[n:],np.asarray(p['states'])[:-n]) if c in previous else [None]*4
   current=components(seeds[c]['states'],p['states']);max_round=np.maximum(max_round,current)
   if adjacent[0] is not None:max_adj=np.maximum(max_adj,adjacent)
   data.append(dict(cycle=p['context']['cycle'],car=c,coordination_round=p['context']['coordination_round'],candidate_source=seeds[c]['origin'],adjacent_position_m=adjacent[0],adjacent_heading_rad=adjacent[1],adjacent_speed_m_s=adjacent[2],adjacent_steering_rad=adjacent[3],round_position_m=current[0],round_heading_rad=current[1],round_speed_m_s=current[2],round_steering_rad=current[3]))
   previous[c]=p
 out=ROOT/(directory.parent.name+'_prediction_changes.csv')
 if data:
  with out.open('w') as f:w=csv.DictWriter(f,fieldnames=list(data[0]));w.writeheader();w.writerows(data)
 return dict(records=len(data),adjacent_max=max_adj.tolist(),seed_to_final_max=max_round.tolist(),alignment='previous indices 10..320 vs current 0..310, 311 nodes, shared absolute time',units=['m position norm','rad wrapped yaw','m/s','rad steering'],not_a_theoretical_bound=True)
if __name__=='__main__':
 state=json.loads((ROOT/'study_status.json').read_text());result={stage['name']:run(ROOT/stage['name']/'r1') for stage in state['stages']}
 (ROOT/'prediction_change_summary.json').write_text(json.dumps(result,indent=2)+'\n');print('prediction stages',len(result))
