"""Executable application wire, stable outer dependencies and atomic inner choices."""
import copy,time
from common import *
from frame import validate_frame_plan,source_valid,node_inclusion
from contract import validate_binding
class Wire:
 def __init__(self,name):self.path=HERE/f'results/wire/{name}.jsonl';self.path.parent.mkdir(parents=True,exist_ok=True);self.sequence=0
 def send(self,sender,receiver,kind,payload,phase):
  data=json.dumps({'sender':sender,'receiver':receiver,'kind':kind,'payload':payload},sort_keys=True,separators=(',',':'),default=str).encode();decoded=json.loads(data);assert decoded['kind']==kind;self.sequence+=1
  row={'seq':self.sequence,'time':time.time(),'phase':phase,'wire_json':data.decode(),'bytes':len(data),'sender':sender,'receiver':receiver,'kind':kind,'transport':'serialized in-process dispatch; not measured network'}
  with self.path.open('a') as f:f.write(json.dumps(row)+'\n')
  return decoded['payload']

def check_dependency(ack,plan,state,frame):
 source_valid(frame,state);assert ack['outer_frame_id']==frame['outer_frame_id']==plan['outer_frame_id'],'stale outer frame';assert ack['valid_from']<=state['cycle']<=ack['valid_through'],'expired dependency';assert ack['per_car_plan_id']==content_id(plan['controls'][ack['actor']]),'private own plan mismatch';return True

def choose(old,candidate,state,frame,D,X,org,wire,acks=None,phase='steady'):
 validate_frame_plan(old,frame);validate_frame_plan(candidate,frame);source_valid(frame,state);required=list(CARS) if org=='C' else D+[X];acks=set(required if acks is None else acks);Dversion=old.get('inner_version',0)+1;d_bundle=content_id({'version':Dversion,'plans':{c:content_id(candidate['controls'][c]) for c in D},'owners':candidate['owners'],'outer':frame['outer_frame_id']});prepared={}
 for c in required:
  coordinator='A1' if org=='C' or c in D else c
  prepare={'actor':c,'outer_frame_id':frame['outer_frame_id'],'per_car_plan_id':content_id(candidate['controls'][c]),'inner_version':Dversion if c in D else None,'bundle':d_bundle if c in D or org=='C' else None,'valid_from':state['cycle'],'valid_through':frame['valid_through'],'dependencies':{'outer_frame_id':frame['outer_frame_id']}}
  wire.send(coordinator,c,'prepare',prepare,phase)
  if c in acks:prepared[c]=wire.send(c,coordinator,'ACK',prepare,phase)
 if org=='C':takeD=takeX=set(prepared)==set(CARS)
 else:takeD=set(D)<=set(prepared);takeX=X in prepared
 if takeD:assert len({prepared[c]['bundle'] for c in D})==1,'mixed inner commit id'
 selected=copy.deepcopy(old)
 if takeD:
  for c in D:selected['controls'][c]=copy.deepcopy(candidate['controls'][c]);selected['per_car_policies'][c]=copy.deepcopy(candidate['per_car_policies'][c])
  selected['owners']=copy.deepcopy(candidate['owners']);selected['frame_branch']=candidate['frame_branch'];selected['inner_version']=Dversion;selected['encounter']=copy.deepcopy(candidate.get('encounter'));selected['inner_commit_id']=d_bundle
 if takeX:selected['controls'][X]=copy.deepcopy(candidate['controls'][X])
 selected['b_i']={c:selected['per_car_policies'][c]['goal'] for c in CARS};selected['goal_permanent']={c:[selected['b_i'][c],'infinity'] for c in CARS};selected['per_car_plan_id']={c:content_id(selected['controls'][c]) for c in CARS}
 validate_frame_plan(selected,frame)
 for c in CARS:assert selected['b_i'][c]<=old['per_car_policies'][c]['goal'],('deadline increase',c)
 if takeX:assert check_dependency(prepared[X],selected,state,frame)
 for group,members,taken in [('D',D,takeD),('X',[X],takeX)]:
  coordinator='A1' if org=='C' or group=='D' else X
  for c in members:wire.send(coordinator,c,'commit' if taken else 'retain',{'outer_frame_id':frame['outer_frame_id'],'per_car_plan_id':selected['per_car_plan_id'][c],'inner_commit_id':selected.get('inner_commit_id') if c in D else None},phase)
 return selected,{'organization':org,'required':required,'ACK':sorted(prepared),'D_participants':list(CARS) if org=='C' else D,'X_participants':list(CARS) if org=='C' else [X],'D_committed':takeD,'X_committed':takeX,'inner_commit_id':selected.get('inner_commit_id'),'outer_frame_id_before':old['outer_frame_id'],'outer_frame_id_after':selected['outer_frame_id'],'external_ack':prepared.get(X),'external_ack_still_valid':takeX,'D_old_private_ids':{c:content_id(old['controls'][c]) for c in D},'D_new_private_ids':{c:content_id(selected['controls'][c]) for c in D}}

def mixed(old,new,takeD,takeX,D,X):
 p=copy.deepcopy(old)
 if takeD:
  for c in D:p['controls'][c]=copy.deepcopy(new['controls'][c]);p['per_car_policies'][c]=copy.deepcopy(new['per_car_policies'][c])
  p['owners']=copy.deepcopy(new['owners']);p['frame_branch']=new['frame_branch'];p['encounter']=copy.deepcopy(new.get('encounter'));p['inner_version']=new.get('inner_version',0)
 if takeX:p['controls'][X]=copy.deepcopy(new['controls'][X])
 p['b_i']={c:p['per_car_policies'][c]['goal'] for c in CARS};p['goal_permanent']={c:[p['b_i'][c],'infinity'] for c in CARS};return p
