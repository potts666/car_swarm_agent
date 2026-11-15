import time
from common import *
class Wire:
 def __init__(self,name):self.path=HERE/f'results/wire/{name}.jsonl';self.path.parent.mkdir(parents=True,exist_ok=True);self.sequence=0
 def send(self,sender,receiver,kind,payload,phase):
  data=json.dumps({'sender':sender,'receiver':receiver,'kind':kind,'payload':payload},sort_keys=True,separators=(',',':'),default=str).encode();decoded=json.loads(data);assert decoded['kind']==kind;self.sequence+=1
  row={'seq':self.sequence,'time':time.time(),'phase':phase,'wire_json':data.decode(),'bytes':len(data),'sender':sender,'receiver':receiver,'kind':kind,'transport':'serialized in-process dispatch; not measured network'}
  with self.path.open('a') as f:f.write(json.dumps(row)+'\n')
  return decoded['payload']

