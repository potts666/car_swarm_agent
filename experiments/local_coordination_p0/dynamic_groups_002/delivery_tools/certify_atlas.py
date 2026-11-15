"""Save chart derivative/uniqueness bounds and test a curved old actual source."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from common import *
from progress import chart,projection_bounds,coordinate,ETA
from contracts import build,bind
from generation import generate
r=read(HERE/'pre_registration.json');w=get(HERE/'objects',r['candidate_witness']);rows=[]
for c in CARS:
 route=get(HERE/'objects',read(HERE/'rational_inputs.json')['routes'][c]);seen=set()
 for j,label in enumerate(route['labels']):
  ch=chart(w,c,w['starts'][c]+j);key=content_id(ch)
  if key in seen:continue
  seen.add(key);bounds=projection_bounds(ch);rows.append(dict(car=c,mode=label,chart_id=key,chart=ch,derivative_bounds=bounds))
oldcases=read(HERE.parent/'dynamic_groups_001/regression_sources.json');entry=next(x for x in oldcases if x['kind']=='existing_forward');s=restore(m,entry['source']);cid='816f85f30ea799f62553e2de1642e8ac51e4c81b7e84b71e8df19d796f83b052';assert get(HERE/'objects',cid)['source']==normal(state_record(s));bind(s,cid);g,gid=generate(s,entry['actor']);assert g['passed'],g;physical=get(HERE/'objects',g['physical_rows_id']);assert any(x.get('rule')=='certified_path_projection_Taylor' for x in physical if x['kind']=='progress')
write(HERE/'results/atlas_certification.json',dict(passed=True,new_calls=0,charts=rows,curved_actual_source=entry['source_id'],curved_generation_id=gid));print(dict(passed=True,charts=len(rows),curved_zero_anchor=True,new_calls=0),flush=True)
