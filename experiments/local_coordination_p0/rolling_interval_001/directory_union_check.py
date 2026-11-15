"""Independent old/new occupancy-union lookup validates catalog-derived closure."""
from common import *
from plan_io import load,save
from discovery import Directory
from frames import static_clear

def run():
 reports=[]
 for name in ('E-anchor','E-MPC','recovery_missing_ACK','recovery_stale_K'):
  for e in read(HERE/f'results/{name}_summary.json')['events']:
   old,new=load(e['old_plan']),load(e['candidate']);frames=[read(HERE/old['frame_file']),read(HERE/new['frame_file'])];directory=Directory();rows=[];seen=set()
   for frame in frames:
    data=load(frame['directory'])
    for r in data['records']:
     if r['end']<e['cycle']:continue
     key=content_id(r)
     if key in seen:continue
     seen.add(key);r={**r,'box':list(map(F,r['box'])),'start':max(e['cycle'],r['start'])};rows.append(r)
   for r in rows:directory.register(**r)
   queries=[directory.query(r['car'],r['box'],r['start'],r['end']) for r in rows];assert all(directory.commit(q) for q in queries)
   changed={c for c in CARS if frames[0]['outer_slots'][c]!=frames[1]['outer_slots'][c]};seeds=set(changed)
   if not seeds:seeds={c for a,b in zip(old['owners'],new['owners']) if a!=b for c in (a,b) if c}
   discovered=set(seeds);discharged=[]
   if changed:
    peers={owner for r,q in zip(rows,queries) if r['car'] in changed for owner in q['owners']}
    for c in peers-changed:
     if old['per_car_policies'][c]['goal']<=e['cycle'] and all(static_clear(c,d)['passed'] for d in changed if d!=c):discharged.append(c)
     else:discovered.add(c)
   assert discovered==set(e['transaction']['participants']),(name,e['cycle'],discovered,e['transaction']['participants'])
   reports.append({'branch':name,'cycle':e['cycle'],'registered_records':len(rows),'queries':len(queries),'participants':sorted(discovered),'changed_slot_or_owner_seeds':sorted(seeds),'static_completed_peers_discharged':sorted(discharged),'permanent_goal_records':sum(r['kind']=='permanent_goal' for r in rows),'registered_all_before_query':True,'runtime_catalog_rule_equivalent_to_explicit_union_lookup':True})
 write(HERE/'results/directory_union_acceptance.json',{'passed':True,'checks':reports,'scope':'independent explicit old/new union replay; actual runtime used registered catalog slot/lease differences and live/completed interface closure','new_optimizer_calls':0});print('PASS old/new directory union',len(reports),'closures')
if __name__=='__main__':run()
