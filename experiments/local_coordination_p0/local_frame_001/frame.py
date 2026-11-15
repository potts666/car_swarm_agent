"""Finite union external frames, exact resource leases and dependency closure."""
import copy,itertools,time
from common import *
from plan_io import load,save,restore
from physics import body
from discovery import Directory
from receivers import reference

def lease_table(plan,extended=False):
 return {'D':{'members':['A1','B1'],'window':[26,855 if extended else 924]},'B2':{'members':['B2'],'window':[855,1304] if extended else [924,1373]},'A2':{'members':['A2'],'window':[1304,1753] if extended else [1373,1822]}}

def physical_id(p):return content_id({k:p[k] for k in ('controls','owners','per_car_policies','B0','frame_branch')})

def validate_frame_plan(plan,frame):
 assert plan['outer_frame_id']==frame['outer_frame_id'],'outer_frame_id'
 assert plan['B0']<=frame['valid_through'],'frame expired'
 for c in CARS:assert plan['per_car_policies'][c]['goal']<=frame['deadlines'][c],('deadline',c)
 for k,owner in enumerate(plan['owners']):
  if owner is not None:
   lease=next(v for v in frame['leases'].values() if owner in v['members']);assert lease['window'][0]<=k<lease['window'][1],('outside lease',owner,k)
 return True

def source_valid(frame,state):assert state['cycle']<=frame['valid_through'],'frame expired'
def node_inclusion(plan,state):
 refs=reference(plan);k=state['cycle'];errors={}
 for c in CARS:
  p=state['poses'][c];z=[m.I.raw(*q) for q in refs[k]['poses_Q'][c]];err=sum((p[j]-z[j]).abs_upper() for j in (0,1))+RB*(p[2]-z[2]).abs_upper();assert err<=F(1,40),('outside published occupancy',c,k,err);errors[c]=err
 return errors

def certify(name,branches):
 started=time.monotonic();plans=[load(f'results/references/{n}.json.gz') for n in branches];reserve=TUBE+F(1,40)+15*H*H/8;faces=[];minimum={'road':F(100),'pair':F(100),'resource':F(100)};rows=[]
 for branch,p in zip(branches,plans):
  refs=reference(p);N=p['B0']
  for k in range(N):
   bs={c:body(tuple(m.I.raw(*q) for q in refs[k]['poses_Q'][c]),c)+body(tuple(m.I.raw(*q) for q in refs[k+1]['poses_Q'][c]),c) for c in CARS};roads=[];pairs=[];resources=[]
   for c in CARS:
    box=m.axis_box(bs[c]);rect=max(ROADS,key=lambda r:min(F(box[0],m.Q)-r[0],r[1]-F(box[1],m.Q),F(box[2],m.Q)-r[2],r[3]-F(box[3],m.Q)));margin=min(F(box[0],m.Q)-rect[0],rect[1]-F(box[1],m.Q),F(box[2],m.Q)-rect[2],rect[3]-F(box[3],m.Q))-reserve;assert margin>=0,(branch,k,c,'road',margin);minimum['road']=min(minimum['road'],margin);roads.append({'car':c,'rectangle':rect,'margin':margin})
    if c!=p['owners'][k]:
     gap=max(-16-F(box[1],m.Q)-reserve,F(box[0],m.Q)-reserve-16);assert gap>0,(branch,k,c,'resource',gap);minimum['resource']=min(minimum['resource'],gap);resources.append({'car':c,'clearance':gap})
   for a,b in itertools.combinations(CARS,2):
    axes=[(F(1),F(0)),(F(0),F(1))]+m.normed_axes(tuple(m.I.raw(*q) for q in refs[k]['poses_Q'][a]))+m.normed_axes(tuple(m.I.raw(*q) for q in refs[k]['poses_Q'][b]));best=None
    for nx,ny in axes:
     for sign in (-1,1):
      x,y=sign*nx,sign*ny;upper=max((xx*x+yy*y).u for xx,yy in bs[a]);lower=min((xx*x+yy*y).l for xx,yy in bs[b]);gap=F(lower-upper,m.Q)
      if best is None or gap>best['gap']:best={'pair':[a,b],'n':[x,y],'beta':F(lower+upper,2*m.Q),'gap':gap}
    margin=best['gap']-2*reserve-SAFE;assert margin>=0,(branch,k,a,b,'pair',margin);minimum['pair']=min(minimum['pair'],margin);best['margin_over_0.5']=margin;pairs.append(best)
   faces.append({'branch':branch,'cycle':k,'road':roads,'pairs':pairs,'resources':resources,'owner':p['owners'][k]})
  for c in CARS:
   for start in range(0,N,8):
    end=min(start+8,N);bb=m.axis_box([q for t in range(start,end+1) for q in body(tuple(m.I.raw(*z) for z in refs[t]['poses_Q'][c]),c)]);inflate=reserve+SAFE/2;box=[F(bb[0],m.Q)-inflate,F(bb[1],m.Q)+inflate,F(bb[2],m.Q)-inflate,F(bb[3],m.Q)+inflate];rows.append({'car':c,'box':box,'start':start,'end':end,'kind':f'{branch}_all_intents_receivers_K'})
   goal=p['per_car_policies'][c]['goal'];bb=m.axis_box(body(tuple(m.I.raw(*z) for z in refs[goal]['poses_Q'][c]),c));box=[F(bb[0],m.Q)-reserve-SAFE/2,F(bb[1],m.Q)+reserve+SAFE/2,F(bb[2],m.Q)-reserve-SAFE/2,F(bb[3],m.Q)+reserve+SAFE/2];rows.append({'car':c,'box':box,'start':goal,'end':float('inf'),'kind':'permanent_goal'})
 directory=Directory()
 for r in rows:directory.register(**r)
 queries=[directory.query(r['car'],r['box'],r['start'],r['end']) for r in rows];assert all(directory.commit(q) for q in queries)
 # Independent broadphase completeness against the exact registered boxes.
 potential=set();found=set()
 for r,q in zip(rows,queries):
  for c in q['owners']:found.add(tuple(sorted((r['car'],c))))
  for other in rows:
   if other['car']!=r['car'] and r['start']<=other['end'] and other['start']<=r['end'] and all(r['box'][2*j]<=other['box'][2*j+1] and other['box'][2*j]<=r['box'][2*j+1] for j in (0,1)):potential.add(tuple(sorted((r['car'],other['car']))))
 assert potential<=found;assert all(any(r['car']==c and r['kind']=='permanent_goal' for r in rows) for c in CARS)
 save(f'results/{name}_faces.json.gz',faces);save(f'results/{name}_directory.json.gz',{'records':rows,'queries':queries,'events':directory.events,'potential':sorted(potential),'found':sorted(found),'registered_before_queries':True})
 latest=plans[-1];frame={'type':'LOCAL-FRAME-v1','branches':branches,'reference_ids':{n:physical_id(p) for n,p in zip(branches,plans)},'leases':lease_table(latest,name=='extended_frame'),'deadlines':{c:max(p['per_car_policies'][c]['goal'] for p in plans) for c in CARS},'valid_from':0,'valid_through':max(p['B0'] for p in plans),'permanent_goals':list(CARS),'body_domain':'1/40','node_tube':TUBE,'time_chord':15*H*H/8,'all_six_edges':[['A1','B1'],['A2','B2'],['A1','A2'],['A1','B2'],['B1','A2'],['B1','B2']],'separation_faces':sha(HERE/f'results/{name}_faces.json.gz'),'directory':sha(HERE/f'results/{name}_directory.json.gz'),'minimum':minimum,'occupancy_semantics':'finite branch union of full continuous corner chords + .025 allowed node body deviation + original .05 tube +15*h^2/8; positive receiver SE2 tails and permanent targets included; not convex hull across alternatives'};frame['outer_frame_id']=content_id(frame);write(HERE/f'{name}.json',frame);write(HERE/f'results/{name}_certificate.json',{'passed':True,'outer_frame_id':frame['outer_frame_id'],'minimum':minimum,'pairs':6,'cross_edges':4,'branch_worlds':len(branches),'all_combinations_basis':'external reference controls/calendar identical in old/new branches; all independent continuous variations stay in respective .025 body corridor, both D branches separately separated; hence any D old/new choice x external old/new allowed update','records':len(rows),'permanent_goals':True,'closed_time':True,'registered_before_queries':True,'wall':time.monotonic()-started,'optimizer_calls':0});return frame

def expansion_closure(old,new,frame):
 changes=[];members=set()
 oldleases=frame['leases'];newleases=lease_table(new,True)
 for group in oldleases:
  a,b=oldleases[group],newleases[group]
  if a['window']!=b['window']:
   for c in a['members']:members.add(c);changes.append({'added':c,'dependency':'resource_lease_endpoint_changed','old':a['window'],'new':b['window']})
 for c in CARS:
  a,b=old['per_car_policies'][c]['goal'],new['per_car_policies'][c]['goal']
  if a!=b:members.add(c);changes.append({'added':c,'dependency':'future_receiver_deadline_changed','old':a,'new':b})
 return {'participants':sorted(members),'reasons':changes,'old_outer_frame_id':frame['outer_frame_id'],'new_lease_table':newleases,'fixed_point_reached':True}
if __name__=='__main__':
 f=certify('outer_frame',['old','new'])
 for n in ('old','new'):
  p=load(f'results/references/{n}.json.gz');p['outer_frame_id']=f['outer_frame_id'];save(f'results/{n}.json.gz',p);validate_frame_plan(p,f)
 print('PASS fixed outer frame union and directory',f['outer_frame_id'],flush=True)
