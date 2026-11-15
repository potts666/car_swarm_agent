"""Explicit sealed reference halfplanes for the union of the numerical windows."""
import itertools
from common import *
from plan_io import load,restore,save
from receivers import reference
from physics import predicted_nodes,body

def run():
 p=load('results/R_original.json.gz');refs=reference();out=[];minima={'road':F(100),'pair':F(100),'resource':F(100)};sub=16;reserve=TUBE+F(1,40)+F(15)*(H/sub)**2/8
 for k in range(32,65):
  s=restore(refs[k]);nodes=predicted_nodes(s,{c:p['controls'][c][k] for c in CARS},sub)
  for t in range(sub):
   bodies={c:body(nodes[c][t],c)+body(nodes[c][t+1],c) for c in CARS};roads=[];pairs=[];resource=[]
   for c,bs in bodies.items():
    box=m.axis_box(bs);rect=max(ROADS,key=lambda r:min(F(box[0],m.Q)-r[0],r[1]-F(box[1],m.Q),F(box[2],m.Q)-r[2],r[3]-F(box[3],m.Q)));margin=min(F(box[0],m.Q)-rect[0],rect[1]-F(box[1],m.Q),F(box[2],m.Q)-rect[2],rect[3]-F(box[3],m.Q))-reserve;assert margin>0;minima['road']=min(minima['road'],margin);roads.append({'car':c,'rectangle':rect,'margin':margin})
    if c!=p['owners'][k]:
     rr=max(-16-F(box[1],m.Q)-reserve,F(box[0],m.Q)-reserve-16);assert rr>0;minima['resource']=min(minima['resource'],rr);resource.append({'car':c,'margin':rr})
   for a,b in itertools.combinations(CARS,2):
    axes=[(F(1),F(0)),(F(0),F(1))]+m.normed_axes(nodes[a][t])+m.normed_axes(nodes[b][t]);best=None
    for x,y in axes:
     for sign in (-1,1):
      nx,ny=sign*x,sign*y;aa=[xx*nx+yy*ny for xx,yy in bodies[a]];bb=[xx*nx+yy*ny for xx,yy in bodies[b]];upper=max(z.u for z in aa);lower=min(z.l for z in bb);gap=F(lower-upper,m.Q)
      if best is None or gap>best['gap']:best={'a':a,'b':b,'n':[nx,ny],'beta':F(lower+upper,2*m.Q),'gap':gap}
    margin=best['gap']-2*reserve-SAFE;assert margin>0;best['margin']=margin;pairs.append(best);minima['pair']=min(minima['pair'],margin)
   out.append({'cycle':k,'subinterval':t,'road':roads,'pairs':pairs,'resource':resource,'owner':p['owners'][k]})
 save('results/common_faces.json.gz',out);write(HERE/'results/common_faces_certificate.json',{'passed':True,'intervals':len(out),'source':'R_original','window':[32,65],'subdivisions':sub,'each_body_reserve':reserve,'additional_domain':'1/40','minimum':minima,'all_arms_share_this_object':True,'full_tail_reference_proof':'results/reference_domain_audit.json','optimizer_calls':0});print('PASS common faces',len(out))
if __name__=='__main__':run()
