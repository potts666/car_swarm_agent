"""Common affine moment lattice; weighted steering nullspace, exact E T=0."""
from common import *

def kernel(free,weights,frozen=False):
 E=[];names=[];n=free
 def add(row,name):E.append(list(map(F,row)));names.append(name)
 for j in range(32):
  for component in (0,1):
   if j>=n or (component==1 and frozen):
    row=[F(0)]*64;row[2*j+component]=1;add(row,f'fixed_{j}_{component}')
 for component,power,name in ((0,0,'terminal_v'),(0,1,'acceleration_time_moment'),(1,0,'terminal_delta')):
  row=[F(0)]*64
  for j in range(n):row[2*j+component]=F(j**power)
  add(row,name)
 row=[F(0)]*64;normal=max(map(abs,weights))
 for j,w in enumerate(weights):row[2*j+1]=w/normal
 add(row,'weighted_steering_heading_moment')
 T=[];blocks=[]
 for component in (0,1):
  if component==1 and frozen:continue
  start=len(T)
  for j in range(n-2):
   if component==0:values=[F(1),F(-2),F(1)]
   else:
    a=weights[j]-weights[j+1];b=weights[j+1]-weights[j+2];assert min(a,b)>0,'weighted integral is not monotone'
    values=[2*b/(a+b),F(-2),2*a/(a+b)]
   column=[F(0)]*64
   for t,x in enumerate(values):column[2*(j+t)+component]=x
   T.append(column)
  blocks.append({'component':component,'start':start,'end':len(T)})
 assert T and all(sum(a*b for a,b in zip(row,col))==0 for row in E for col in T)
 error=[sum(abs(col[j]) for col in T)/2/F(10**12) for j in range(64)]
 return {'E':E,'equality_names':names,'T':T,'blocks':blocks,'rank_free_dimension':len(T),'component_grid_error':error,'max_grid_error':max(error),'Delta':F(1,10**12)}

def quantize(raw,alpha,lattice):
 cols=[[F(x) for x in c] for c in lattice['T']];coords=[]
 for block in lattice['blocks']:
  component=block['component'];start=block['start'];stop=block['end'];z=[]
  for j,t in enumerate(range(start,stop)):
   value=F(alpha)*F(str(raw[2*j+component]))
   for prev in range(max(0,j-2),j):value-=cols[start+prev][2*j+component]*z[prev]
   z.append(value/cols[t][2*j+component])
  coords.extend(z)
 ints=[round(z*10**12) for z in coords];delta=[sum(col[j]*F(n,10**12) for col,n in zip(cols,ints)) for j in range(64)]
 assert all(sum(F(x)*y for x,y in zip(row,delta))==0 for row in lattice['E'])
 projected=[sum(c[j]*z for c,z in zip(cols,coords)) for j in range(64)];err=max(abs(x-y) for x,y in zip(delta,projected));assert err<=F(lattice['max_grid_error'])
 return delta,{'integer_coordinates':ints,'coordinate_values':coords,'grid_error':err,'delta':delta,'exact_equalities':True}
