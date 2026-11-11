#!/usr/bin/env python3
"""Plot recorded actual states and effective owner; no prediction substitutes execution."""
import argparse
import json
from pathlib import Path
import numpy as np
from m2b_recording import lines
from m2b_scene import CARS
from bicycle_core import corners


def plot(root):
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 from matplotlib.patches import Rectangle,Polygon
 colors=dict(A1='#cf3d3d',A2='#d18524',B1='#236ab1',B2='#8657b0')
 fig,axes=plt.subplots(2,3,figsize=(16,6),gridspec_kw=dict(height_ratios=[2,1]))
 for col,group in enumerate(('C-FIFO','L-FIFO','L-ALT')):
  directory=root/group/'r1'
  if not (directory/'config.json').exists():continue
  scene=json.loads((directory/'config.json').read_text())['scene'];actual=lines(directory/'actual.jsonl');ax=axes[0,col]
  for x0,x1,y0,y1 in scene['road_rectangles']:ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,color='#ececec',zorder=0))
  for c in CARS:
   z=np.array([scene['initial'][c]]+[q for row in actual for q in row['states'][c][1:]])
   ax.plot(z[:,0],z[:,1],color=colors[c],lw=1.2,label=c);ax.scatter(*z[0,:2],s=22,marker='o',color=colors[c]);ax.add_patch(Polygon(corners([z[-1]],scene['vehicle'])[0],closed=True,fill=False,edgecolor=colors[c],lw=1.2))
  ax.set(xlim=(-53,53),ylim=(-6,6),xlabel='x (m)',ylabel='y (m; expanded scale)',title=group+' actual r1');ax.grid(alpha=.2);ax.legend(ncol=4,fontsize=8)
  owner_axis=axes[1,col];last=None;start=0
  for i,row in enumerate(actual+[dict(owner='END')]):
   owner=row['owner']
   if i==0:last=owner;continue
   if owner!=last:
    owner_axis.barh(0,(i-start)*scene['h'],left=start*scene['h'],height=.6,color=colors.get(last,'#bdbdbd'),edgecolor='none');last=owner;start=i
  owner_axis.set(xlabel='simulation time (s)',yticks=[],title='Effective owner (gray = idle)',xlim=(0,scene['timeout_s']))
 fig.suptitle('Four-car ideal synchronous simulation; actual r1 only, finite samples')
 fig.tight_layout();fig.savefig(root/'fourcar_trajectories.png',dpi=160);plt.close(fig)

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();plot(a.run)
