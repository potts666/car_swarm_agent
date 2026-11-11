#!/usr/bin/env python3
"""Supplement the frozen auditor with independent seed-prefix and new-tail replay."""
import argparse
import json
from pathlib import Path
import numpy as np
from m2b_recording import lines
from verify_entry_bicycle import replay


def check(root):
 results={}
 for group in ('C-FIFO','L-FIFO','L-ALT'):
  for directory in sorted((root/group).glob('r[12]')):
   cfg=json.loads((directory/'config.json').read_text());s=cfg['scene'];n=round(s['h']/s['sample_s']);plans=lines(directory/'plans.jsonl');index={p['id']:p for p in plans};maximum=0.;count=0
   for p in plans:
    if p['kind']!='seed':continue
    z=np.array(p['states']);count+=1
    if p['shifted_from'] is None:
     assert p['context']['cycle']==0 and all(u[:2]==[0.,0.] for u in p['commands'])
     maximum=max(maximum,float(np.max(abs(z-np.array(p['report']['z'])))))
    else:
     source=index[p['shifted_from']];assert source['kind']=='final' and source['car']==p['car'] and source['context']['cycle']==p['context']['cycle']-1
     assert p['commands']==source['commands'][1:]+[[0.,0.,'SHIFT_TAIL']]
     maximum=max(maximum,float(np.max(abs(z[:-n]-np.array(source['states'][n:])))))
     tail=replay(np.array(source['states'][-1]),[(0.,0.)]*n,s['sample_s'],s['vehicle'])
     maximum=max(maximum,float(np.max(abs(z[-n:]-tail[1:]))))
   assert maximum<=1e-12,(str(directory),maximum)
   results[str(directory.relative_to(root))]=dict(seed_plans=count,maximum_prefix_or_tail_error=maximum,passed=True)
   print('seed replay passed',directory,maximum,flush=True)
 (root/'seed_replay_validation.json').write_text(json.dumps(results,indent=2)+'\n');return results

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();check(a.run)
