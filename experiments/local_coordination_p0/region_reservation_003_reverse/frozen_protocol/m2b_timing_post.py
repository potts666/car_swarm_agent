#!/usr/bin/env python3
"""Observed consecutive cycle starts include diagnostic preparation after commit."""
import argparse
import json
import csv
from pathlib import Path
import numpy as np
from m2b_recording import lines


def stats(values):
 a=np.asarray(values,float);return dict(count=len(a),p50=float(np.percentile(a,50)),p95=float(np.percentile(a,95)),p99=float(np.percentile(a,99)),max=float(a.max()),total=float(a.sum()))

def summarize(root):
 result={}
 for group in ('C-FIFO','L-FIFO','L-ALT'):
  for directory in sorted((root/group).glob('r[12]')):
   rows=lines(directory/'planning.jsonl');summary=json.loads((directory/'summary.json').read_text());period=[];post=[]
   for a,b in zip(rows[:-1],rows[1:]):
    assert a['committed'] and b['cycle']==a['cycle']+1
    period.append(b['start_monotonic_s']-a['start_monotonic_s']);post.append(b['start_monotonic_s']-a['end_monotonic_s'])
   assert len(period)==summary['committed_cycles'] and rows[-1]['outcome']=='completed'
   result[str(directory.relative_to(root))]=dict(committed_cycle_start_intervals_s=stats(period),post_commit_bookkeeping_s=stats(post),intervals_over_200ms=sum(x>.2 for x in period),final_completion_exit_logged_s=rows[-1]['total_cycle_wall_s'],scope='consecutive starts include compact records, diagnostic preparation and progress output; final no-input completion exit is separate')
 (root/'timing_scope_supplement.json').write_text(json.dumps(result,indent=2)+'\n')
 if (root/'metrics.json').exists():
  metrics=json.loads((root/'metrics.json').read_text())
  for name,m in metrics.items():
   period=result[name]['committed_cycle_start_intervals_s'];m.setdefault('logged_commit_end_cycle_s',m['cycle_s']);m['cycle_s']=period;m['complete_cycle_interval_s']=period;m['post_commit_bookkeeping_s']=result[name]['post_commit_bookkeeping_s'];m['cycle_timing_scope']=result[name]['scope'];m['cycles_over_200ms']=result[name]['intervals_over_200ms']
  (root/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
  if (root/'comparison.csv').exists():
   with (root/'comparison.csv').open() as stream:reader=csv.DictReader(stream);fields=reader.fieldnames;rows=list(reader)
   extra=[f'logged_commit_end_{q}_ms' for q in ('p50','p95','p99','max')]
   for row in rows:
    for q,key in zip(('p50','p95','p99','max'),extra):
     row.setdefault(key,row[f'cycle_{q}_ms']);row[f'cycle_{q}_ms']=str(metrics[row['run']]['cycle_s'][q]*1000)
   with (root/'comparison.csv').open('w',newline='') as stream:writer=csv.DictWriter(stream,fieldnames=fields+[q for q in extra if q not in fields]);writer.writeheader();writer.writerows(rows)
  if (root/'report.md').exists():
   text=(root/'report.md').read_text();out=[]
   for line in text.splitlines():
    parts=line.split('|')
    if len(parts)>2 and parts[1] in metrics:
     m=metrics[parts[1]]
     if len(parts)==10:parts[5]=f"{m['cycle_s']['max']*1000:.1f}";line='|'.join(parts)
     elif len(parts)==8:
      parts[2:7]=[f"{m['cycle_s'][q]*1000:.1f}" for q in ('p50','p95','p99','max')]+[str(m['cycles_over_200ms'])];line='|'.join(parts)
    out.append(line)
   note='完整周期口径：comparison.csv及汇总cycle_s采用相邻周期实际开始时刻之差，覆盖提交后的精简记录、关键诊断整理和进度输出；原开始至提交结束计时保留为logged_commit_end字段。最后一个无输入完成出口单列，详见timing_scope_supplement.json。'
   if note not in text:out+=['',note]
   (root/'report.md').write_text('\n'.join(out)+'\n')
 return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();print(json.dumps(summarize(a.run)))
