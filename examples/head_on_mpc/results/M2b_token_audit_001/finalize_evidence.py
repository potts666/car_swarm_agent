import collections,csv,datetime,gzip,hashlib,json,shutil
from pathlib import Path
A=Path(__file__).resolve().parent
ROOT='01a18b08-73cc-7001-ba3d-92d49ecdd80e'
def read(n):return json.loads((A/n).read_text())
def write(n,d):(A/n).write_text(json.dumps(d,indent=2,ensure_ascii=False)+'\n')
E=[json.loads(x) for x in (A/'event_metadata.jsonl').open()];C=read('task_tools.json');O=read('task_output_observations.json');summary=read('audit_summary.json');handles={};decoded=[]
def original(e):
 if e['path'] not in handles:handles[e['path']]=Path(e['path']).open('rb')
 f=handles[e['path']];f.seek(e['byte_offset']);return json.loads(f.readline())['payload']
def walk(value,fields):
 if isinstance(value,str):
  if value.startswith('data:image/'):return
  try:walk(json.loads(value),fields)
  except (ValueError,TypeError):pass
 elif isinstance(value,list):
  for x in value:walk(x,fields)
 elif isinstance(value,dict):
  if value.get('type') in ('input_image','image'):fields['images']+=1
  for k,v in value.items():
   if k in ('session_id','cell_id','exit_code','wall_time_seconds'):fields[k].append(v)
   if k=='output' and isinstance(v,str):fields['stdout_bytes']+=len(v.encode())
   if k=='image_url':continue
   walk(v,fields)
for o in O:
 p=original(o);fields=collections.defaultdict(list);fields['images']=0;fields['stdout_bytes']=0;walk(p.get('output'),fields)
 decoded.append(dict(line=o['line'],timestamp=o['timestamp'],call_id=o['call_id'],raw_output_bytes=o['output_bytes'],stdout_bytes=fields['stdout_bytes'],image_blocks=fields['images'],session_ids=fields['session_id'],exit_codes=fields['exit_code'],wall_time_seconds=fields['wall_time_seconds']))
for f in handles.values():f.close()
write('decoded_tool_output_metrics.json',decoded)
summary['image_blocks']=sum(x['image_blocks'] for x in decoded);summary['decoded_terminal_stdout_bytes']=sum(x['stdout_bytes'] for x in decoded)
for x in summary['largest_outputs']:
 d=next(z for z in decoded if z['line']==x['line']);x['contains_image_payload']=bool(d['image_blocks']);x['decoded_terminal_stdout_bytes']=d['stdout_bytes']
readcounts=collections.Counter()
for c in C:
 for line in c['shell_commands']:
  if not __import__('re').match(r'^(cat|head|tail|sed|rg)\b',line):continue
  if __import__('re').match(r'^cat\s*[><]|^sed\s+-i',line):continue
  for p in __import__('re').findall(r'(?:examples/head_on_mpc/|/home/sunflower/)[^\s;\"\']+',line):readcounts[p]+=1
summary['repeated_read_paths']=[dict(path=p,count=n) for p,n in readcounts.most_common(12)]
# stdout-empty is not the same as a duplicate model response; keep exact observations.
out_by={x['call_id']:x for x in decoded};polls=[c for c in C if c.get('sessions_waited')]
summary['pure_poll_empty_stdout']=[dict(line=c['line'],timestamp=c['timestamp']) for c in polls if c['pure_wait'] and out_by.get(c['call_id'],{}).get('stdout_bytes')==0]
summary['context_window_recorded']=475000;summary['displayed_context_percentage']=None
write('audit_summary.json',summary)
# A long-lived process can span many agent observations. These are observations, not exact CPU boundaries.
observations=[]
for sid in ('40671','95377','68141','39892','67621'):
 calls=[c for c in C if sid in c.get('sessions_waited',[])];starts=[d for d in decoded if any(str(v)==sid for v in d['session_ids'])]
 observations.append(dict(exec_session_id=sid,first_session_id_observed=starts[0]['timestamp'] if starts else None,first_observation_line=starts[0]['line'] if starts else None,write_stdin_model_envelopes=len(calls),observation_lines=[c['line'] for c in calls],last_return_observed=max((d['timestamp'] for d in decoded if d['call_id'] in {c['call_id'] for c in calls}),default=None),last_known_exit_codes=[d['exit_codes'] for d in decoded if d['call_id'] in {c['call_id'] for c in calls}][-1:] ))
write('long_command_observations.json',observations)
# Same-version schema extraction is kept; discard only this audit's unrelated generated schemas.
if (A/'protocol_schema').exists():shutil.rmtree(A/'protocol_schema')
print(json.dumps(dict(stdout_bytes=summary['decoded_terminal_stdout_bytes'],image_blocks=summary['image_blocks'],empty_pure_polls=len(summary['pure_poll_empty_stdout']),long_commands=observations[:2],read_paths=summary['repeated_read_paths'][:4]),ensure_ascii=False))
