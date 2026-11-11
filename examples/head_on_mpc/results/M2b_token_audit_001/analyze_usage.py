import ast,collections,csv,datetime,hashlib,json,re,statistics
from pathlib import Path
OUT=Path(__file__).resolve().parent
ROOT='01a18b08-73cc-7001-ba3d-92d49ecdd80e'
TURNS={'01a19760-00c8-74f2-a1ce-764b0e651cc0','01a1979c-a4e1-7493-b137-0ce18a0d1fc2','01a19972-5fcc-71a2-bf8a-f076f9d4d354'}
START='2026-11-01T13:50:13.786Z';END='2026-11-01T23:46:00.619Z'
def write(name,data): (OUT/name).write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')
def dt(t):return datetime.datetime.fromisoformat(t.replace('Z','+00:00'))
E=[json.loads(s) for s in (OUT/'event_metadata.jsonl').open()];U=[json.loads(s) for s in (OUT/'token_usage_detail.jsonl').open()]
handles={}
def original(e):
 f=handles.setdefault(e['path'],Path(e['path']).open('rb')) if e['path'] not in handles else handles[e['path']];f.seek(e['byte_offset']);return json.loads(f.readline())['payload']
def texts(v):
 if isinstance(v,str):
  try:return texts(json.loads(v))
  except (ValueError,TypeError):return [v]
 if isinstance(v,dict):
  return [t for k,x in v.items() if k in ('content','text','output') for t in texts(x)]
 if isinstance(v,list):return [t for x in v for t in texts(x)]
 return []
C=[];O=[];markers=[]
for e in E:
 if e['session_id']!=ROOT or not START<=e.get('timestamp','')<=END or e['type']!='response_item':continue
 if e['payload_type'] in ('function_call','custom_tool_call'):
  p=original(e);raw=p.get('arguments',p.get('input',''));commands=[]
  if p.get('name')=='exec_command':
   try:commands=[json.loads(raw)['cmd']]
   except:pass
  else:
   for match in re.finditer(r'''\bcmd\s*:\s*("(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*')''',raw):
    try:commands.append(json.loads(match[1]) if match[1].startswith('"') else ast.literal_eval(match[1]))
    except:pass
  shell=[];writing=[];pathreads=[]
  for cmd in commands:
   heredoc=None
   for line in cmd.splitlines():
    x=line.strip()
    if heredoc:
     if x==heredoc:heredoc=None
     continue
    here=re.search(r"<<\s*['\"]?([A-Za-z_][A-Za-z0-9_]*)",x)
    if here:heredoc=here[1]
    if x.startswith(('python3 ','source ','export ','cat ','rg ','tail ','head ','ls ','du ','find ','ps ','git ','mkdir ','cp ','tar ')):shell.append(x[:400])
    if re.search(r'\b(cat|tee)\s*>|\bsed\s+-i',x):writing.append(x[:150])
    if re.match(r'^(cat|head|tail|sed|rg)\b',x):pathreads+=re.findall(r'(?:examples/head_on_mpc/|/home/sunflower/)[^\s;\"\']+',x)
  try:args=json.loads(raw)
  except:args={}
  sleeps=[int(x) for x in e.get('sleep_ms',[])]
  if isinstance(args,dict) and 'duration_ms' in args:sleeps.append(args['duration_ms'])
  item={**e,'shell_commands':shell,'source_writes':writing,'read_paths':pathreads,'sleep_duration_ms':sleeps,'has_escalation':'require_escalated' in raw,'call_text_bytes':len(raw.encode()),'pure_wait':bool('write_stdin' in raw and not commands) or p.get('name') in ('sleep','clock.sleep','wait'),'file_modification':bool(writing or '.write_text(' in raw or 'apply_patch' in raw)}
  C.append(item)
  for x in shell:
   if x.startswith(('python3 ','OPENBLAS_NUM_THREADS=')) and any(k in x for k in ('run_m2b.py','run_m2b_study.py','verify_m2b.py','finalize_m2b','package_m2b.py')):markers.append(dict(line=e['line'],timestamp=e['timestamp'],command=x))
 elif e['payload_type'] in ('function_call_output','custom_tool_call_output'):
  p=original(e);body='\n'.join(texts(p.get('output','')));obs=[]
  for line in body.splitlines():
   if line.startswith(('START ','POST ','r1 cycle ','r2 cycle ','formal audits ','storage measured ','diagnostic bytes ','existing archive ','full diagnostic restore ','M2b FINAL')):obs.append(line[:350])
  item={**e,'observations':obs,'progress_cycles':[list(m) for m in re.findall(r'(r[12]) cycle (\d+) sim_s ([0-9.]+)',body)],'returned_session_ids':re.findall(r'(?:session_id[\"\']?\s*:\s*|session ID\s+)(\d+)',body),'no_new_output':not body.strip() or bool(re.search(r'Output:\s*"?\s*$',body)),'failed':bool(e.get('exit_codes') and any(x!='0' for x in e['exit_codes']))}
  O.append(item)
for f in handles.values():f.close()
write('task_tools.json',C);write('task_output_observations.json',O);write('program_launch_markers.json',markers)
print('LAUNCH MARKERS')
for m in markers:print(m['line'],m['timestamp'],m['command'][-180:])
# Deduplicate both historical record IDs and cumulative snapshots, checking every effective increment.
F=('input_tokens','cached_input_tokens','cache_write_input_tokens','output_tokens','reasoning_output_tokens','total_tokens')
last={};seen=set();selected=[];resets=[];mismatches=[];duplicates=[]
for r in U:
 sid=r['thread_id'];key=(sid,r['response_id_hash']);v=r['thread_token_usage'];old=last.get(sid,{k:0 for k in F});delta={k:v[k]-old[k] for k in F}
 if key in seen:duplicates.append(dict(thread_id=sid,line=r['line']));continue
 seen.add(key)
 if any(x<0 for x in delta.values()):resets.append(dict(thread_id=sid,line=r['line']));delta=r['usage']
 if delta!=r['usage']:mismatches.append(dict(thread_id=sid,line=r['line'],delta=delta,reported=r['usage']))
 last[sid]=v
 if r['root_turn_id'] in TURNS:selected.append({**r,'effective_delta':delta})
write('task_usage_increments.json',selected)
checks=dict(task_start_utc=START,task_end_utc=END,root_thread_id=ROOT,root_turn_ids=sorted(TURNS),task_records=len(selected),duplicate_response_records=duplicates,cumulative_resets=resets,counter_delta_mismatches=mismatches,total_identity_passed=all(x['usage']['input_tokens']+x['usage']['output_tokens']==x['usage']['total_tokens'] for x in U),cached_subset_inequality_passed=all(x['usage']['cached_input_tokens']<=x['usage']['input_tokens'] for x in U),reasoning_subset_inequality_passed=all(x['usage']['reasoning_output_tokens']<=x['usage']['output_tokens'] for x in U),sum_effective_delta={k:sum(x['effective_delta'][k] for x in selected) for k in F})
write('usage_consistency.json',checks)
print('CONSISTENCY',json.dumps(checks))
print('TOOLS',len(C),'outputs',len(O),'sleeps',sum(bool(x['sleep_duration_ms']) for x in C),'sleep_seconds',sum(sum(x['sleep_duration_ms']) for x in C)/1000,'pure_wait',sum(x['pure_wait'] for x in C),'escalating_tool_envelopes',sum(x['has_escalation'] for x in C),'output_bytes',sum(x.get('output_bytes',0) for x in O))
print('LARGEST OUTPUTS',[(x['line'],x['timestamp'],x.get('output_bytes'),x.get('output_tags')) for x in sorted(O,key=lambda x:x.get('output_bytes',0),reverse=True)[:5]])
