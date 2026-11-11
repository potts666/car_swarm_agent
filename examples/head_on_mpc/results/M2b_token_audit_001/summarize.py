import ast,collections,csv,datetime,hashlib,json,statistics
from pathlib import Path
A=Path(__file__).resolve().parent;R=Path('examples/head_on_mpc/results/M2b_fourcar_001');ROOT='01a18b08-73cc-7001-ba3d-92d49ecdd80e'
def read(name):return json.loads((A/name).read_text())
def write(name,d):(A/name).write_text(json.dumps(d,indent=2,ensure_ascii=False)+'\n')
U=read('task_usage_increments.json');C=read('task_tools.json');O=read('task_output_observations.json');F=('input_tokens','cached_input_tokens','cache_write_input_tokens','output_tokens','reasoning_output_tokens','total_tokens')
STAGES=['编码/场景门禁','调试修复/回归','正式运行/监看等待','离线验收观察','记录整理/归档工具开发','归档恢复/交付核查','打包/最终交付']
audit_windows=[('2026-11-01T15:30:08.825Z','2026-11-01T15:33:20.903Z'),('2026-11-01T15:53:19.969Z','2026-11-01T15:56:39.019Z'),('2026-11-01T16:17:34.277Z','2026-11-01T16:21:39.955Z')]
def temporal(t):
 if t<'2026-11-01T14:00:17.473Z':return STAGES[0]
 if t<'2026-11-01T15:08:07.040Z':return STAGES[1]
 if t<'2026-11-01T16:21:39.955Z':return STAGES[3] if any(a<=t<b for a,b in audit_windows) else STAGES[2]
 if t<'2026-11-01T23:29:32.115Z':return STAGES[4]
 if t<'2026-11-01T23:44:02.806Z':return STAGES[5]
 return STAGES[6]
# Root records link to the single tool envelope emitted since its preceding usage record.
previous=0;root_association={}
for u in U:
 if u['thread_id']!=ROOT:continue
 calls=[c for c in C if previous<c['line']<u['line']];previous=u['line'];stage=temporal(u['timestamp']);association='time_window'
 if len(calls)==1:
  c=calls[0];association='root_response_to_preceding_tool_envelope';root_association[u['line']]=c
  if '2026-11-01T15:08:07.040Z'<=u['timestamp']<'2026-11-01T16:21:39.955Z' and c['file_modification']:stage=STAGES[4]
 u['stage']=stage;u['attribution']=association
# Guardian stage uses matching root turn and the interval of a parent tool envelope when unique.
by_call={c.get('call_id'):c for c in C};tool_intervals=[]
for o in O:
 c=by_call.get(o.get('call_id'))
 if c:tool_intervals.append((c,o))
for u in U:
 if u['thread_id']==ROOT:continue
 matches=[(c,o) for c,o in tool_intervals if c['timestamp']<=u['timestamp']<=o['timestamp']]
 stage=temporal(u['timestamp'])
 if len(matches)==1:
  c,o=matches[0]
  if '2026-11-01T15:08:07.040Z'<=u['timestamp']<'2026-11-01T16:21:39.955Z' and c['file_modification']:stage=STAGES[4]
  u['parent_tool_line_inferred']=c['line'];u['attribution']='root-turn exact; parent-tool time interval inferred'
 else:u['attribution']='root-turn exact; stage time-window inferred'
 u['stage']=stage
write('task_usage_attributed.json',U)
program=[]
for g in ('C-FIFO','L-FIFO','L-ALT'):
 for p in sorted((R/g).glob('r[12]/summary.json')):
  d=json.loads(p.read_text());v=json.loads((p.parent/'independent_validation.json').read_text());program.append(dict(run=str(p.parent.relative_to(R)),online_wall_s=d['online_wall_s'],export_wall_s=d['export_wall_s'],audit_wall_s=v['independent_audit_wall_s']))
debug=[]
for g in ('debug_v1','debug_v2','debug_v3','debug_c1'):
 p=R/g/'r1/summary.json';d=json.loads(p.read_text());debug.append(dict(run=g,online_wall_s=d['online_wall_s'],failure=d['failure'],export_status='incomplete known prototype export; not assigned a duration'))
program_sums={k:sum(x[k] for x in program) for k in ('online_wall_s','export_wall_s','audit_wall_s')};storage=json.loads((R/'storage_measurements.json').read_text())
summary=[]
for stage in STAGES:
 rows=[u for u in U if u['stage']==stage];root=[u for u in rows if u['thread_id']==ROOT];guard=[u for u in rows if u['thread_id']!=ROOT]
 summary.append(dict(stage=stage,usage_records=len(rows),root_records=len(root),guardian_records=len(guard),**{k:sum(x['effective_delta'][k] for x in rows) for k in F},program_wall_s=(sum(x['online_wall_s'] for x in debug) if stage==STAGES[1] else program_sums['online_wall_s'] if stage==STAGES[2] else program_sums['audit_wall_s'] if stage==STAGES[3] else program_sums['export_wall_s']+storage['offline_measurement_wall_s'] if stage==STAGES[4] else None),wall_scope=('debug Σ, overlapping; not elapsed time' if stage==STAGES[1] else 'six formal online sums' if stage==STAGES[2] else 'six formal independent audit sums' if stage==STAGES[3] else 'formal exports + one measured storage pass only; other work unmeasured' if stage==STAGES[4] else 'not independently measured'),attribution='tokens measured; semantic stage attributed by operation/observation window, not exact causal allocation'))
write('stage_summary.json',summary)
with (A/'stage_summary.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(summary[0]));w.writeheader();w.writerows(summary)
with (A/'usage_increments.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=['thread_id','root_turn_id','timestamp','path','line','stage','attribution',*F]);w.writeheader()
 for u in U:w.writerow({**{k:u.get(k) for k in ('thread_id','root_turn_id','timestamp','path','line','stage','attribution')},**u['effective_delta']})
read_counts=collections.Counter(p for c in C for p in c['read_paths']);polls=[c for c in C if c.get('sessions_waited')];pure_poll=[c for c in polls if c['pure_wait']];out_by={o.get('call_id'):o for o in O}
no_progress=[c for c in pure_poll if not out_by.get(c.get('call_id'),{}).get('progress_cycles')]
body_types={};samples=[];handles={}
for o in sorted(O,key=lambda x:x.get('output_bytes',0),reverse=True)[:5]:
 with Path(o['path']).open('rb') as f:f.seek(o['byte_offset']);p=json.loads(f.readline())['payload']
 raw=p.get('output','');image=isinstance(raw,str) and ('data:image/' in raw or '"image"' in raw and 'base64' in raw)
 samples.append(dict(line=o['line'],timestamp=o['timestamp'],raw_serialized_output_bytes=o['output_bytes'],contains_image_payload=image,evidence=f"{o['path']}:{o['line']}"))
# Static source audit, only frozen runtime files (no experiment logs read).
freeze=json.loads((R/'formal_freeze.json').read_text());code=[];network=[]
for name,expected in freeze['source_sha256'].items():
 p=Path('examples/head_on_mpc/scripts')/name;text=p.read_text();tree=ast.parse(text);imports=[];matches=[]
 for node in ast.walk(tree):
  if isinstance(node,ast.Import):imports.extend(x.name for x in node.names)
  elif isinstance(node,ast.ImportFrom):imports.append(node.module)
 for n,line in enumerate(text.splitlines(),1):
  if any(k in line for k in ('osqp.OSQP','solver.solve','create_publisher','subprocess.Popen','subprocess.run','def predict_joint')):matches.append(dict(line=n,text=line.strip()[:180]))
 bad=[x for x in imports if x and x.split('.')[0] in ('openai','anthropic','httpx','requests','aiohttp','urllib','litellm')]
 network.extend([dict(file=name,imports=bad)] if bad else [])
 code.append(dict(file=str(p),sha256=hashlib.sha256(text.encode()).hexdigest(),matches_frozen=hashlib.sha256(text.encode()).hexdigest()==expected,imports=sorted(set(x for x in imports if x)),selected_call_evidence=matches))
write('program_code_audit.json',dict(files=code,llm_or_http_import_matches=network,scope='all 17 runtime files identified by formal_freeze; no dynamic network trace; generated shell dependency download is separate'))
rootU=[u for u in U if u['thread_id']==ROOT];guardU=[u for u in U if u['thread_id']!=ROOT]
rootwait=[u for u in rootU if root_association.get(u['line'],{}).get('pure_wait')]
aggregate=dict(usage_records=len(U),root_usage_records=len(rootU),guardian_usage_records=len(guardU),totals={k:sum(u['effective_delta'][k] for u in U) for k in F},root_totals={k:sum(u['effective_delta'][k] for u in rootU) for k in F},guardian_totals={k:sum(u['effective_delta'][k] for u in guardU) for k in F},root_input_per_response=dict(mean=statistics.mean(u['usage']['input_tokens'] for u in rootU),min=min(u['usage']['input_tokens'] for u in rootU),max=max(u['usage']['input_tokens'] for u in rootU)),root_pure_wait_records=len(rootwait),root_pure_wait_tokens=sum(u['usage']['total_tokens'] for u in rootwait),tool_envelopes=len(C),returned_tool_outputs=len(O),sleep_calls=sum(bool(c['sleep_duration_ms']) for c in C),sleep_seconds=sum(sum(c['sleep_duration_ms']) for c in C)/1000,write_stdin_envelopes=len(polls),pure_write_stdin_envelopes=len(pure_poll),pure_write_stdin_without_new_cycle_line=len(no_progress),raw_tool_output_bytes=sum(o['output_bytes'] for o in O),largest_outputs=samples,repeated_read_paths=[dict(path=p,count=n) for p,n in read_counts.most_common(12)],source_escalation_envelopes=sum(c['has_escalation'] for c in C),spawn_or_coordination_tool_names=[c['name'] for c in C if c['name'] in ('spawn_agent','followup_task','send_message')],program_reported_sums=program_sums,program_runs=program,debug_runs=debug)
write('audit_summary.json',aggregate)
print('STAGES',[(x['stage'],x['usage_records'],x['total_tokens']) for x in summary]);print('PURE WAIT',aggregate['root_pure_wait_records'],aggregate['root_pure_wait_tokens']);print('POLL',len(polls),len(pure_poll),len(no_progress));print('PROGRAM',program_sums);print('ROOT INPUT',aggregate['root_input_per_response']);print('SOURCE NETWORK',network);print('LARGEST',samples)
