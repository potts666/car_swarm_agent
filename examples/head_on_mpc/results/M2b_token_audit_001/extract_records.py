"""Bounded stream extraction: identifiers, metrics and small operation summaries only."""
import collections
import hashlib
import json
from pathlib import Path
import re

OUT=Path(__file__).resolve().parent
BASE=Path('/home/sunflower/.codex/sessions')
FILES=[BASE/'2026/10/30/rollout-2026-10-30T12-19-09-01a18b08-73cc-7001-ba3d-92d49ecdd80e.jsonl',BASE/'2026/11/01/rollout-2026-11-01T12-46-36-01a1956e-4cce-7131-8d9c-6b797ba71e29.jsonl',BASE/'2026/11/01/rollout-2026-11-01T22-56-21-01a1979c-8d37-7aa3-ba72-e53cf10480a8.jsonl',BASE/'2026/11/02/rollout-2026-11-02T07-29-24-01a19972-433f-7c82-ac47-efafdf5ddefb.jsonl']
TOKENS=('M2b','MRIB','四车','token消耗','继续','M2a_storage','finalize_m2b','run_m2b_study','package_m2b','177.0','176.54','存储','压缩')
meta=[];errors=[];users=[];kinds=collections.Counter();usage=[]
with (OUT/'event_metadata.jsonl').open('w') as events:
 for path in FILES:
  sid=path.stem.rsplit('-',5)[-1];filemeta={'path':str(path),'bytes':path.stat().st_size};offset=0
  with path.open('rb') as stream:
   for number,line in enumerate(stream,1):
    start=offset;offset+=len(line)
    try:r=json.loads(line)
    except Exception:
     errors.append(dict(path=str(path),line=number,byte_offset=start,bytes=len(line),nul_bytes=line.count(b'\0'),blank=not line.strip()));continue
    d=r.get('payload',{});t=r.get('type');pt=d.get('type') if isinstance(d,dict) else None;kinds[(t,pt)]+=1
    item=dict(path=str(path),line=number,byte_offset=start,timestamp=r.get('timestamp'),type=t,payload_type=pt)
    if t=='session_meta':
     sid=d['id'];filemeta.update({k:d.get(k) for k in ('id','timestamp','cwd','originator','cli_version','source','model_provider','parent_thread_id','forked_from_id')})
    item['session_id']=sid
    if t=='turn_context':item['context']={k:d.get(k) for k in ('model','effort','reasoning_effort','turn_id')}
    if t=='event_msg' and pt=='token_count':
     safe=dict(info=d.get('info'),rate_limits=d.get('rate_limits'));item['usage']=safe;usage.append(item)
    if t in ('response_item','compacted'):
     name=d.get('name','');role=d.get('role');item.update(name=name,role=role,call_id=d.get('call_id'))
     if pt in ('function_call','custom_tool_call'):
      text=d.get('arguments',d.get('input',''));text=text if isinstance(text,str) else json.dumps(text)
      item.update(input_bytes=len(text.encode()),input_sha256=hashlib.sha256(text.encode()).hexdigest(),tool_tags=[k for k in ('exec_command','write_stdin','clock.sleep','clock__curr_time','unittest','pytest','cat ','read_text','rg ','tail ','head ','image','sleep','package_m2b','run_m2b','verify_m2b','storage','archive','recover','finaliz','spawn_agent','send_message') if k in text])
      item['operation_lines']=[x.strip()[:230] for x in text.splitlines() if any(k in x for k in ('cmd:','"cmd":','python3 ','write_stdin','duration_ms','session_id','cell_id'))][:5]
      item['paths']=re.findall(r'(?:/home/sunflower/[^\s\"\'`<>;,)]+|examples/head_on_mpc/[^\s\"\'`<>;,)]+)',text)
      item['sessions_waited']=re.findall(r'session_id[\"\']?\s*:\s*(\d+)',text)
      item['sleep_ms']=re.findall(r'duration_ms[\"\']?\s*:\s*(\d+)',text)
     elif pt in ('function_call_output','custom_tool_call_output'):
      text=d.get('output','');text=text if isinstance(text,str) else json.dumps(text)
      item.update(output_bytes=len(text.encode()),output_sha256=hashlib.sha256(text.encode()).hexdigest(),output_tags=[k for k in TOKENS if k in text]);item['session_ids_returned']=re.findall(r'session_id[\"\']?\s*:\s*(\d+)',text);item['cell_ids_returned']=re.findall(r'cell ID\s+(\d+)',text)
      item['exit_codes']=re.findall(r'(?:exit_code[\"\']?\s*:\s*|exit code\s+)(-?\d+)',text)
      item['wall_seconds']=re.findall(r'(?:wall_time_seconds[\"\']?\s*:\s*|Wall time:\s*)([0-9.]+)',text)
     elif pt=='message':
      content=d.get('content',[]);text='\n'.join(v.get('text','') for v in content if isinstance(v,dict));item.update(channel=d.get('channel'),text_bytes=len(text.encode()),tags=[k for k in TOKENS if k in text])
      if role=='user':
       label='environment/context' if '<environment_context>' in text or '<external_codex_apps_open_page>' in text or text.startswith('<skill>') else text[:260].replace('\n',' ')
       item['user_label']=label;users.append({k:item.get(k) for k in ('session_id','line','timestamp','text_bytes','tags','user_label')})
      if role=='assistant' and (d.get('channel')=='final' or item['tags']):item['assistant_excerpt']=text[:220].replace('\n',' ')
     elif t=='compacted':item['keys']=list(d)
    events.write(json.dumps(item,ensure_ascii=False)+'\n')
  meta.append(filemeta)
(OUT/'session_metadata.json').write_text(json.dumps(meta,indent=2,ensure_ascii=False)+'\n')
(OUT/'usage_records.jsonl').write_text(''.join(json.dumps(v,ensure_ascii=False)+'\n' for v in usage))
(OUT/'user_boundaries.json').write_text(json.dumps(users,indent=2,ensure_ascii=False)+'\n')
(OUT/'parse_errors.json').write_text(json.dumps(errors,indent=2)+'\n')
print(json.dumps(dict(files=len(FILES),usage_events=len(usage),parse_errors=len(errors),invalid_bytes=sum(e['bytes'] for e in errors),type_counts=[[*k,v] for k,v in kinds.items()]),ensure_ascii=False))
print('ROOT TASK BOUNDARIES')
for u in users:
 if u['session_id']==meta[0]['id'] and ('M2b' in u['tags'] or '四车' in u['tags'] or 'token消耗' in u['tags'] or '继续' in u['tags']):print(json.dumps(u,ensure_ascii=False))
