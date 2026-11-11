"""Seek only metadata-indexed usage records; avoid rereading transcript bodies."""
import hashlib,json
from pathlib import Path
OUT=Path(__file__).resolve().parent
records=[];settings=[];states=[];compactions=[];handles={}
for line in (OUT/'event_metadata.jsonl').open():
 e=json.loads(line)
 if e['type'] not in ('token_usage_record','compacted','turn_context') and not(e['type']=='event_msg' and e['payload_type'] in ('task_complete','task_started','thread_settings_applied')):continue
 f=handles.setdefault(e['path'],Path(e['path']).open('rb')) if e['path'] not in handles else handles[e['path']]
 f.seek(e['byte_offset']);raw=json.loads(f.readline());p=raw.get('payload',{})
 if e['type']=='token_usage_record':
  records.append(dict(path=e['path'],line=e['line'],byte_offset=e['byte_offset'],timestamp=e['timestamp'],thread_id=p.get('thread_id'),session_id=p.get('session_id'),turn_id=p.get('turn_id'),root_turn_id=p.get('root_turn_id'),response_id_hash=hashlib.sha256(p.get('response_id','').encode()).hexdigest(),usage=p.get('usage'),turn_token_usage=p.get('turn_token_usage'),thread_token_usage=p.get('thread_token_usage')))
 elif e['type']=='turn_context':settings.append(dict(path=e['path'],line=e['line'],timestamp=e['timestamp'],session_id=e['session_id'],**{k:p.get(k) for k in ('turn_id','model','effort','reasoning_effort','model_provider','auth_mode')}))
 elif e['type']=='compacted':compactions.append(dict(path=e['path'],line=e['line'],timestamp=e['timestamp'],keys=list(p),compaction_response_id=p.get('compaction_response_id'),latest_token_usage_record=p.get('latest_token_usage_record')))
 else:states.append(dict(path=e['path'],line=e['line'],timestamp=e['timestamp'],session_id=e['session_id'],payload_type=e['payload_type'],safe_fields={k:p.get(k) for k in ('turn_id','model','reasoning_effort','auth_mode','service_tier','plan_type') if k in p}))
for f in handles.values():f.close()
for name,rows in [('token_usage_detail.jsonl',records),('turn_settings.jsonl',settings),('task_events.jsonl',states),('compaction_metadata.jsonl',compactions)]:
 (OUT/name).write_text(''.join(json.dumps(x,ensure_ascii=False)+'\n' for x in rows))
root='01a18b08-73cc-7001-ba3d-92d49ecdd80e';turns={'01a19760-00c8-74f2-a1ce-764b0e651cc0','01a1979c-a4e1-7493-b137-0ce18a0d1fc2','01a19972-5fcc-71a2-bf8a-f076f9d4d354'}
chosen=[r for r in records if r['root_turn_id'] in turns];print('records',len(records),'task',len(chosen))
from collections import Counter
print('by_thread',Counter(r['thread_id'] for r in chosen));print('window',min(r['timestamp'] for r in chosen),max(r['timestamp'] for r in chosen));print('usage_sum',{k:sum(r['usage'][k] for r in chosen) for k in chosen[0]['usage']});print('root_window',[(r['line'],r['timestamp'],r['thread_token_usage']) for r in records if r['thread_id']==root and r['root_turn_id'] in turns][:1]);print('root_last',[(r['line'],r['timestamp'],r['thread_token_usage']) for r in records if r['thread_id']==root and r['root_turn_id'] in turns][-1:])
print('task_events_root',[s for s in states if s['session_id']==root and s['timestamp']>'2026-11-01T13:00:00' and s['payload_type'] in ('task_started','task_complete')])
