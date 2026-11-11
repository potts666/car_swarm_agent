"""Read only explicitly matched benchmark threads and their new guardians."""
import csv
import hashlib
import json
from pathlib import Path
import re
import sqlite3

OUT = Path(__file__).resolve().parent
FIELDS = ['input_tokens', 'cached_input_tokens', 'cache_write_input_tokens',
          'output_tokens', 'reasoning_output_tokens', 'total_tokens']


def load(path):
    return json.loads(path.read_text())


def sum_usage(rows):
    return {k:sum(r['usage'].get(k,0) for r in rows) for k in FIELDS}


def read_thread(path):
    result = dict(path=str(path), usage=[], commands=[], program_outputs=[],
                  policy_messages=0, metadata={}, custom_tool_calls=0)
    previous = dict.fromkeys(FIELDS,0)
    seen = set()
    for line_no,line in enumerate(path.open(),1):
        x=json.loads(line);p=x.get('payload',{})
        if x['type']=='session_meta':
            result['metadata']={k:p.get(k) for k in ['id','session_id','parent_thread_id','source','cwd','cli_version']}
        if x['type']=='token_usage_record':
            key=(p['thread_id'],p['response_id'])
            assert key not in seen
            seen.add(key)
            cumulative=p['thread_token_usage']
            delta={k:cumulative.get(k,0)-previous.get(k,0) for k in FIELDS}
            assert delta=={k:p['usage'].get(k,0) for k in FIELDS}, (path,line_no)
            assert delta['total_tokens']==delta['input_tokens']+delta['output_tokens']
            assert delta['cached_input_tokens']<=delta['input_tokens']
            assert delta['reasoning_output_tokens']<=delta['output_tokens']
            previous=cumulative
            result['usage'].append(dict(line=line_no,timestamp=x.get('timestamp'),thread_id=p['thread_id'],
                                        root_session_id=p['session_id'],root_turn_id=p['root_turn_id'],
                                        response_id_sha256=hashlib.sha256(p['response_id'].encode()).hexdigest(),usage=delta))
        if p.get('type')=='message':
            text=' '.join(str(c.get('text','')) for c in p.get('content',[]) if isinstance(c,dict))
            result['policy_messages']+=int('Token-efficient execution:' in text)
        if p.get('type')=='custom_tool_call':
            result['custom_tool_calls']+=1
            source=p.get('input','')
            m=re.search(r'\bcmd\s*:\s*("(?:[^"\\]|\\.)*")',source)
            if m:result['commands'].append(json.loads(m.group(1)))
        if p.get('type')=='custom_tool_call_output':
            for block in p.get('output',[]):
                try:r=json.loads(block.get('text',''))
                except (ValueError,AttributeError):continue
                if isinstance(r,dict) and 'exit_code' in r and 'output' in r:
                    result['program_outputs'].append(r)
    return result


def main():
    cases=[]
    for label,folder in [('pilot',OUT/'pilot_sandbox_failure'),('measured',OUT)]:
        manifest=load(folder/'manifest.json')
        for arm in manifest['arms']:
            records=[json.loads(x) for x in (folder/(arm['arm']+'.events.jsonl')).open()]
            sid=next(x['thread_id'] for x in records if x.get('type')=='thread.started')
            completed=[x for x in records if x.get('type')=='turn.completed']
            assert len(completed)==1
            cases.append(dict(label=label,folder=folder,manifest=manifest,arm=arm,sid=sid,completed_usage=completed[0]['usage']))
    con=sqlite3.connect('file:/home/sunflower/.codex/state_5.sqlite?mode=ro',uri=True)
    start=min(c['arm']['started_unix'] for c in cases)-3
    end=max(c['arm']['ended_unix'] for c in cases)+3
    index=con.execute('select id,source,rollout_path from threads where created_at>=? and created_at<=?',(int(start),int(end))).fetchall()
    main_ids={c['sid'] for c in cases}
    threads={}
    for sid,source,path in index:
        if sid in main_ids:
            threads[sid]=read_thread(Path(path))
        elif 'guardian' in source:
            # Only inspect first-line relationship before reading child usage.
            with Path(path).open() as f:meta=json.loads(f.readline()).get('payload',{})
            if meta.get('parent_thread_id') in main_ids:
                threads[sid]=read_thread(Path(path))
    summaries=[];increments=[]
    for case in cases:
        sid=case['sid'];main=threads[sid]
        own=main['usage']
        guardians=[t for t in threads.values() if t['metadata'].get('parent_thread_id')==sid]
        child=[r for t in guardians for r in t['usage'] if r['root_session_id']==sid]
        a=sum_usage(own);b=sum_usage(child)
        assert all(a[k]==v for k,v in case['completed_usage'].items())
        outputs=main['program_outputs']
        valid=bool(outputs) and all(r['exit_code']==0 and re.search(r'Ran 3 tests in [0-9.]+s',r['output']) and r['output'].strip().endswith('OK') for r in outputs)
        if case['label']=='measured':
            assert valid
            assert main['commands']==[case['manifest']['command']]
        state=Path('/home/sunflower/.codex/hooks/token_guard/state')/(hashlib.sha256(sid.encode()).hexdigest()+'.json')
        summary=dict(case=case['label'],arm=case['arm']['arm'],session_id=sid,
                     source=main['path'],hooks_enabled=case['arm']['hooks_enabled'],
                     numerical_checks_passed=valid,test_count=3 if valid else 0,
                     codex_wall_seconds=case['arm']['wall_seconds'],main_usage_records=len(own),
                     guardian_usage_records=len(child),guardian_sessions=[t['metadata']['id'] for t in guardians],
                     main=a,guardian=b,total={k:a[k]+b[k] for k in FIELDS},
                     custom_tool_calls=main['custom_tool_calls'],policy_messages=main['policy_messages'],
                     hook_state=load(state) if state.exists() else None,
                     program_outputs=outputs,answer=(case['folder']/(case['arm']['arm']+'.answer.txt')).read_text().strip())
        summaries.append(summary)
        for actor,records in [('main',own),('guardian',child)]:
            increments.extend(dict(case=case['label'],arm=case['arm']['arm'],actor=actor,**r) for r in records)
    measured=[s for s in summaries if s['case']=='measured']
    before,after=measured
    assert before['policy_messages']==0 and after['policy_messages']==1
    assert before['main_usage_records']==after['main_usage_records']==2
    assert before['custom_tool_calls']==after['custom_tool_calls']==1
    delta={actor:{k:after[actor][k]-before[actor][k] for k in FIELDS} for actor in ['main','guardian','total']}
    result=dict(design='One fresh-session pair; hooks disabled vs enabled, identical prompt/model/checks; A then B',
                model='gpt-6.1-sol',reasoning='medium',cases=summaries,measured_delta=delta,
                measured_total_change_percent=100*delta['total']['total_tokens']/before['total']['total_tokens'],
                all_benchmark_sessions_total=sum(s['total']['total_tokens'] for s in summaries),
                sources_unchanged=load(OUT/'manifest.json')['sources_unchanged'],
                limitations=['One pair only; ordering/cache state and model variation uncontrolled.',
                             'Same numerical shell command; generated justification wording differs.',
                             'No native polling/waiting occurred; no operational hook note fired.',
                             'Current parent conversation, setup, analysis and reporting usage excluded and not treated as savings.',
                             'Record tokens are not fees or subscription units.'])
    (OUT/'comparison.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    (OUT/'usage_increments.json').write_text(json.dumps(increments,indent=2)+'\n')
    with (OUT/'comparison.csv').open('w',newline='') as f:
        keys=['case','arm','actor','records',*FIELDS]
        writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader()
        for s in summaries:
            for actor in ['main','guardian']:
                writer.writerow(dict(case=s['case'],arm=s['arm'],actor=actor,records=s[actor+'_usage_records'],**s[actor]))
    print(json.dumps({k:result[k] for k in ['measured_delta','measured_total_change_percent','all_benchmark_sessions_total','sources_unchanged']},indent=2))


if __name__=='__main__':main()
