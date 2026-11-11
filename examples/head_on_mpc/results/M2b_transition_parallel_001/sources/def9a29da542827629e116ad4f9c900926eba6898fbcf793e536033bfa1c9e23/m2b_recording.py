"""Compact normal records; full recent/key-cycle diagnostics, gzip and recoverable plans."""
import copy
import hashlib
import json
import math
import gzip
from pathlib import Path
import numpy as np
from m2a_logs import write_log,encode,physical,plan_id,stored_messages


def plain(value):
    if isinstance(value,np.ndarray):return plain(value.tolist())
    if isinstance(value,np.generic):return plain(value.item())
    if isinstance(value,float) and not math.isfinite(value):return 'Infinity' if value>0 else '-Infinity' if value<0 else 'NaN'
    if isinstance(value,dict):return {k:plain(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [plain(v) for v in value]
    return value


def compact(record):
    if record is None:return None
    if isinstance(record,dict):return {k:compact(v) for k,v in record.items() if k not in ('_details','separating_rows')}
    if isinstance(record,(list,tuple)):return [compact(v) for v in record]
    return plain(record)


def diagnostics_record(record):
    # Normal logs retain all errors/iterations/inputs. Matrices and row certificates are key-only.
    return plain(record)


def stored_plans(plans):
    index={};stored=[];saved=0
    for p in plans:
        if p['id'] in index or plan_id(p)!=p['id']:raise ValueError('duplicate/invalid plan ID')
        source=index.get(p.get('shifted_from'));candidate=p
        if p['kind']=='seed' and source is not None:
            skip=(len(p['states'])-1)//len(p['commands']);recovered=source['states'][skip:]+p['states'][-skip:]
            if recovered==p['states'] and plan_id({**p,'states':recovered})==p['id']:
                candidate={**p,'states':{'$shift_states':source['id'],'skip':skip,'tail':p['states'][-skip:]}}
                saved+=len(encode([p]))-len(encode([candidate]))
        stored.append(candidate);index[p['id']]=p
    return stored,saved


def export(directory,logs,diagnostics):
    plans,saved=stored_plans(logs['plans']);messages=list(stored_messages(logs['messages'],logs['plans']))
    for name,rows in logs.items():write_log(directory/(name+'.jsonl'),plans if name=='plans' else messages if name=='messages' else rows)
    write_log(directory/'diagnostics.jsonl',diagnostics)
    (directory/'recording_stats.json').write_text(json.dumps(dict(seed_prefix_raw_bytes_saved=saved,seed_state_references=sum(isinstance(p['states'],dict) for p in plans)),indent=2)+'\n')
    (directory/'log_format.json').write_text(json.dumps(dict(format='M2b-logs-v2-shift-ref',base='M2a-logs-v2-gzip-plan-ref',plan_content='plans.jsonl.gz',references='$plan',seed_states='previous committed final suffix + explicit new tail; original ID verified',final_states='full 321 nodes',float_precision_unchanged=True,detailed_cycles='failure and previous two; handoff/repair and neighbors'),indent=2)+'\n')


def lines(path,plans=None):
    """Stream compressed text; recovered plan bodies are shared read-only, not triplicated."""
    path=Path(path);source=physical(path)
    opener=gzip.open if source.suffix=='.gz' else open
    with opener(source,'rt',encoding='utf-8') as stream:rows=[json.loads(row) for row in stream]
    if path.name=='plans.jsonl':
        index={}
        for p in rows:
            body=p['states']
            if isinstance(body,dict):
                if set(body)!={'$shift_states','skip','tail'} or body['$shift_states'] not in index:raise ValueError('missing shifted plan content')
                previous=index[body['$shift_states']];skip=body['skip']
                if previous['kind']!='final' or p['shifted_from']!=previous['id'] or p['car']!=previous['car'] or p['kind']!='seed' or p['context']['cycle']!=previous['context']['cycle']+1 or not isinstance(skip,int) or skip<=0 or skip!=len(body['tail']):raise ValueError('invalid shifted plan reference')
                p['states']=previous['states'][skip:]+body['tail']
            if p['id'] in index or p['id']!=plan_id(p):raise ValueError('invalid/duplicate plan content ID')
            index[p['id']]=p
    if path.name=='messages.jsonl':
        plans=lines(path.with_name('plans.jsonl')) if plans is None else plans
        index={}
        for p in plans:
            if p['id']!=plan_id(p) or p['id'] in index:raise ValueError('invalid/duplicate plan content ID')
            index[p['id']]=p
        for m in rows:
            if m.get('event')=='send' and m.get('kind') in ('seed','final'):
                ref=m['payload']
                if set(ref)!={'$plan'} or ref['$plan'] not in index:raise ValueError('missing/corrupt plan reference')
                m['payload']=index[ref['$plan']]
    return rows
