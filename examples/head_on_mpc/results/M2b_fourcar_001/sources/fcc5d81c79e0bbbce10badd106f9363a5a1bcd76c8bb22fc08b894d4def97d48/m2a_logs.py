"""Lossless gzip JSONL and recoverable plan references; wire format is unchanged."""
import copy
import gzip
import hashlib
import json
from pathlib import Path

FORMAT='M2a-logs-v2-gzip-plan-ref'


def physical(path):
    path=Path(path);compressed=path.with_name(path.name+'.gz')
    if path.exists() and compressed.exists():raise ValueError(f'ambiguous log: {path}')
    return path if path.exists() else compressed


def raw_bytes(path):
    source=physical(path)
    return gzip.decompress(source.read_bytes()) if source.suffix=='.gz' else source.read_bytes()


def plan_id(plan):
    content={k:v for k,v in plan.items() if k!='id'}
    return hashlib.sha256(json.dumps(content,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def lines(path):
    path=Path(path);rows=[json.loads(s) for s in raw_bytes(path).decode('utf-8').splitlines()]
    if path.name=='messages.jsonl' and any(r.get('kind') in ('seed','final') and r.get('event')=='send' and '$plan' in r.get('payload',{}) for r in rows):
        plans={}
        for p in lines(path.with_name('plans.jsonl')):
            if p['id']!=plan_id(p) or p['id'] in plans:raise ValueError('invalid or duplicate stored plan ID')
            plans[p['id']]=p
        for row in rows:
            if row.get('event')=='send' and row.get('kind') in ('seed','final'):
                ref=row['payload']
                if set(ref)!={'$plan'} or ref['$plan'] not in plans:raise ValueError('missing or invalid plan reference')
                row['payload']=copy.deepcopy(plans[ref['$plan']])
    return rows


def encode(rows):
    return ''.join(json.dumps(r,ensure_ascii=False,allow_nan=False)+'\n' for r in rows).encode('utf-8')


def stored_messages(messages,plans):
    index={}
    for p in plans:
        if p['id']!=plan_id(p) or p['id'] in index:raise ValueError('invalid or duplicate stored plan ID')
        index[p['id']]=p
    for row in messages:
        if row.get('event')=='send' and row.get('kind') in ('seed','final'):
            p=row['payload']
            if p.get('id') not in index or p!=index[p['id']]:raise ValueError('wire/stored plan mismatch')
            row={**row,'payload':{'$plan':p['id']}}
        yield row


def write_log(path,rows):
    path=Path(path);destination=path.with_name(path.name+'.gz')
    if path.exists() or destination.exists():raise FileExistsError(destination)
    destination.write_bytes(gzip.compress(encode(rows),mtime=0))


def write_bundle(directory,logs):
    plans=logs['plans'];messages=list(stored_messages(logs['messages'],plans))
    for name,rows in logs.items():write_log(directory/(name+'.jsonl'),messages if name=='messages' else rows)
    (directory/'log_format.json').write_text(json.dumps(dict(format=FORMAT,compression='gzip',plan_content='plans.jsonl.gz',plan_reference='$plan',wire_payload_unchanged=True,float_precision_unchanged=True),indent=2)+'\n')
