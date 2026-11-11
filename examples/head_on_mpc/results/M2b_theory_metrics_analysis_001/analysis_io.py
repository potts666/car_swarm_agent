"""Bounded streaming of gzip logs and verified in-bundle plan references."""
import gzip
import hashlib
import itertools
import json
from pathlib import Path


def canonical(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)
def digest(value):return hashlib.sha256(canonical(value).encode()).hexdigest()
def file_sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(1048576),b''):h.update(chunk)
    return h.hexdigest()


def physical(path):
    path=Path(path);gz=path.with_name(path.name+'.gz')
    if path.exists() and gz.exists():raise ValueError('ambiguous log '+str(path))
    actual=path if path.exists() else gz
    if not actual.exists():raise FileNotFoundError(str(actual))
    return actual


def stream(path):
    path=physical(path)
    with (gzip.open(path,'rt') if path.suffix=='.gz' else path.open()) as f:
        for n,line in enumerate(f,1):
            try:yield json.loads(line)
            except ValueError as e:raise ValueError(f'{path}:{n}: invalid JSON') from e


def restored_plans(path):
    # Seeds can only refer to their own last committed final. Four plans remain live.
    previous={};seen_cycle=None;seen=set()
    for p in stream(path):
        cycle=p['context']['cycle']
        if seen_cycle!=cycle:seen_cycle=cycle;seen=set()
        if p['id'] in seen:raise ValueError('duplicate plan within cycle')
        seen.add(p['id'])
        if isinstance(p['states'],dict):
            ref=p['states'];old=previous.get(p['car'])
            if set(ref)!={'$shift_states','skip','tail'} or old is None or old['id']!=ref['$shift_states']:
                raise ValueError('missing shifted plan body')
            skip=ref['skip']
            if not isinstance(skip,int) or skip<=0 or len(ref['tail'])!=skip or p['kind']!='seed' or p['shifted_from']!=old['id'] or old['context']['cycle']!=cycle-1:
                raise ValueError('invalid shifted plan reference')
            p['states']=old['states'][skip:]+ref['tail']
        if digest({k:v for k,v in p.items() if k!='id'})!=p['id']:raise ValueError('plan content hash mismatch')
        if p['kind']=='final':previous[p['car']]=p
        yield p


class CycleStream:
    def __init__(self,values,key):
        self.iterator=iter(values);self.key=key;self.next=next(self.iterator,None)
    def take(self,cycle):
        rows=[]
        if self.next is not None and self.key(self.next)<cycle:raise ValueError('unconsumed or unordered cycle')
        while self.next is not None and self.key(self.next)==cycle:
            rows.append(self.next);self.next=next(self.iterator,None)
        return rows


def logical_motion(directory):
    h=hashlib.sha256();count=0
    for row in stream(directory/'actual.jsonl'):
        h.update(canonical({k:row[k] for k in ['cycle','owner','permission_version','states','controls']}).encode());count+=1
    return h.hexdigest(),count


def run_paths(results):
    old=results/'M2b_fourcar_001';stage=results/'M2b_stage1_001'
    for group in ['C-FIFO','L-FIFO','L-ALT']:
        for repeat in ['r1','r2']:yield group+'/'+repeat,old/group/repeat,'formal'
    for group in ['debug_v1','debug_v2','debug_v3','debug_c1']:yield group,old/group/'r1','natural_debug'
    yield 'stage1/full_L_FIFO',stage/'full_L_FIFO/r1','formal_optimized'
    for group in ['A_uncached','B_cached','B_no_profiler','phase_probe_uncached','phase_probe_cached']:
        yield 'stage1/'+group,stage/group/'r1','bounded_prefix'
    yield 'stage1/fault_injection',stage/'fault_injection/r1','artificial_rejection'
