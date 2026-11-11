#!/usr/bin/env python3
"""Lossless shared QP detail objects; all logical JSONL bytes must restore unchanged."""
import argparse
from functools import lru_cache
import gzip
import hashlib
import json
from pathlib import Path
import re
import os
from m2a_logs import encode


def sha(raw):return hashlib.sha256(raw).hexdigest()


def atomic_write(path,raw):
 temporary=path.with_name(path.name+'.write_tmp')
 with temporary.open('wb') as stream:stream.write(raw);stream.flush();os.fsync(stream.fileno())
 temporary.replace(path);fd=os.open(path.parent,os.O_DIRECTORY)
 try:os.fsync(fd)
 finally:os.close(fd)

def object_loader(root):
 @lru_cache(maxsize=32)
 def load(key):
  if not re.fullmatch('[0-9a-f]{64}',key):raise ValueError('invalid diagnostic object ID')
  raw=gzip.decompress((root/'diagnostic_objects'/(key+'.json.gz')).read_bytes())
  if sha(raw)!=key:raise ValueError('corrupt diagnostic object')
  return json.loads(raw)
 return load

def expand(value,load):
 if isinstance(value,dict):
  if '$details' in value:
   if set(value)!={'$details'}:raise ValueError('invalid detail reference')
   return load(value['$details'])
  return {k:expand(v,load) for k,v in value.items()}
 if isinstance(value,list):return [expand(v,load) for v in value]
 return value

def logical_digest(path,load=None):
 h=hashlib.sha256();count=0
 with gzip.open(path,'rb') as stream:
  for line in stream:
   raw=encode([expand(json.loads(line),load)]) if load else line
   h.update(raw);count+=1
 return h.hexdigest(),count

def archive(root):
 status=json.loads((root/'execution_status.json').read_text());assert status['completed']
 objects=root/'diagnostic_objects';objects.mkdir(exist_ok=True);manifest={};load=object_loader(root)
 def shrink(value):
  if isinstance(value,dict):
   result={}
   for k,v in value.items():
    if k=='_details' and v is not None:
     raw=encode([v]);key=sha(raw);out=objects/(key+'.json.gz')
     if not out.exists():atomic_write(out,gzip.compress(raw,mtime=0))
     result[k]={'$details':key}
    else:result[k]=shrink(v)
   return result
  if isinstance(value,list):return [shrink(v) for v in value]
  return value
 for group in ('C-FIFO','L-FIFO','L-ALT'):
  for directory in sorted((root/group).glob('r[12]')):
   path=directory/'diagnostics.jsonl.gz';descriptor=directory/'diagnostic_archive.json'
   if descriptor.exists():
    data=json.loads(descriptor.read_text());digest,count=logical_digest(path,load)
    if digest!=data['original_logical_sha256'] or count!=data['rows']:raise ValueError('existing archive checkpoint mismatch')
    manifest[str(path.relative_to(root))]=data;print('existing archive revalidated',directory,flush=True);continue
   before,count=logical_digest(path);old_size=path.stat().st_size;temporary=path.with_name(path.name+'.archive_tmp')
   with gzip.open(path,'rb') as source,temporary.open('wb') as raw:
    with gzip.GzipFile(fileobj=raw,filename='',mode='wb',mtime=0) as output:
     for line in source:output.write(encode([shrink(json.loads(line))]))
    raw.flush();os.fsync(raw.fileno())
   after,new_count=logical_digest(temporary,load)
   if before!=after or count!=new_count:raise ValueError('diagnostic logical bytes changed')
   temporary.replace(path)
   data=dict(format='M2b-shared-QP-details-v1',original_logical_sha256=before,restored_logical_sha256=after,rows=count,original_gzip_bytes=old_size,referenced_gzip_bytes=path.stat().st_size,objects='../../diagnostic_objects',full_matrices_preserved=True)
   atomic_write(descriptor,(json.dumps(data,indent=2)+'\n').encode());manifest[str(path.relative_to(root))]=data
   print('diagnostic bytes restored exactly',directory,flush=True)
 atomic_write(root/'diagnostic_archive_manifest.json',(json.dumps(dict(files=manifest,objects=len(list(objects.glob('*.gz'))),stored_object_bytes=sum(p.stat().st_size for p in objects.glob('*.gz'))),indent=2)+'\n').encode())

def restore(root,output):
 output.mkdir(parents=True,exist_ok=False);load=object_loader(root)
 for descriptor in sorted(root.glob('*/r[12]/diagnostic_archive.json')):
  data=json.loads(descriptor.read_text());source=descriptor.with_name('diagnostics.jsonl.gz');out=output/source.relative_to(root);out.parent.mkdir(parents=True,exist_ok=True)
  with gzip.open(source,'rb') as stream,out.open('wb') as raw:
   with gzip.GzipFile(fileobj=raw,filename='',mode='wb',mtime=0) as target:
    for line in stream:target.write(encode([expand(json.loads(line),load)]))
  digest,count=logical_digest(out)
  if digest!=data['original_logical_sha256'] or count!=data['rows']:raise ValueError('restored diagnostic mismatch')
  print('full diagnostic restore verified',source.parent,flush=True)
 return output

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--restore-to',type=Path);a=p.parse_args();restore(a.run,a.restore_to) if a.restore_to else archive(a.run)
