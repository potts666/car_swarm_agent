#!/usr/bin/env python3
"""Offline size measurements and byte-identical plan-file hardlinks after all runs."""
import argparse
import gzip
import hashlib
import io
import json
import os
from pathlib import Path
import time
from m2b_recording import lines
from m2a_logs import encode


def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()

def write(p,x):p.write_text(json.dumps(x,indent=2)+'\n')

def encoded_size(rows):
 buffer=io.BytesIO();raw=0
 with gzip.GzipFile(fileobj=buffer,mode='wb',mtime=0,compresslevel=9) as stream:
  for r in rows:
   b=encode([r]);raw+=len(b);stream.write(b)
 return raw,len(buffer.getbuffer())

def raw_size(p):
 size=0
 with gzip.open(p,'rb') as stream:
  for b in iter(lambda:stream.read(1024*1024),b''):size+=len(b)
 return size

def process(root):
 started=time.perf_counter();status=json.loads((root/'execution_status.json').read_text());assert status['completed']
 measurements={};objects=root/'plan_objects';objects.mkdir(exist_ok=True)
 for group in ('C-FIFO','L-FIFO','L-ALT'):
  for directory in sorted((root/group).glob('r[12]')):
   plans=lines(directory/'plans.jsonl');messages=lines(directory/'messages.jsonl',plans)
   plan_raw,plan_gzip=encoded_size(plans);message_raw,message_gzip=encoded_size(messages)
   logs=list(directory.glob('*.jsonl.gz'));raws={p.name:raw_size(p) for p in logs};stored=sum(p.stat().st_size for p in logs);old=directory/'plans.jsonl.gz';digest=sha(old);obj=objects/(digest+'.jsonl.gz')
   if not obj.exists():os.link(old,obj)
   if old.stat().st_ino!=obj.stat().st_ino:
    temporary=directory/'plans.jsonl.gz.dedup_tmp';os.link(obj,temporary);os.replace(temporary,old)
   assert sha(old)==digest
   write(directory/'plan_store.json',dict(compressed_sha256=digest,object=str(obj.relative_to(root)),link_type='hardlink',logical_log_bytes_unchanged=True))
   measurements[str(directory.relative_to(root))]=dict(full_log_raw_bytes=sum(raws.values())-raws['plans.jsonl.gz']-raws['messages.jsonl.gz']+plan_raw+message_raw,gzip_only_log_bytes=stored-old.stat().st_size-(directory/'messages.jsonl.gz').stat().st_size+plan_gzip+message_gzip,referenced_log_raw_bytes=sum(raws.values()),referenced_gzip_log_bytes=stored,full_plan_raw_bytes=plan_raw,full_message_raw_bytes=message_raw,plan_object=digest,normal_logical_content_unchanged=True)
   write(root/'storage_measurements.json',dict(runs=measurements,completed=False));print('storage measured',directory,flush=True)
 files=[p for p in root.rglob('*') if p.is_file() and '__pycache__' not in p.parts];unique={}
 for p in files:unique[(p.stat().st_dev,p.stat().st_ino)]=p.stat().st_size
 summary=dict(runs=measurements,completed=True,scope='JSONL logs for before/after pipeline; package sizes include all files',package_apparent_bytes=sum(p.stat().st_size for p in files),package_unique_file_bytes=sum(unique.values()),plan_store_objects=len(list(objects.glob('*.gz'))),soft_package_200MiB_met=sum(unique.values())<=200*2**20,offline_measurement_wall_s=time.perf_counter()-started,hardlinks_require_tar_preservation_or_object_relink_on_copy=True)
 write(root/'storage_measurements.json',summary);return summary

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();process(a.run)
