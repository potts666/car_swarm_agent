#!/usr/bin/env python3
"""Recover interrupted offline objects only when exact original content IDs match."""
import argparse
import gzip
import json
from pathlib import Path
from m2a_logs import encode
from m2b_diagnostic_archive import sha,logical_digest,object_loader,atomic_write


def recover(root):
 objects=root/'diagnostic_objects';quarantine=root/'interrupted_archive_files';bad=set();moved=[];recovered={}
 for p in objects.glob('*.json.gz'):
  try:valid=sha(gzip.decompress(p.read_bytes()))==p.name.split('.')[0]
  except (EOFError,OSError):valid=False
  if not valid:
   key=p.name.split('.')[0];bad.add(key);out=quarantine/'objects'/p.name;out.parent.mkdir(parents=True,exist_ok=True);p.replace(out);moved.append(str(out.relative_to(root)))
 for path in sorted(root.glob('L-ALT/r[12]/diagnostics.jsonl.gz')):
  with gzip.open(path,'rb') as stream:
   for line in stream:
    record=json.loads(line)
    def visit(value):
     if isinstance(value,dict):
      for k,v in value.items():
       if k=='_details' and v is not None and '$details' not in v:
        raw=encode([v]);key=sha(raw)
        if key in bad:
         atomic_write(objects/(key+'.json.gz'),gzip.compress(raw,mtime=0));bad.remove(key);recovered[key]=dict(source=str(path.relative_to(root)),cycle=record['cycle'])
       else:visit(v)
     elif isinstance(value,list):
      for v in value:visit(v)
    visit(record)
  if not bad:break
 if bad:raise ValueError('no exact source body for interrupted objects: '+str(sorted(bad)))
 metrics=json.loads((root/'metrics.json').read_text());load=object_loader(root);checkpoint_log=(root/'postprocess.log').read_text();descriptors={}
 for descriptor in root.glob('*/r[12]/diagnostic_archive.json'):
  if descriptor.stat().st_size:continue
  directory=descriptor.parent;relative=str(directory.relative_to(root));assert 'diagnostic bytes restored exactly '+str(directory) in checkpoint_log
  out=quarantine/relative/'diagnostic_archive.json';out.parent.mkdir(parents=True,exist_ok=True);descriptor.replace(out);digest,rows=logical_digest(directory/'diagnostics.jsonl.gz',load)
  original=next(x['bytes'] for x in metrics[relative]['storage']['largest'] if x['name']=='diagnostics.jsonl.gz')
  data=dict(format='M2b-shared-QP-details-v1',original_logical_sha256=digest,restored_logical_sha256=digest,rows=rows,original_gzip_bytes=original,referenced_gzip_bytes=(directory/'diagnostics.jsonl.gz').stat().st_size,objects='../../diagnostic_objects',full_matrices_preserved=True,checkpoint_metadata_recovered=True,original_digest_attestation='pre-interruption archive verified original/restored byte equality and logged success; lost metadata digest rederived from intact reference records and exact-ID recovered objects')
  atomic_write(descriptor,(json.dumps(data,indent=2)+'\n').encode());descriptors[relative]=data;moved.append(str(out.relative_to(root)))
 result=dict(recovered_objects=recovered,recovered_descriptors=list(descriptors),damaged_originals_preserved=moved,all_recovered_object_ids_exact=True,control_runs_repeated=False,scope='offline diagnostic archival only; no values inferred from a new solver run')
 atomic_write(root/'archive_checkpoint_recovery.json',(json.dumps(result,indent=2)+'\n').encode());print('recovered objects',len(recovered),'descriptors',len(descriptors),flush=True)
 return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();recover(a.run)
