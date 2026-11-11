#!/usr/bin/env python3
"""Create and read back a self-contained tar.gz, preserving immutable plan hardlinks."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import tarfile


def digest(path):
 h=hashlib.sha256()
 with path.open('rb') as stream:
  for b in iter(lambda:stream.read(1024*1024),b''):h.update(b)
 return h.hexdigest()

def package(root,output):
 if output.exists():raise FileExistsError(output)
 manifest=json.loads((root/'artifact_manifest.json').read_text());temporary=output.with_name(output.name+'.writing')
 with temporary.open('xb') as raw:
  with tarfile.open(fileobj=raw,mode='w:gz',dereference=False,compresslevel=6) as archive:
   archive.add(root,arcname=root.name,filter=lambda info:None if '__pycache__' in Path(info.name).parts else info)
  raw.flush();os.fsync(raw.fileno())
 verified={};links=0
 with tarfile.open(temporary,'r:gz') as archive:
  for member in archive:
   if member.isdir():continue
   relative=str(Path(member.name).relative_to(root.name))
   if member.islnk():
    expected=verified.get(member.linkname)
    if expected is None:raise ValueError('hardlink target not earlier in archive')
    links+=1;value=expected
   elif member.isfile():
    stream=archive.extractfile(member);h=hashlib.sha256()
    for b in iter(lambda:stream.read(1024*1024),b''):h.update(b)
    value=h.hexdigest()
   else:raise ValueError('unexpected archive member type')
   if relative=='artifact_manifest.json':assert value==digest(root/'artifact_manifest.json')
   else:assert manifest[relative]==value,relative
   verified[member.name]=value
 assert len(verified)==len(manifest)+1
 temporary.replace(output);fd=os.open(output.parent,os.O_DIRECTORY)
 try:os.fsync(fd)
 finally:os.close(fd)
 result=dict(archive=output.name,bytes=output.stat().st_size,MiB=output.stat().st_size/2**20,sha256=digest(output),verified_file_entries=len(verified),hardlinks_preserved=links,manifest_sha256=digest(root/'artifact_manifest.json'),soft_archive_200MiB_met=output.stat().st_size<=200*2**20)
 receipt=output.with_name(output.name+'.receipt.json')
 with receipt.open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n');stream.flush();os.fsync(stream.fileno())
 print(json.dumps(result),flush=True);return result

if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();package(a.run,a.output)
