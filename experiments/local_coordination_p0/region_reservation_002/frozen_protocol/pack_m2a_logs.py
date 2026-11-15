#!/usr/bin/env python3
"""Create a compact mirror, retaining originals; verify exact restored bytes."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
from m2a_logs import FORMAT,encode,lines,raw_bytes,stored_messages


def sha(raw):return hashlib.sha256(raw).hexdigest()


def restore_bytes(path):
    return encode(lines(path)) if path.name=='messages.jsonl' else raw_bytes(path)


def verify(destination):
    manifest=json.loads((destination/'storage_manifest.json').read_text())
    for name,item in manifest['files'].items():
        p=destination/name
        raw=restore_bytes(p) if name.endswith('.jsonl') else p.read_bytes()
        if len(raw)!=item['original_bytes'] or sha(raw)!=item['original_sha256']:raise ValueError(f'restored hash mismatch: {name}')
        stored=destination/item['stored_path']
        if sha(stored.read_bytes())!=item['stored_sha256']:raise ValueError(f'stored hash mismatch: {name}')
    return manifest


def pack(source,destination):
    source=source.resolve();destination=destination.resolve()
    if destination==source or source in destination.parents:raise ValueError('destination must be outside source')
    destination.mkdir(parents=True,exist_ok=False)
    manifest=dict(format=FORMAT,source=str(source),originals_retained=True,files={})
    for p in sorted(source.rglob('*')):
        if not p.is_file() or '__pycache__' in p.parts:continue
        relative=p.relative_to(source);out=destination/relative;out.parent.mkdir(parents=True,exist_ok=True);raw=p.read_bytes()
        if p.suffix=='.jsonl':
            stored=raw
            if p.name=='messages.jsonl':
                messages=lines(p);plans=lines(p.with_name('plans.jsonl'))
                stored=encode(stored_messages(messages,plans))
            out=out.with_name(out.name+'.gz');out.write_bytes(gzip.compress(stored,mtime=0))
        else:shutil.copy2(p,out)
        manifest['files'][str(relative)]=dict(original_bytes=len(raw),original_sha256=sha(raw),stored_path=str(out.relative_to(destination)),stored_bytes=out.stat().st_size,stored_sha256=sha(out.read_bytes()))
    manifest['original_bytes']=sum(v['original_bytes'] for v in manifest['files'].values())
    manifest['stored_bytes']=sum(v['stored_bytes'] for v in manifest['files'].values())
    manifest['reduction_fraction']=1-manifest['stored_bytes']/manifest['original_bytes']
    (destination/'storage_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    verify(destination)
    return manifest


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--source',type=Path);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--verify',action='store_true');args=parser.parse_args()
    if not args.verify and args.source is None:parser.error('--source is required for packing')
    result=verify(args.output) if args.verify else pack(args.source,args.output)
    print(json.dumps({k:v for k,v in result.items() if k!='files'}))
