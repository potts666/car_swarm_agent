"""Standalone integrity check; no ROS, NumPy or optimizer is required."""
import gzip
import hashlib
import json
from pathlib import Path

root=Path(__file__).resolve().parent
manifest=json.loads((root/'artifact_manifest.json').read_text())
for name,digest in manifest.items():
 p=root/name;h=hashlib.sha256()
 with p.open('rb') as stream:
  for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
 assert h.hexdigest()==digest,name
for p in (root/'diagnostic_objects').glob('*.json.gz'):
 assert hashlib.sha256(gzip.decompress(p.read_bytes())).hexdigest()==p.name.split('.')[0],p.name
print(json.dumps(dict(artifact_files_passed=len(manifest),diagnostic_object_content_ids_passed=True,scope='integrity only; mathematical acceptance is recorded separately')))
