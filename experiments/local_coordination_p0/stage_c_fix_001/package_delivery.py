"""Content-deduplicated distribution and relocation acceptance; no new solves."""
from fix_common import *
import os,sys,subprocess,tarfile,tempfile
import clarabel,numpy,scipy

def manifest():
    files={str(f.relative_to(HERE)):dict(bytes=f.stat().st_size,sha256=sha(f)) for f in sorted(HERE.rglob('*')) if f.is_file() and '__pycache__' not in f.parts and f!=HERE/'DELIVERY_MANIFEST.json'}
    write(HERE/'DELIVERY_MANIFEST.json',dict(files=files,source_commit=read(HERE/'plan.json')['source_commit'],content_objects='objects/<sha256>.json.gz'))

def archive(dest):
    seen={};total=0;unique=0
    with tarfile.open(dest,'w:gz') as tar:
        for f in sorted(HERE.rglob('*')):
            if not f.is_file() or '__pycache__' in f.parts:continue
            name=str(Path(HERE.name)/f.relative_to(HERE));info=tar.gettarinfo(str(f),arcname=name);digest=sha(f);total+=f.stat().st_size
            if digest in seen:info.type=tarfile.LNKTYPE;info.linkname=seen[digest];info.size=0;tar.addfile(info)
            else:
                seen[digest]=name;unique+=f.stat().st_size
                with f.open('rb') as stream:tar.addfile(info,stream)
    return dict(uncompressed_bytes=total,unique_bytes=unique,unique_contents=len(seen))

def main():
    delivery=HERE.parents[1]/'deliverables';delivery.mkdir(exist_ok=True);dest=delivery/'DMPC_C0_fix_001_2026-11-13.tar.gz';assert not dest.exists(),'preserve existing distribution'
    objects=[]
    for f in (HERE/'objects').glob('*.json.gz'):get(HERE/'objects',f.name[:-8]);objects.append(f.name[:-8])
    write(HERE/'results/object_identity_check.json',dict(passed=True,count=len(objects),objects=sorted(objects)))
    write(HERE/'versions.json',dict(python=sys.version,clarabel=clarabel.__version__,numpy=numpy.__version__,scipy=scipy.__version__))
    (HERE/'requirements.txt').write_text(f'clarabel=={clarabel.__version__}\nnumpy=={numpy.__version__}\nscipy=={scipy.__version__}\n')
    manifest()
    with tempfile.TemporaryDirectory(prefix='dmpc-c0-fix-relocation-') as tmp:
        packed=Path(tmp)/'candidate.tar.gz';archive(packed)
        with tarfile.open(packed) as tar:tar.extractall(tmp,filter='data')
        new=Path(tmp)/HERE.name
        env={**os.environ,'PYTHONPATH':'/tmp/dmpc-p0-deps','OPENBLAS_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1'}
        checked=subprocess.run([sys.executable,str(new/'verify_delivery.py')],cwd=tmp,env=env,capture_output=True,text=True,timeout=180)
        (HERE/'results/relocation_stdout.log').write_text(checked.stdout);(HERE/'results/relocation_stderr.log').write_text(checked.stderr)
        assert checked.returncode==0,(checked.returncode,checked.stderr)
        write(HERE/'relocation_check.json',dict(passed=True,exit_code=checked.returncode,new_root=tmp,original_sibling_dependencies=False,independent_acceptance=json.loads(checked.stdout),solver_calls=0))
    # Include the actual relocation evidence in the final content manifest and tar.
    manifest();stats=archive(dest)
    digest=sha(dest);dest.with_suffix(dest.suffix+'.sha256').write_text(digest+'  '+dest.name+'\n')
    print(json.dumps(dict(path=str(dest),bytes=dest.stat().st_size,sha256=digest,**stats),indent=2))
if __name__=='__main__':main()
