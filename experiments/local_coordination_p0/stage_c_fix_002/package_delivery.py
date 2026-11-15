"""Deduplicate the archive, independently replay after relocation, export Windows data."""
from fix_common import *
import os,sys,subprocess,tarfile,tempfile,zipfile,shutil


def source_files():return [f for f in sorted(HERE.rglob('*')) if f.is_file() and '__pycache__' not in f.parts and 'delivery' not in f.relative_to(HERE).parts]


def manifest():
    files={str(f.relative_to(HERE)):dict(bytes=f.stat().st_size,sha256=sha(f)) for f in source_files() if f!=HERE/'DELIVERY_MANIFEST.json'}
    write(HERE/'DELIVERY_MANIFEST.json',dict(files=files,source_commit=read(HERE/'plan.json')['source_commit'],object_store='objects/<content-id>.json.gz'))


def tarball(dest):
    seen={}
    with tarfile.open(dest,'w:gz') as tar:
        for f in source_files():
            name=str(Path(HERE.name)/f.relative_to(HERE));item=tar.gettarinfo(str(f),arcname=name);digest=sha(f)
            if digest in seen:item.type=tarfile.LNKTYPE;item.linkname=seen[digest];item.size=0;tar.addfile(item)
            else:
                seen[digest]=name
                with f.open('rb') as stream:tar.addfile(item,stream)


def main():
    output=HERE/'delivery';assert not output.exists();output.mkdir()
    manifest()
    with tempfile.TemporaryDirectory(prefix='dmpc-two-update-relocation-') as tmp:
        candidate=Path(tmp)/'candidate.tar.gz';tarball(candidate)
        with tarfile.open(candidate) as tar:tar.extractall(tmp,filter='data')
        env={**os.environ,'PYTHONPATH':'/tmp/dmpc-p0-deps','OPENBLAS_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1'}
        result=subprocess.run([sys.executable,str(Path(tmp)/HERE.name/'verify_delivery.py')],cwd=tmp,env=env,capture_output=True,text=True,timeout=180)
        (output/'relocation_stdout.log').write_text(result.stdout);(output/'relocation_stderr.log').write_text(result.stderr)
        assert result.returncode==0,(result.returncode,result.stderr)
        write(HERE/'relocation_check.json',dict(passed=True,exit_code=result.returncode,solver_calls=0,independent=json.loads(result.stdout),source_root_available_but_not_referenced=True,new_root=tmp))
    manifest();tarpath=output/'DMPC_C0_fix_002_2026-11-13.tar.gz';tarball(tarpath)
    zippath=output/'DMPC_C0_fix_002_2026-11-13.zip'
    with zipfile.ZipFile(zippath,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
        for f in source_files():archive.write(f,str(Path(HERE.name)/f.relative_to(HERE)))
    with zipfile.ZipFile(zippath) as archive:assert archive.testzip() is None
    analysis=output/'C0_fix_002_Windows_analysis_2026-11-13';analysis.mkdir()
    for name in ('report.md','GATE.json','comparison.csv','comparison_windows.csv','metrics.json','event_index.json','versions.json','plan.json','HANDOFF.md','changes.patch','self_checks.json','preservation_check.json','object_identity_check.json','relocation_check.json','requirements.txt','DELIVERY_MANIFEST.json'):
        shutil.copy2(HERE/name,analysis/name)
    shutil.copy2(HERE/'results/solver_calls.json',analysis/'solver_calls.json')
    for name in ('curve','straight_acceleration'):
        target=analysis/name;target.mkdir();src=HERE/'results'/name
        for f in ('config.json','summary.json','events.json','execution.jsonl.gz','independent_acceptance.json'):shutil.copy2(src/f,target/f)
        (target/'execution.jsonl').write_bytes(gzip.decompress((src/'execution.jsonl.gz').read_bytes()))
        for d in ('round_1','round_2','damage_forks'):shutil.copytree(src/d,target/d)
    for f in (tarpath,zippath):
        f.with_suffix(f.suffix+'.sha256').write_text(sha(f)+'  '+f.name+'\n');shutil.copy2(f,analysis/f.name);shutil.copy2(f.with_suffix(f.suffix+'.sha256'),analysis/(f.name+'.sha256'))
    (analysis/'先读我_Windows.txt').write_text('C0_fix_002：连续两轮真实重新优化（2026-11-13）\n\n先读report.md、GATE.json和comparison_windows.csv（UTF-8 BOM，便于Excel）。\n四次新求解，各接口两轮受理/首拍修改、240拍独立验收及损坏拒绝续行均通过；原始状态仍全为AlmostSolved。\nmetrics.json按不重叠34拍实际区间计算收益；每轮32拍预测改善不可相加。\nevent_index.json及solver_calls.json绑定四次求解及提交来源。\ncurve/、straight_acceleration/含原始日志、完整设置、数值输出、控制执行与独立验收；execution.jsonl已解压。damage_forks是复制控制分支，与主性能分开。\n完整代码、对象、冻结依赖及任务书在DMPC_C0_fix_002_2026-11-13.zip，解压后运行verify_delivery.py可独立复核（需requirements.txt中的Python库）。\nDELIVERY_MANIFEST.json对应ZIP内stage_c_fix_002目录；SHARED_FILES_SHA256.json对应本分析目录。\n未开展区域预约、多活动车、C2或批量；车辆优先规则仍待设计。历史结果和论文未改。\n',encoding='utf-8-sig')
    hashes={str(f.relative_to(analysis)):dict(bytes=f.stat().st_size,sha256=sha(f)) for f in sorted(analysis.rglob('*')) if f.is_file()}
    write(analysis/'SHARED_FILES_SHA256.json',hashes)
    shared=Path('/media/sf_DMPC_Theory')/analysis.name;assert not shared.exists(),'preserve an existing shared delivery'
    shutil.copytree(analysis,shared)
    for name,item in hashes.items():assert sha(shared/name)==item['sha256'],name
    assert sha(shared/'SHARED_FILES_SHA256.json')==sha(analysis/'SHARED_FILES_SHA256.json')
    print(json.dumps(dict(passed=True,local_directory=str(output),shared_directory=str(shared),shared_files_verified=len(hashes)+1,archive_sha256=sha(tarpath),archive_bytes=tarpath.stat().st_size,zip_sha256=sha(zippath),zip_bytes=zippath.stat().st_size,relocation_passed=True),ensure_ascii=False,indent=2))
if __name__=='__main__':main()
