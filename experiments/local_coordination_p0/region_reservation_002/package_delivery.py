"""Content-deduplicated portable package and Windows shared export."""
from common import *
import os,sys,subprocess,tempfile,tarfile,zipfile,shutil


def files():return [f for f in sorted(HERE.rglob('*')) if f.is_file() and '__pycache__' not in f.parts and 'delivery' not in f.relative_to(HERE).parts]
def manifest():write(HERE/'DELIVERY_MANIFEST.json',dict(files={str(f.relative_to(HERE)):dict(bytes=f.stat().st_size,sha256=sha(f)) for f in files() if f!=HERE/'DELIVERY_MANIFEST.json'}))
def archive(dest):
    seen={}
    with tarfile.open(dest,'w:gz') as tar:
        for f in files():
            name=str(Path(HERE.name)/f.relative_to(HERE));item=tar.gettarinfo(str(f),arcname=name);digest=sha(f)
            if digest in seen:item.type=tarfile.LNKTYPE;item.linkname=seen[digest];item.size=0;tar.addfile(item)
            else:
                seen[digest]=name
                with f.open('rb') as s:tar.addfile(item,s)

def main():
    out=HERE/'delivery';assert not out.exists();out.mkdir();manifest()
    with tempfile.TemporaryDirectory(prefix='region-two-round-relocation-') as tmp:
        packed=Path(tmp)/'candidate.tar.gz';archive(packed)
        with tarfile.open(packed) as tar:tar.extractall(tmp,filter='data')
        env={**os.environ,'PYTHONPATH':'/tmp/dmpc-p0-deps','OPENBLAS_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1'}
        run=subprocess.run([sys.executable,str(Path(tmp)/HERE.name/'verify_delivery.py')],cwd=tmp,env=env,capture_output=True,text=True,timeout=180)
        (out/'relocation_stdout.log').write_text(run.stdout);(out/'relocation_stderr.log').write_text(run.stderr);assert run.returncode==0,(run.returncode,run.stderr)
        write(HERE/'relocation_check.json',dict(passed=True,exit_code=0,solver_calls=0,mode='content checked full acceptance plus fresh two actual sources/prefixes per case; no full-task repeat',result=json.loads(run.stdout)))
    manifest();tarpath=out/'DMPC_region_reservation_002_2026-11-13.tar.gz';archive(tarpath);zippath=out/'DMPC_region_reservation_002_2026-11-13.zip'
    with zipfile.ZipFile(zippath,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for f in files():z.write(f,str(Path(HERE.name)/f.relative_to(HERE)))
    with zipfile.ZipFile(zippath) as z:assert z.testzip() is None
    analysis=out/'region_reservation_002_Windows_analysis_2026-11-13';analysis.mkdir()
    for name in ('report.md','GATE.json','comparison.csv','comparison_windows.csv','metrics.json','event_index.json','shifted_receiver_certificate.json','domain_checks.json','shifted_fixture_checks.json','self_checks.json','second_round_rejection_checks.json','preservation_check.json','executed_source_versions.json','versions.json','requirements.txt','plan.json','work_queue.json','changes.patch','HANDOFF.md','relocation_check.json','DELIVERY_MANIFEST.json'):shutil.copy2(HERE/name,analysis/name)
    for case in ('opposite','same_direction'):
        src=HERE/'results'/case;dest=analysis/case;dest.mkdir()
        for name in ('config.json','summary.json','events.json','execution.jsonl.gz','independent_acceptance.json'):shutil.copy2(src/name,dest/name)
        (dest/'execution.jsonl').write_bytes(gzip.decompress((src/'execution.jsonl.gz').read_bytes()))
        for name in ('local_socp_round_1','local_socp_round_2','second_round_rejections','version_rejection'):shutil.copytree(src/name,dest/name)
    for f in (tarpath,zippath):
        f.with_suffix(f.suffix+'.sha256').write_text(sha(f)+'  '+f.name+'\n');shutil.copy2(f,analysis/f.name);shutil.copy2(f.with_suffix(f.suffix+'.sha256'),analysis/(f.name+'.sha256'))
    (analysis/'先读我_Windows.txt').write_text('区域预约002：两车重叠时连续两拍真实重新优化\n\n先读report.md、GATE.json、comparison_windows.csv。四次新求解，两个用例均连续两次受理，并完成2802拍和独立验收。状态为Solved一次、AlmostSolved三次，按完整证书规则受理。\n第一轮787、第二轮788。新域中心后移0.1m，接收/K从819变为820；若第二轮损坏则保留第一轮实际尾段和K819。12个复制拒绝分支均实际执行33拍旧尾段并独立通过。\nmetrics.json的34拍实际收益不重复累计两个32拍预测窗。event_index和内容对象绑定实际联合状态/邻车预测/原始数值及共同提交。\nJSONL已解压；B车实际航向=pi+theta，Q=1e20，速度有符号。精确证据以JSON分数/区间为准，Excel分数列请作为文本导入。\nZIP包含完整独立复核源码/原始对象/冻结依赖。默认verify_delivery校验已通过的完整轨迹证据并新复算两轮source/前缀；--full-replay才重算完整2802拍轨迹。\n优先队列1已完成；2倒车SOCP、3更多顺序组合仍待推进。历史和论文保留，未做正式批量或Git提交。\n',encoding='utf-8-sig')
    hashes={str(f.relative_to(analysis)):dict(bytes=f.stat().st_size,sha256=sha(f)) for f in sorted(analysis.rglob('*')) if f.is_file()};write(analysis/'SHARED_FILES_SHA256.json',hashes)
    shared=Path('/media/sf_DMPC_Theory')/analysis.name;assert not shared.exists();shutil.copytree(analysis,shared)
    for name,item in hashes.items():assert sha(shared/name)==item['sha256'],name
    assert sha(shared/'SHARED_FILES_SHA256.json')==sha(analysis/'SHARED_FILES_SHA256.json')
    print(json.dumps(dict(passed=True,shared_directory=str(shared),shared_files_verified=len(hashes)+1,zip_sha256=sha(zippath),zip_bytes=zippath.stat().st_size,tar_sha256=sha(tarpath),tar_bytes=tarpath.stat().st_size,relocation_passed=True),ensure_ascii=False,indent=2))
if __name__=='__main__':main()
