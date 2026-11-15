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
    with tempfile.TemporaryDirectory(prefix='region-reverse-v2-relocation-') as tmp:
        packed=Path(tmp)/'candidate.tar.gz';archive(packed)
        with tarfile.open(packed) as tar:tar.extractall(tmp,filter='data')
        env={**os.environ,'PYTHONPATH':'/tmp/dmpc-p0-deps','OPENBLAS_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1'}
        run=subprocess.run([sys.executable,str(Path(tmp)/HERE.name/'verify_delivery.py')],cwd=tmp,env=env,capture_output=True,text=True,timeout=180)
        (out/'relocation_stdout.log').write_text(run.stdout);(out/'relocation_stderr.log').write_text(run.stderr);assert run.returncode==0,(run.returncode,run.stderr)
        write(HERE/'relocation_check.json',dict(passed=True,exit_code=0,solver_calls=0,mode='content checked full acceptance plus fresh one actual source/prefix per side; no full-task repeat',result=json.loads(run.stdout)))
    manifest();tarpath=out/'DMPC_region_reservation_003_reverse_2026-11-13.tar.gz';archive(tarpath);zippath=out/'DMPC_region_reservation_003_reverse_2026-11-13.zip'
    with zipfile.ZipFile(zippath,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for f in files():z.write(f,str(Path(HERE.name)/f.relative_to(HERE)))
    with zipfile.ZipFile(zippath) as z:assert z.testzip() is None
    analysis=out/'region_reservation_003_reverse_Windows_analysis_2026-11-13';analysis.mkdir()
    for name in ('report.md','method_spec.md','method_spec_initial.md','bounds_proof.md','GATE.json','comparison.csv','energy_windows.csv','metrics.json','event_index.json','domain_checks.json','preservation_check.json','materials_check.json','executed_source_versions.json','environment.json','versions.json','requirements.txt','plan.json','pre_registered.json','regression_sources.json','v1_audit.json','work_queue.json','changes.patch','HANDOFF.md','relocation_check.json','DELIVERY_MANIFEST.json'):
        shutil.copy2(HERE/name,analysis/name)
    for side in ('A','B'):
        src=HERE/'results'/side;dest=analysis/side;shutil.copytree(src,dest)
        (dest/'execution.jsonl').write_bytes(gzip.decompress((src/'execution.jsonl.gz').read_bytes()))
        (dest/'rejection/execution.jsonl').write_bytes(gzip.decompress((src/'rejection/execution.jsonl.gz').read_bytes()))
    for name in ('rule_checks_initial.json','rule_checks.json','rule_checks_rebuilt.json','source_range_checks.json','zero_successor_checks.json','rejection_checks.json','solver_calls.json'):
        shutil.copy2(HERE/'results'/name,analysis/name)
    for f in (tarpath,zippath):
        f.with_suffix(f.suffix+'.sha256').write_text(sha(f)+'  '+f.name+'\n');shutil.copy2(f,analysis/f.name);shutil.copy2(f.with_suffix(f.suffix+'.sha256'),analysis/(f.name+'.sha256'))
    (analysis/'先读我_Windows.txt').write_text('区域预约003：v2统一生成与倒车SOCP\n\n先读report.md、GATE.json、method_spec.md、comparison.csv。新数值调用2次：A AlmostSolved、B Solved，均经完整证书受理；A/B源首拍实际改变，负速段各9拍变化；各完成2802拍与独立8子段验收。\n33拍统一实际区间输入平方积分下降约0.0525714%，固定时长不变；32/33拍指标是嵌套窗口，不能累加。\n首次后移域生成失败保留在rule_checks_initial及完整内容对象；修正为中点参考后续与先覆盖旧区间的固定裕量分配后7/7通过。16损坏提案拒绝，每侧真实执行1条99拍旧尾段通过原接收/K。\n完整ZIP含代码、最小补丁、全部去重引用对象、原始x/s/z和日志。默认verify_delivery内容核对已完成的完整验收并新复算两处源/前缀；--full-replay才重算整条物理轨迹，无新求解。\n执行JSONL已解压；Q=1e20，B车世界航向=pi+theta，速度保留符号；CSV分数列按文本导入Excel。源码级环境配置见environment.json/requirements.txt。\n历史3042文件未变，论文无修改、Git无提交、正式批量未启动；更多顺序和组合仍排队。\n',encoding='utf-8-sig')
    hashes={str(f.relative_to(analysis)):dict(bytes=f.stat().st_size,sha256=sha(f)) for f in sorted(analysis.rglob('*')) if f.is_file()};write(analysis/'SHARED_FILES_SHA256.json',hashes)
    shared=Path('/media/sf_DMPC_Theory')/analysis.name;assert not shared.exists();shutil.copytree(analysis,shared)
    for name,item in hashes.items():assert sha(shared/name)==item['sha256'],name
    assert sha(shared/'SHARED_FILES_SHA256.json')==sha(analysis/'SHARED_FILES_SHA256.json')
    print(json.dumps(dict(passed=True,shared_directory=str(shared),shared_files_verified=len(hashes)+1,zip_sha256=sha(zippath),zip_bytes=zippath.stat().st_size,tar_sha256=sha(tarpath),tar_bytes=tarpath.stat().st_size,relocation_passed=True),ensure_ascii=False,indent=2))
if __name__=='__main__':main()
