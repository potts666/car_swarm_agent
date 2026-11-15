"""Portable, deduplicated archives plus a Windows analysis export."""
import os,sys,subprocess,shutil,tarfile,tempfile,zipfile
from common import *


def files():return [f for f in sorted(HERE.rglob('*')) if f.is_file() and '__pycache__' not in f.parts and 'delivery' not in f.relative_to(HERE).parts]
def manifest():write(HERE/'DELIVERY_MANIFEST.json',dict(files={str(f.relative_to(HERE)):dict(bytes=f.stat().st_size,sha256=sha(f)) for f in files() if f!=HERE/'DELIVERY_MANIFEST.json'},source_commit=read(HERE/'plan.json')['source_commit']))
def tarball(path):
    seen={}
    with tarfile.open(path,'w:gz') as tar:
        for f in files():
            name=str(Path(HERE.name)/f.relative_to(HERE));info=tar.gettarinfo(str(f),arcname=name);key=sha(f)
            if key in seen:info.type=tarfile.LNKTYPE;info.linkname=seen[key];info.size=0;tar.addfile(info)
            else:
                seen[key]=name
                with f.open('rb') as stream:tar.addfile(info,stream)

def main():
    output=HERE/'delivery'/'final_002';assert not output.exists();output.mkdir(parents=True);manifest()
    with tempfile.TemporaryDirectory(prefix='dmpc-region-relocation-') as tmp:
        archive=Path(tmp)/'candidate.tar.gz';tarball(archive)
        with tarfile.open(archive) as tar:tar.extractall(tmp,filter='data')
        env={**os.environ,'PYTHONPATH':'/tmp/dmpc-p0-deps','OPENBLAS_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1'}
        result=subprocess.run([sys.executable,str(Path(tmp)/HERE.name/'verify_delivery.py')],cwd=tmp,env=env,capture_output=True,text=True,timeout=180)
        (output/'relocation_stdout.log').write_text(result.stdout);(output/'relocation_stderr.log').write_text(result.stderr);assert result.returncode==0,(result.returncode,result.stderr)
        write(HERE/'relocation_check.json',dict(passed=True,exit_code=0,solver_calls=0,mode='content-checked full-trajectory evidence plus freshly recomputed derivative/numeric/33-tick prefix checks; not a second full-task replay',result=json.loads(result.stdout),temporary_root=tmp))
    manifest();tpath=output/'DMPC_region_reservation_001_2026-11-13.tar.gz';tarball(tpath);zpath=output/'DMPC_region_reservation_001_2026-11-13.zip'
    with zipfile.ZipFile(zpath,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for f in files():z.write(f,str(Path(HERE.name)/f.relative_to(HERE)))
    with zipfile.ZipFile(zpath) as z:assert z.testzip() is None
    analysis=output/'region_reservation_001_Windows_analysis_2026-11-13_v2';analysis.mkdir()
    for name in ('report.md','GATE.json','comparison.csv','comparison_windows.csv','metrics.json','event_index.json','CERTIFICATE_BUNDLE.json','rational_inputs.json','domain_checks.json','uniform_quantization_check.json','integration_bounds.json','feedback_checks.json','flow_images.json','flow_checks.json','self_checks.json','source_binding_checks.json','native_rejection_checks.json','preservation_check.json','versions.json','requirements.txt','pending_parameters.json','changes.patch','HANDOFF.md','relocation_check.json','DELIVERY_MANIFEST.json'):shutil.copy2(HERE/name,analysis/name)
    for case in ('opposite','same_direction'):
        src=HERE/'results'/case;dest=analysis/case;dest.mkdir()
        for name in ('config.json','events.json','summary.json','execution.jsonl.gz','independent_acceptance.json','trajectory_display.csv','flow_receivers.jsonl.gz'):shutil.copy2(src/name,dest/name)
        (dest/'execution.jsonl').write_bytes(gzip.decompress((src/'execution.jsonl.gz').read_bytes()))
        for name in ('local_socp','version_rejection','native_K_rejection'):shutil.copytree(src/name,dest/name)
    for f in (tpath,zpath):
        checksum=f.with_suffix(f.suffix+'.sha256');checksum.write_text(sha(f)+'  '+f.name+'\n');shutil.copy2(f,analysis/f.name);shutil.copy2(checksum,analysis/checksum.name)
    (analysis/'先读我_Windows.txt').write_text('区域预约001工程验证\n\n先读report.md和GATE.json；comparison_windows.csv可用Excel打开，分数列请按文本导入。\n两条四车完整任务均2802拍、预算0，资源外出口/准备两车重叠，核心资源保持排他。局部求解各一次：Solved与AlmostSolved，均通过完整证书实际采用。\n注意：9.49e-12m是节点参考差；连续全时域量化界为uniform_quantization_check.json中的3.23e-10m，预留1e-8m。\nflow_images.json与flow_receivers描述保留流像；metrics区分逻辑时间、墙钟、预测与实际输入代价。\n每个case有实际JSONL、独立验收、绘图CSV、原始求解设置/输出、版本及新K拒绝分支。trajectory_display的浮点中值仅供画图；精确区间在JSONL/CSV的Q整数字段中，Q=1e20，B车实际航向=pi+theta。速度有符号，负值为倒车。\n完整ZIP含源代码、原理论材料、内容对象、执行源码快照和复核工具。v2补齐旧协议传递导入的OSQP版本；本轮只调用Clarabel两次，数值结果与v1相同。objects中也保留故意损坏提案，是否实际认可须按event_index与提交ID判断。\n默认verify_delivery复用已内容校验的完整独立验收，并重算两处源与33拍前缀；加--full-replay才重新跑两个2802拍独立验收。\nDELIVERY_MANIFEST对应ZIP内region_reservation_001目录，SHARED_FILES_SHA256对应本分析文件夹。历史实验与论文保留，无Git提交和正式批量。\n',encoding='utf-8-sig')
    hashes={str(f.relative_to(analysis)):dict(bytes=f.stat().st_size,sha256=sha(f)) for f in sorted(analysis.rglob('*')) if f.is_file()};write(analysis/'SHARED_FILES_SHA256.json',hashes)
    shared=Path('/media/sf_DMPC_Theory')/analysis.name;assert not shared.exists();shutil.copytree(analysis,shared)
    for name,item in hashes.items():assert sha(shared/name)==item['sha256'],name
    assert sha(shared/'SHARED_FILES_SHA256.json')==sha(analysis/'SHARED_FILES_SHA256.json')
    print(json.dumps(dict(passed=True,local_delivery=str(output),shared_directory=str(shared),shared_files_verified=len(hashes)+1,tar_sha256=sha(tpath),tar_bytes=tpath.stat().st_size,zip_sha256=sha(zpath),zip_bytes=zpath.stat().st_size,relocation_passed=True),ensure_ascii=False,indent=2))
if __name__=='__main__':main()
