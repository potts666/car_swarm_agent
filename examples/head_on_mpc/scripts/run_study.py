#!/usr/bin/env python3
"""Run only the head-on cases and generate the complete simulation log report."""
import argparse
from datetime import datetime, timezone, timedelta
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys

import cv2
import matplotlib
import numpy as np
import scipy

from head_on_core import CASES, run_case, write_json
from render import plot, export_video


ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_files():
    for path in sorted(ROOT.rglob('*')):
        relative = path.relative_to(ROOT)
        if path.is_file() and not any(p in ('results','build','install','log','__pycache__','.git') for p in relative.parts):
            yield path


def provenance(command):
    def git(*args):
        result = subprocess.run(['git','-C',str(ROOT),*args],capture_output=True,text=True)
        return result.stdout.strip() if result.returncode==0 else None
    files = {str(p.relative_to(ROOT)):sha(p) for p in source_files()}
    return {'generated_at':datetime.now(timezone(timedelta(hours=8))).isoformat(),
            'upstream_git_head_at_run':git('rev-parse','HEAD'),
            'parent_worktree_dirty':bool(git('status','--porcelain')),
            'source_sha256':files,
            'source_tree_sha256':hashlib.sha256(json.dumps(files,sort_keys=True).encode()).hexdigest(),
            'version_note':'Git HEAD is the parent revision; this extracted package is uncommitted until the user commits it. The file hashes identify the executed source.',
            'python':sys.version,'numpy':np.__version__,'scipy':scipy.__version__,
            'matplotlib':matplotlib.__version__,'opencv':cv2.__version__,
            'platform':platform.platform(),'command_argv':command,
            'openblas_threads':os.environ.get('OPENBLAS_NUM_THREADS'),
            'randomness':'none; deterministic initial conditions',
            'video':'OpenCV FFmpeg backend, H.264 avc1, 1280x720, 20fps, 2x simulation playback'}


def reports(output):
    version = json.loads((output/'provenance.json').read_text())
    validation = json.loads((output/'validation.json').read_text())
    ros_path = output/'rviz_validation.json'
    ros = json.loads(ros_path.read_text()) if ros_path.exists() else None
    ros_text = ('真实ROS发布/订阅验证通过，三种策略最终车辆位置与原始日志一致，收到相机TF。'
                if ros and all(r['passed'] for r in ros) else
                '提供可构建的ROS2回放入口；本结果尚未附带本包的ROS发布订阅验证记录。')
    rows = []
    for name in CASES:
        folder = output/name
        cfg = json.loads((folder/'config.json').read_text())
        s = json.loads((folder/'summary.json').read_text())
        video = json.loads((folder/'video.json').read_text())
        settings = cfg['settings']
        rows.append(f"| [{name}]({name}/report.md) | {s['status']} | {s['simulation_seconds']:.1f} | "
                    f"{s['minimum_sampled_sat_gap_m']:.3f} | {s['B_reverse_distance_m']:.2f} | [MP4]({name}/replay.mp4) |")
        text = f'''# 一次模拟日志总结：{name}

生成时间：{version['generated_at']}。本目录中的数据、图片、视频和RViz回放使用同一份执行日志。

## 1. 提交版本

上游提交 `{version['upstream_git_head_at_run']}`；本提取目录尚待用户最终提交。不能把上游提交号当作新案例已提交的证明。执行源码指纹 `{version['source_tree_sha256']}`，逐文件哈希见 [provenance.json](../provenance.json)；完整源码随本案例目录进入Git版本控制，不再复制无关项目源码。

## 2. 地图与初始条件

主通道x∈[-22,22]m、y∈[-1.4,1.4]m；旁侧暂停区x∈[-3,3]m，入口退让区x∈[7,13]m，两区上界y=4.4m。矩形车身含余量，长{settings['length']}m、宽{settings['width']}m，状态原点为车身中心。

A初始(x,vx)=(-12,1)，朝东；B初始(6,-0.8)，朝西。A/B目标位置分别为18/-18m。A等待线x=-6m；暂停位置x=0m；倒车前制动目标x=5.4m；后退目标x=10m。精确配置见 [config.json](config.json)。

## 3. 控制周期、模型与约束

控制周期{settings['dt']}s、预测{settings['horizon']}步/{settings['dt']*settings['horizon']:.1f}s；模型和执行约束采样间隔{settings['dt']/settings['substeps']}s。采用精确零阶保持纵向双积分，优化变量为每步加速度，仅执行第一项。

代价：0.6×位置误差平方和 + 0.4×速度平方和 + 0.15×加速度平方和 + 0.2×加速度变化平方和，变化相对于上一实际控制；统一缩放不改变最优解。SLSQP使用解析梯度，速度绝对值≤2m/s、加速度绝对值≤1m/s²，同道预测中心间距≥5.2m，另检查目标不越界与方向。执行时独立检查矩形车身间隙、道路和纵向限制；验收间隙≥0.08m，数值容差1e-5。

高层规则指定A优先与本次策略。旁侧进出用6秒五次平滑横向占用变化，**未验证自行车转向可达性**。本例不是自主策略选择、连续时间安全证明或实车实验。

## 4. 通信设置

进程内同步双向预测交换，每轮冻结上一轮数据；正常包龄0.4s，初始包龄0。字段为时间戳、绝对时间数组、x/vx/y预测；尾部补一个周期零加速度。无丢包、延迟抖动或周期内反复协调。这里是离线求解的仿真通信，不是ROS实车通信。ROS只用于后处理回放。

## 5. 原始日志与图片

本次结果 **{s['status']}**；仿真时长{s['simulation_seconds']:.1f}s；到达时间{s['arrival_times_s']}；最小采样SAT间隙{s['minimum_sampled_sat_gap_m']:.6f}m；B倒车{s['B_reverse_distance_m']:.3f}m；最大执行采样违背{s['max_executed_sampled_violation']:.3g}；求解{s['solve_count']}次，拒绝{s['rejected_solves']}次。

`blocked`表示本规则和观察时间内未完成任务，不代表通行成功或一般不可行性证明。耗时依机器变化，不能由本例证明实时性。

[执行状态 actual.csv](actual.csv) · [全部控制与求解记录 solves.json](solves.json) · [预测交换 predictions.jsonl](predictions.jsonl) · [模式事件 events.json](events.json) · [摘要 summary.json](summary.json)

![地图与运动曲线](overview.png)

## 6. RViz2 案例回放

按项目[README](../../../README.md)独立构建后运行：

```bash
ros2 launch head_on_mpc replay.launch.py case:={name}
```

默认加载随包安装的本次结果，也可用`study_dir:=/absolute/path/to/run_001`指定结果目录。`playback_rate:=2.0`双倍速，`loop:=false`停留最终帧。显示地图、车辆、车头方向、已执行/预测轨迹和阶段。{ros_text} 当前环境无桌面显示，未将消息验证表述为已验证GUI渲染画面。

## 7. 可播放视频

[打开或下载H.264 MP4](replay.mp4)：1280×720、{video['fps']:g}fps、{video['playback_rate']:g}倍速，视频{video['video_seconds']:.2f}s，其中包含结尾停留。编码后逐帧解码{video['decoded_frames']}帧，核验记录见 [video.json](video.json)。视频从原始日志绘制，**不是RViz录屏**；它与RViz共用`recording.py`读取状态，横向抽象与实验一致。

[![点击查看视频](video_poster.png)](replay.mp4)

## 验证与完整性

离线测试退出码：{validation['returncode']}，见 [测试日志](../tests.log)。本包原始文件校验见 [manifest.json](../manifest.json)，可执行`python3 scripts/verify_bundle.py`独立检查源码/工件哈希、动力学重放、采样安全、通信时间戳和视频可解码性。
'''
        (folder/'report.md').write_text(text)
    (output/'report.md').write_text(f'''# 方向一：单通道双车相向通行最小案例

本次以同一地图、初值比较“主道停车”“旁侧暂停区”“倒车退让”。每个子目录都是按七项标准保存的一次模拟日志总结。方向二的插入案例不包含在本次交付中。

版本：上游`{version['upstream_git_head_at_run']}` + 提取源码SHA-256 `{version['source_tree_sha256']}`。最终Git提交由用户完成。

| 模拟报告 | 结果 | 仿真时长s | 最小采样SAT间隙m | B倒车m | 视频 |
|---|---|---:|---:|---:|---|
{chr(10).join(rows)}

本组固定初值下，旁侧暂停区策略比退回入口更快。只在主道停车的对照保持间距但没有完成通行。这些结果只验证纵向空间和时序，不能据此声称已自主选择策略或验证真实转向可达性。

标准项目：提交版本、地图与初始条件、控制周期、通信设置、原始日志与图片、RViz2回放、可播放MP4；三份模拟报告均逐项列明。

{ros_text} 详见 [ROS验证记录](rviz_validation.json)（完成验证后提供）、[离线验证](validation.json)、[来源与依赖](provenance.json)。

代码只有本方向的纵向优化、状态机、共享日志读取、报告/视频导出、RViz2回放与相关测试。没有旧地图规划器、C++原生二进制、其他方向或旧演示。
''')


def seal(output):
    original = json.loads((output/'provenance.json').read_text())['source_sha256']
    current = {str(p.relative_to(ROOT)):sha(p) for p in source_files()}
    if current != original:
        raise RuntimeError('Source changed since execution; create a new run instead of refreshing its version identity')
    paths = [*source_files(),*(p for p in sorted(output.rglob('*'))
              if p.is_file() and p.name!='manifest.json' and '__pycache__' not in p.parts)]
    write_json(output/'manifest.json',{str(p.relative_to(ROOT)):sha(p) for p in paths})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'results/run_001')
    parser.add_argument('--config',type=Path,default=ROOT/'config/scenarios.json')
    parser.add_argument('--refresh-report',action='store_true',help='refresh reports/manifest after ROS validation, preserving all raw data')
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT):
        parser.error('Keep output under this package, e.g. results/rerun_001, so report links and checksums are portable')
    if args.refresh_report:
        reports(output)
        seal(output)
        return 0
    output.mkdir(parents=True,exist_ok=False)
    write_json(output/'provenance.json',provenance(sys.argv))
    test_command = [sys.executable,'-m','unittest','discover','-s',str(ROOT/'tests'),'-p','test_*.py']
    with (output/'tests.log').open('w') as stream:
        test = subprocess.run(test_command,stdout=stream,stderr=subprocess.STDOUT)
    write_json(output/'validation.json',{'argv':test_command,'returncode':test.returncode,'log':'tests.log'})
    if test.returncode:
        raise RuntimeError(f'Tests failed: {output}/tests.log')
    scene = json.loads(args.config.read_text())
    summaries = {}
    for name in CASES:
        directory = output/name
        summaries[name] = run_case(name,directory,scene)
        plot(directory)
        export_video(directory)
    write_json(output/'study_summary.json',summaries)
    reports(output)
    seal(output)
    print(f'Report: {output}/report.md',flush=True)
    return 0 if all(s['expectation_met'] for s in summaries.values()) else 1


if __name__=='__main__':
    raise SystemExit(main())
