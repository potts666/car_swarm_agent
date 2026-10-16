#!/usr/bin/env python3
"""Generate complete seven-part simulation reports for the two merging cases."""
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

from merging_core import CASES, run_case, write_json
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
    ros_text = ('两个案例的真实ROS发布订阅检查通过：三车末态、轨迹、状态标签和相机TF与日志一致。'
                if ros and all(check['passed'] for check in ros) else
                '已提供独立ROS2回放入口，本次结果暂未附带本包的发布订阅验证记录。')
    table = []
    for name in CASES:
        directory = output/name
        cfg = json.loads((directory/'config.json').read_text())
        s = json.loads((directory/'summary.json').read_text())
        video = json.loads((directory/'video.json').read_text())
        settings = cfg['settings']
        table.append(f"| [{name}]({name}/report.md) | {s['status']} | {s['admitted_at_s']} | "
                     f"{s['merge_completed_at_s']} | {s['simulation_seconds']:.1f} | "
                     f"{s['B_minimum_speed_m_s']:.3f} | [MP4]({name}/replay.mp4) |")
        (directory/'report.md').write_text(f'''# 一次模拟日志总结：{name}

生成时间：{version['generated_at']}。本次报告、图片、视频及RViz2回放使用同一份原始执行日志。

## 1. 提交版本

运行时父仓库提交`{version['upstream_git_head_at_run']}`；提取目录尚待用户最终提交。该提交号不是新增merging代码已经提交的证明。实际执行源码指纹为`{version['source_tree_sha256']}`，逐文件SHA-256、依赖版本和实际命令见 [provenance.json](../provenance.json)。最终Git提交由用户完成，不能提前编造提交号。

## 2. 地图与初始条件

道路x∈[{cfg['road']['xmin']},{cfg['road']['xmax']}]m、y∈[{cfg['road']['ymin']},{cfg['road']['ymax']}]m。主道中心y={cfg['main_lane_y']}m，邻道中心y={cfg['merge_lane_y']}m。三车均朝东、坐标原点为车身中心；含余量矩形车身{settings['length']}×{settings['width']}m。

初始(x,vx)：A={cfg['initial_states_x_v']['A']}，B={cfg['initial_states_x_v']['B']}，C={cfg['initial_states_x_v']['C']}。主道A在B前方，C从邻道插入两者之间；目标顺序A-C-B。巡航速度{cfg['cruise_speed']}m/s；t={cfg['request_time_s']}s提出请求，横向动作持续{settings['lateral_seconds']}s；成功后至少观察到{cfg['observation_time_s']}s并检查车速恢复与最终顺序。精确输入见 [config.json](config.json)。

## 3. 控制周期、模型与优化

控制周期{settings['dt']}s，{settings['horizon']}步/{settings['horizon']*settings['dt']:.1f}s预测时域，执行与约束采样间隔{settings['dt']/settings['substeps']}s。纵向状态[x,vx]，控制为加速度，采用精确零阶保持双积分。各车独立用SLSQP求解，提供解析梯度，只执行第一项，再以实际积分状态重新优化。

代价是2×巡航速度误差平方和 + 0.15×加速度平方和 + 0.2×相对于上一实际控制的加速度变化平方和。协作激活后，B和C另外加入1.2×跟驰间距误差平方和，期望中心间距为`{settings['length']}+{settings['margin']}+{settings['headway']}*v+{cfg['following_extra_gap_m']}`m。整体缩放不改变最优解。

约束：0≤vx≤{settings['max_speed']}m/s、|a|≤{settings['max_acceleration']}m/s²；基础前车中心间距≥{settings['length']+settings['margin']}m，协作跟驰激活后增加{settings['headway']}s速度时距。请求后协作配置采用A→C→B虚拟前车链；不配合配置下B仍以前方A为约束对象，不主动为C开间距。

集中许可检查固定A-C-B顺序，在完整{settings['lateral_seconds']}s动作时域的31个时刻要求每对中心间距≥`length+margin+headway*v_behind+{cfg['admission_buffer_m']}`；不满足则C保持邻道。条件满足后用五次平滑函数预设横向占用，**未优化转角、未验证自行车换道可达性**。本例不是自主顺序协商或连续时间安全证明。

## 4. 通信设置

进程内同步全连接预测数据，每轮冻结上一轮收件箱；正常包龄{settings['dt']}s、初始化包龄0。数据包含时间戳、绝对时间数组和x/vx/y预测；尾部补一周期零加速度以覆盖下一轮时域。没有丢包、抖动、实网传输或同周期迭代协商。每车求解只使用其当前指定前车的固定预测；`solves.json`保存前车身份及收到的时间戳。ROS用于后处理回放，不是本次控制通信通道。

## 5. 原始日志与图片

本次结果 **{s['status']}**。许可时刻{s['admitted_at_s']}s，完成并线时刻{s['merge_completed_at_s']}s，总观察时长{s['simulation_seconds']:.1f}s；B最低速度{s['B_minimum_speed_m_s']:.6f}m/s；最小采样车身SAT间隙{s['minimum_sampled_sat_gap_m']:.6f}m；最大执行采样约束违背{s['max_executed_sampled_violation']:.3g}；求解{s['solve_count']}次，拒绝{s['rejected_solves']}次。

`None`表示没有获得许可或完成并线。`blocked`表示指定策略和观察时间内并线未完成；负对照符合预期不等于并线成功，也不能证明所有策略均不可行。耗时依环境变化，不能据此宣称实时运行。

[实际执行 actual.csv](actual.csv) · [优化与前车记录 solves.json](solves.json) · [预测交换 predictions.jsonl](predictions.jsonl) · [许可检查 admission_checks.json](admission_checks.json) · [阶段事件 events.json](events.json) · [摘要 summary.json](summary.json)

![地图与执行曲线](overview.png)

## 6. RViz2 案例回放

按[README](../../../README.md)独立构建后：

```bash
ros2 launch merging_mpc replay.launch.py case:={name}
```

默认加载随包安装的本次日志。可用`study_dir:=/absolute/path/to/run_001`切换结果，`playback_rate:=2.0`双倍速、`loop:=false`停止于最后。蓝色A、橙色B、绿色C，白条表示车头；粗线为已执行轨迹、细线为预测轨迹。{ros_text} 当前环境无桌面，未将消息检查表述为已经检查GUI渲染画面。

## 7. 可播放视频

[打开或下载H.264 MP4](replay.mp4)，1280×720，{video['fps']:g}fps，{video['playback_rate']:g}倍速，视频{video['video_seconds']:.2f}s，包含结尾停留。已逐帧解码{video['decoded_frames']}帧，详见 [video.json](video.json)。视频从原始日志绘制，**不是RViz录屏**；它与RViz共用`recording.py`，相机随三车移动。

[![点击查看视频](video_poster.png)](replay.mp4)

## 验证与完整性

六项离线测试退出码{validation['returncode']}，见[测试日志](../tests.log)。`verify_bundle.py`独立重放执行动力学、检查车身/道路/速度/加速度、重算全部许可决策、核对虚拟前车和包龄、重放预测控制并解码视频。源码与工件校验清单见[manifest.json](../manifest.json)，独立审计见[verification.json](../verification.json)（封存时生成）。
''')
    (output/'report.md').write_text(f'''# 方向二：单C车插入A、B之间的merging最小案例

同一初值运行“不配合开间距”和“配合开间距”两个配置。目标是让邻道C进入主道A-B之间，最终形成A-C-B。每个子目录保存一次完整的七项标准模拟日志总结。

版本：运行时父提交`{version['upstream_git_head_at_run']}` + 执行源码SHA-256 `{version['source_tree_sha256']}`。新增目录尚待用户最终Git提交。

| 模拟报告 | 状态 | 获准时刻s | 完成并线s | 观察时长s | B最低速度m/s | 视频 |
|---|---|---:|---:|---:|---:|---|
{chr(10).join(table)}

固定巡航的负对照没有满足许可条件，C保持邻道。协作配置在请求后将B的前车从A切换为C，通过间距代价/约束让B主动减速，留出间距后再执行预设横向动作。并线完成后继续观察巡航恢复及A-C-B顺序。

结论限于本地图、初值、纵向模型和集中许可规则；横向换道轨迹尚未通过自行车运动学验证，无分布式协商或连续安全保证。下一步可扫描初始间距、请求时刻与通信扰动，再验证横向可达性。

七项标准：提交版本、地图与初始条件、控制周期、通信设置、原始日志与图片、RViz2回放、可播放MP4。两份模拟报告均逐项给出。

{ros_text} 证据见[ROS验证](rviz_validation.json)、[离线验证](validation.json)、[独立审计](verification.json)、[源码与依赖](provenance.json)。完整封存前，尚未运行的验证文件不会被当作已通过。

本目录独立保留三车纵向优化、许可状态机、共享日志读取、图片/视频/报告导出、ROS回放及相关测试。
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
