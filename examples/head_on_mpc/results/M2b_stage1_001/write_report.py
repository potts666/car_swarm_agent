import json
from pathlib import Path

OUT=Path(__file__).resolve().parent


def ms(value):return f'{value*1000:.2f}'


if __name__=='__main__':
    data=json.loads((OUT/'measurements.json').read_text());runs=data['runs']
    full=runs['full_L_FIFO'];status=json.loads((OUT/'full_status.json').read_text())
    assert full['validation']['actual_full_acceptance'] and status['actual_full_acceptance']
    assert data['motion_matches_historical']
    resource=status['resource'];a=runs['A_uncached'];b=runs['B_cached'];plain=runs['B_no_profiler']
    lines=['# M2b 第一阶段：核查、计时与重复验证优化', '',
        '**已完成本阶段的一次完整L-FIFO回归与独立验收；第二阶段真实并行、第三阶段连续转向尚未实现。**', '',
        f"新运行四车完成时间{full['summary']['completion_sim_s']}秒，882个实际提交周期，独立原物理/跨零/协议/局部信息边界验收通过。实际状态、输入、owner与许可版本的逐周期逻辑哈希与历史L-FIFO/r1完全一致；输入平方积分{full['actual_signature']['input_square_integral']:.9f}。固定场景JSON与原冻结场景完全相同。", '',
        '## 改动与边界', '',
        '仅在维护代码引入本端验证复用、可选细分计时和显式有界前缀入口。每端缓存每次begin清空，最多16项；命中前仍核对重新计算的计划内容ID、报告、上下文、完整场景与验证源码版本。返回状态副本，调用者不能污染缓存。ACK仍由每端独立检查四计划组合的六对旋转车身间距，屏障与许可提交不变。没有更改QP、热启动、输入修复、物理门限、路线、调度或导数。', '',
        '旧日志的total_cycle_wall_s语义保留；新增control_executable_wall_s、诊断/记录耗时、diagnostic-end的full_cycle_wall_s，以及下一轮开始后补记的start_to_next_start_wall_s。相邻起点统计包括计时器收尾与进度输出；最终无输入完成出口单列。完整周期指标不能只看控制器函数。', '',
        '## 完整运行结果', '',
        '|指标|本轮缓存L-FIFO|历史L-FIFO/r1锚点|', '|---|---:|---:|',
        f"|车辆运动时间|{full['summary']['completion_sim_s']} s|176.4 s|",
        f"|在线墙钟|{full['summary']['online_wall_s']:.3f} s|583.621 s|",
        f"|导出墙钟（在线之后）|{full['summary']['export_wall_s']:.3f} s|不用于在线速度结论|",
    ]
    old={'p50':650.1313275000484,'p95':725.895634849894,'p99':912.9809695798208,'max':1233.8271730000088}
    t=full['consecutive_start_s']
    for q in ['p50','p95','p99','max']:lines.append(f'|完整周期 {q.upper()}|{ms(t[q])} ms|{old[q]:.2f} ms|')
    lines += [f"|完整周期超200ms|{t['over_200ms']}/{t['count']}|882/882|",
              f"|本车调用超0.5s|{full['local_over_500ms']}|历史完整任务通过|",
              f"|应用发送字节|{full['application_bytes']:,}|715,916,537|", '',
        '这里的历史/新运行不是同一时间的性能配对，不把差异全部归给缓存；历史缺少可复原的资源/调度轨迹。在线计时沿用原入口，不含Wire创建/初始DDS发现，整个驱动的资源口径另列。计算期间仍冻结仿真时间；整周期超过200ms的问题未关闭。', '',
        '## 小型配对和计时自身影响', '',
        f"前三组各6周期、真实五节点DDS、同一实际轨迹哈希，三组独立前缀验收均通过。含密集计时：未缓存平均{ms(a['cycle_s']['mean'])}ms，缓存平均{ms(b['cycle_s']['mean'])}ms；缓存关闭密集计时为{ms(plain['cycle_s']['mean'])}ms。配对方向支持减少重复工作，但一次顺序小样本不是总体加速率。",
        f"计时开启/关闭缓存组的观测差为{ms(b['cycle_s']['mean']-plain['cycle_s']['mean'])}ms/周期，混合了探针和单次运行波动，不能全当精确探针开销。完整回归关闭密集探针，计时只作诊断。", '',
        '## 灰色/黄色内部是什么', '',
        '下表来自完整回归结束后另外两组各2周期的实测探针，不是倒推出旧882周期的349.56/178.72ms。灰色=末次final发送之前的阶段，扣除candidate_compute整个嵌套子树；黄色=末次final发送至首次commit发送，包含末次投递及剩余检查。每个格为毫秒及占该阶段的比例；exclusive类别互斥，inclusive父子不相加。每个提交周期都验证独占项之和等于根span。', '',
        '|互斥类别|灰色未缓存|灰色缓存|黄色未缓存|黄色缓存|','|---|---:|---:|---:|---:|']
    pa=runs['phase_probe_uncached'];pb=runs['phase_probe_cached']
    columns=[pa['grey_without_candidate_mean_s'],pb['grey_without_candidate_mean_s'],pa['phases_exclusive_mean_s']['after_last_final'],pb['phases_exclusive_mean_s']['after_last_final']]
    keys=set().union(*(d.keys() for d in columns));order=sorted(keys,key=lambda k:-sum(d.get(k,0.) for d in columns));selected=order[:9]
    names={'prediction_integration':'非线性预测积分','polygon_clip':'多边形裁剪','plan_validation':'单计划检查其他','pair_geometry':'旋转车身间距','dds_wait':'DDS等待/执行器派发','decoding':'JSON解码','copy':'显式深拷贝','body_geometry':'车身几何','encoding':'编码','cycle':'运行器未细分工作','plan_content_binding':'内容/版本绑定'}
    def cells(values):
        return '|'.join(f'{ms(v)} ({100*v/sum(d.values()):.1f}%)' for v,d in zip(values,columns))
    for k in selected:lines.append('|'+names.get(k,k)+'|'+cells([d.get(k,0.) for d in columns])+'|')
    lines.append('|其余互斥类别|'+cells([sum(v for k,v in d.items() if k not in selected) for d in columns])+'|')
    lines.append('|阶段合计|'+'|'.join(ms(sum(d.values())) for d in columns)+'|')
    lines += ['', '完整互斥分类、原始秒数、父子inclusive与调用次数保存在planning日志和measurements.json。reference_total/bicycle_total为包含构造、求解与后处理的父范围；matrix_solver还包括固定变量消元/缩放，探针另外实测osqp_setup与osqp_solve，不能把任一子项叫全部QP时间。', '',
        'DDS等待项包含调度与派发，解码另计，不能当作纯网络传播延迟。复制只统计显式clone入口，不冒称所有解释器内存复制。first commit之后的执行/记录和诊断另列；探针与记录本身也有成本。初始前缀不包含交接/终端全过程，其分布不能替代完整运行。', '',
        '## 资源、验收和未完成项', '',
        f"完整驱动（在线+导出+独立验收）的CPU用户/系统时间为{resource['user_cpu_s']:.3f}/{resource['system_cpu_s']:.3f}秒，进程最大RSS={resource['max_rss_kib']/1024:.1f}MiB，观测最大线程数={resource['max_observed_threads']}。采样线程也包括在资源中；DDS/后台线程不是四车并行求解证明。CPU亲和与负载见environment.json和full_status.json。prefix的单次事后/proc观察未抓到存活进程，未拿它编造峰值。", '',
        '受影响测试：缓存变更/旧版本/修复后未更新/返回数组污染/缺计划/碰撞/释放重入/计时分区，以及原协议、联合与本车QP、停车、场景检查，真实结果见tests.log。原求解器wheel版本和SHA已保存；恢复在/tmp独立目录，未改系统包。', '',
        '原始历史包未覆盖。当前工件包括可恢复计划/消息引用、必要诊断、实际轨迹及独立验收；目录大小见measurements.json与artifact_manifest.json。没有跑新的中央完整六轮，也不把单次新完整运行称多场景成功率。', '',
        '**下一阶段入口：**第一阶段结果可作新基线；求解顺序与逐条send/wait仍是原串行。第二阶段要实现四个工作进程/同轮信息版本/到齐屏障并证明时间重叠；第三阶段的论文核查已做，但连续转向受控实验尚未执行。它们分别建立新分支结果，不能在本报告标成完成。无需改变固定物理边界或追加远期任意初态规划。', '',
        '疑问逐项回应：[questions.md](questions.md)。原文方法与迁移边界：[literature_notes.md](literature_notes.md)。短交接与恢复入口：[HANDOFF.md](HANDOFF.md)。数据：[comparison.csv](comparison.csv)、[measurements.json](measurements.json)、full_L_FIFO/r1/independent_validation.json。', '',
        'token直接引用M2b_token_audit_001与token_hook_ab_001，不把数值周期当模型调用、不按墙钟换算用量；本轮未额外做当前任务token审计。']
    lines.append('\n## 交付补录与勘误（2026-11-03）\n\n1. 测试源码已补至test_reproduction/，含5个测试模块、夹具、配置、对应冻结运行源码、SHA清单和run_tests.py。隔离复制后重新执行恰好21项，结果见test_snapshot_validation.log；这是现在补录，原tests.log保留。第三方依赖仍需准备，说明见快照README.md。\n2. measurements.json中full_L_FIFO与B_no_profiler的cache_hits改为null，cache_hits_measured=false，表示未测。开启密集探针且覆盖所有提交周期的组保留实测整数及true标记；其中0才可解释为测得零命中。summarize_stage1.py已同步修复，无需重读完整实验日志。\n3. HANDOFF.md统一为21项，并补齐原测试命令中遗漏的test_m2b_scene以及包内独立复现入口。\n\n本次不重跑闭环、不修改控制器或原测试通过日志；重新生成本目录artifact_manifest.json。\n')
    (OUT/'report.md').write_text('\n'.join(lines)+'\n')
    print('report.md generated')
