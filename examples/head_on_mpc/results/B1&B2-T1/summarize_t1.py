"""Render the T1 report and flat result tables from retained measurements."""
import csv
import json
from pathlib import Path
import statistics

ROOT=Path(__file__).resolve().parent
D=json.loads((ROOT/'derivative_results.json').read_text())
P=json.loads((ROOT/'parking_results.json').read_text())

def csv_write(name,rows):
    with (ROOT/name).open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)

def md_table(headers,rows):
    return '\n'.join(['|'+'|'.join(headers)+'|','|'+'|'.join(['---']*len(headers))+'|',*['|'+'|'.join(map(str,row))+'|' for row in rows]])

flat=[]
for instance in D:
    for run in instance['runs']:
        r=run['record']
        flat.append(dict(instance=instance['instance'],derivative=run['derivative'],update_limit=run['update_limit'],accepted=r['accepted'],
            updates=r['qp_updates'],iterations=sum(s['iterations'] for s in r['subproblems']),
            position_m=max(e['position_m'] for e in r['mapping_errors'].values()),
            heading_rad=max(e['heading_rad'] for e in r['mapping_errors'].values()),
            speed_m_s=max(e['speed_m_s'] for e in r['mapping_errors'].values()),
            steering_rad=max(e['steering_rad'] for e in r['mapping_errors'].values()),
            affine_violation=r['affine_violation'],offline_bicycle_ms=run['offline_bicycle_wall_s']*1000,
            reference_build_solve_check_ms=run['reference_build_solve_check_s']*1000,
            qp_solver_ms=sum(s['solver_s'] for s in run['stages'])*1000,
            budget_remaining_s=run['budget_remaining_s'],committed=run['committed']))
csv_write('derivative_summary.csv',flat)
flat_parking=[]
for run in P:
    downstream=run['replay'].get('downstream_bicycle_original_3_updates',{}).get('record',{})
    flat_parking.append({k:run[k] for k in ('cycle','repeat','mode','status','iterations','accepted','physical_constraint_violation',
        'terminal_equation_error','stationarity_original_free','complementarity_original','build_s','setup_s','solver_s','total_s')} |
        dict(path_geometry_diagnostic_passed=run['replay']['path_geometry_diagnostic_passed'],
            downstream_bicycle_accepted=downstream.get('accepted','not rerun'),committed=False))
csv_write('parking_summary.csv',flat_parking)
first=D[0]
derivative_rows=[]
for r in [r for r in flat if r['instance']=='B2_single_56']:
    derivative_rows.append([r['derivative'],r['update_limit'],r['accepted'],r['updates'],f"{r['position_m']*1e6:.6f}",
        f"{r['offline_bicycle_ms']:.2f}",f"{r['reference_build_solve_check_ms']:.2f}",f"{r['qp_solver_ms']:.2f}"])
parking_rows=[]
for mode in ('cold','shift_primal','shift_primal_dual','no_row_scaling'):
    runs=[r for r in P if r['cycle']==121 and r['mode']==mode];r=runs[0]
    t=[s['total_s']*1000 for s in runs]
    downstream=r['replay'].get('downstream_bicycle_original_3_updates',{}).get('record',{})
    parking_rows.append([mode,r['status'],r['iterations'],r['accepted'],f"{r['physical_constraint_violation']:.3e}",
        f"{r['terminal_equation_error']:.3e}",f'{statistics.mean(t):.2f} / {min(t):.2f}–{max(t):.2f}',downstream.get('accepted','未调用')])
neighbor_rows=[]
for cycle in range(117,122):
    values=[]
    for mode in ('cold','shift_primal','shift_primal_dual','no_row_scaling'):
        run=next((r for r in P if r['cycle']==cycle and r['mode']==mode and r['repeat']==0),None)
        values.append(f"{run['iterations']} / {run['status']}" if run else '无前周期种子')
    neighbor_rows.append([cycle,*values])
fd=first['finite_difference'];probe=first['same_optimized_candidate'];amplitude=first['composed_candidate_amplitude_probe']
report=f'''# B1&B2-T1：冻结失败实例的机制对照

已完成 T1 实验与复现检查；没有把未消除的拒绝改报成功。导数不一致确实存在，但不是 cycle56 超门的主导来源；移位热启动能消除 cycle121 这个冻结参考 QP 的收敛拒绝，并继续通过该状态的原 B2 非线性候选验收。本轮不构成完整闭环、实时 DMPC 或任务加速证明。

## 1. 冻结与复现

以 `../bicycle_e3_002/source` 为准，三个计算核心文件逐一匹配原 provenance 的 SHA256，复制在 `source/`。原目录未修改。

冻结 B2_single cycle56、B2_sequence cycle55–61 与117–121，共13个状态。每车状态取同一时刻 actual.csv，并与上一实际提交周期的首个预测端点交叉核对（绝对容差2e-12）；阶段、段索引、许可、路径进度和剩余停车步取 constraints；前次加速度取已执行命令；跨段时按原逻辑清零。重建后 `build_qp()` 的全部参数必须与 solves 中的参数完全相等。每个快照保留原求解记录。

cycle56 原候选的128维解与原日志逐项一致（核对容差1e-10）；停车117–121冷启动迭代数均与原日志一致，包括121的20000次拒绝。`frozen/*_reference.npz` 保留原H、q、不等式、等式与输入界；`*_target.npz` 保留固定参考和输入；每个自行车数值QP保存实际送入OSQP的矩阵、向量和设置；停车求解另存原始/对偶输出、自由变量索引、行标识和缩放。

## 2. 导数映射对照

固定原场景、目标、N=32、h=0.2 s、sample_s=0.02 s、走廊、分离轴、信赖域、物理界和验收门限。唯一模型变量是：单个0.2 s RK4导数，或十个0.02 s RK4的链式复合导数。每个实例参考QP只解一次，四组候选复用同一参考；每组更新上限分别为1或3，第二次求解仍按原程序使用target_inputs作数值热启动。

独立中心有限差分调用十次原 `step()`，不用解析导数计算映射。epsilon=1e-5；cycle56全参考节点最大绝对导数误差：原导数 {fd[0]['max_abs_fd_error']:.6e}，复合导数 {fd[1]['max_abs_fd_error']:.6e}。8个实例复合导数均通过2e-8自检。有限差分和固定扰动是数学探针，允许跨出执行输入界，不作为执行候选。

先比较相同参考加相同扰动，再比较相同的原优化候选，最后才比较各自优化的候选。对同一cycle56原候选，原/复合仿射模型的最大位置差仅 {first['derivative_effect_position_m']['B']:.6e} m，而其与非线性映射的位置差分别为 {probe[0]['errors']['B']['position_m']:.6e} / {probe[1]['errors']['B']['position_m']:.6e} m。复合导数下将该候选相对参考的输入偏移缩为1/4、1/2、1，位置模型差依次为 {', '.join(f"{a['errors']['B']['position_m']:.6e}" for a in amplitude)} m，近似按平方增长。这支持该冻结实例的一阶线性化高阶余项主导拒绝；不是对全部状态或数值误差来源的普遍证明。

cycle56候选结果（位置模型差单位μm）：

{md_table(['导数','更新上限','接受','实际更新','位置模型差μm','离线自行车总ms','参考构造求解校验ms','自行车QP求解ms'],derivative_rows)}

原/复合导数均在cycle55一次通过，cycle56–61一次均拒绝、第二次均通过；复合导数没有减少这些实例的更新数。报告保留导数替换未消除拒绝的反例，不能声称其带来数量级的候选误差下降或计算加速。

计时：离线自行车总时间含构造、QP、重演和候选检查，参考调用替换为冻结结果；参考时间单列，不能把任一列称为完整运行周期。QP列含原实现记录的setup+solve。矩阵文件在计时结束后写盘。邻近实例的逐状态误差与计时见 derivative_summary.csv，逐次检查见 derivative_results.json。每组仅一次求候选，未估计其计时波动。

## 3. 同一停车QP的等价对照

保持停车终点、终端速度、非负路径速度、到达步和原数值带不变。依次独立比较冷启动、上周期解按车移位一格的原始变量种子、原始/对偶种子、去掉显式行归一化（保留目标缩放与OSQP内部缩放）。每组重复3次，只估计本机计时波动，不代表多场景成功率。种子来自同一原历史状态下前一周期冷启动的解；未让各方案重新生成不同的历史轨迹。

对偶映射按车/行含义/时间节点建立，节点和输入行移位，终端行保留其逻辑含义，缺失行置零；先还原原目标/约束行单位，再转换到当前缩放。自由变量和约束行每周期重新建立映射，不复用旧稀疏结构。它是热启动启发式，不假设上周期乘子仍为当前最优乘子，也不声称乘子唯一。

cycle121结果：

{md_table(['方案','状态','迭代','参考QP接受','原单位约束最大违例*','原终端方程最大误差*','参考总ms 均值/范围','后续原B2接受'],parking_rows)}

* 此两列为不同物理量的最大值，不能解释为单一单位的残差。位置(m)、速度(m/s)、加速度(m/s²)按行含义的独立误差保存在 violation_by_row_kind；端点位置与速度方程另按原表达检查。所有接受项遵循原1e-5物理验收门限、原可行性带、原求解状态规则，并检查有限值及H半正定性。KKT原单位自由变量驻点残差和互补性见原始JSON，用作诊断，未以它们替代原非线性检查。

邻近状态对照：

{md_table(['cycle','cold 迭代/状态','shift_primal','shift_primal_dual','no_row_scaling'],neighbor_rows)}

冷启动与原日志一致，热启动作用不单调；原始/对偶种子在118、120反而迭代更多；取消显式行缩放在119、120引入拒绝，因此不能当成通用修复。

对121三种参考QP接受方案，分别接入**原导数、原最多3次更新**的B2层，均一次更新通过其模型误差、信赖域与非线性安全检查（详细record保留）。未把导数替换与热启动混在同一收益对照中。参考层固定几何路径与该冻结实际姿态之间另有约3.008e-5 m偏差，保留为 path_geometry_diagnostic_passed=false；原B2以参考非线性重演为target，并不把这个参考路径诊断作为门限。最终通过与否仍由原B2的候选重演逻辑判定。冷启动未成功时不继续自行车层，也不把占位误差当作0。

这说明同一冻结问题可用等价初始化通过原验收，支持求解数值过程是该次拒绝的可改善因素；不证明全局数值退化、对偶不唯一或跨周期始终可行。未改变停车终端表述；未提高max_iter；未引入新在线求解器。

## 4. 数据与复现

- `run_t1.py`：冻结、原日志复现断言、受控实验、计时与重演。
- `source/`：原计算核心；复合导数仅在实验runner内切换。
- `frozen/`：状态、原记录、矩阵、参考、求解器原始/对偶结果。
- `solver_settings.json`、`provenance.json`：原参数、环境、命令、输入/源码/快照哈希。
- `derivative_results.json`、`parking_results.json`：完整求解与非线性验收记录。
- `derivative_summary.csv`、`parking_summary.csv`：便于比较的平表。
- `verify_t1.py`、`verification.json`：可重复检查的断言与结果。

依赖与原实验一致：Python3.12、numpy1.26.4、scipy1.11.4、OSQP1.1.3。当前机器的OSQP/joblib只解压到/tmp/t1_qp_deps；下载地址、固定版本与SHA256同原dependency_download_manifest.json，已核对。未修改系统或项目依赖。需要在其他机器自行准备这些依赖。

从仓库根目录运行：

```bash
OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/t1_qp_deps python3 'examples/head_on_mpc/results/B1&B2-T1/run_t1.py'
python3 'examples/head_on_mpc/results/B1&B2-T1/verify_t1.py'
python3 'examples/head_on_mpc/results/B1&B2-T1/summarize_t1.py'
```

运行会更新本T1目录的派生产物；历史bicycle_e3_002保持只读。所有候选均为离线、committed=false。计时没有包含实际车辆持续运动、通信延迟或完整未来释放检查；没有全程重跑B1/B2，也没有修改生产控制器。本轮可交付的是13个冻结状态上的机制证据及可复现实验，不是部署修复。
'''
(ROOT/'report.md').write_text(report)
print('report.md and two CSV tables generated')
