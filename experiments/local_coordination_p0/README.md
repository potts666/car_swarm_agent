# 局部协调 DMPC：材料、实现与开发证据

源码基准 `ebc3d29`（`feature/deadlock-experiment`）。新增工作全部在本目录，历史控制入口、结果、本地大归档和论文保留。按用户指定 ponytail skill，复用原区间原语、路线几何和协议屏障。

先看 [核对报告](theory_review.md)、[33条定理前提矩阵](theorem_premises.md)、[参数登记](pending_parameters.json)。开发观察最终由 `development_report.md` 汇总。早期“材料缺失/P0未通过”报告保存在 `results/document_history_033/`，不能覆盖新结论。

## 材料身份与阅读顺序

1. 用户设计完整副本在 `design_source.txt`。
2. 先读指定 `DMPC_theory_utf8/README_UBUNTU.md`，按索引完整阅读统一三定理、运动交接严谨推导及修订说明。原文副本和SHA在 `theory/manifest.json`。
3. 先读 `DMPC_P0_sources/README_P0.md`，验证原对象清单，复制重建在 `results/source_rebuild_005/work/`。没有修改Downloads原件。

外部参考论文的PDF未在本轮重新审读；指定作者理论稿与外部文献须区分。

## 基准差距、复用与新实现

|基准事实/缺口|可复用模块|本次实现|
|---|---|---|
|五状态自行车、h=.2/N32已有，RK4采样不等于认证|`bicycle_core`、四车配置|原认证区间积分；连续道路、六对净空及权限外包；独立几何验收|
|旧阶段靠停稳推进，移动域/K未绑定|`m2b_scene`路线概念、源包`certificate_graph`几何|`cross_stop.py`同几何S/M连接；`moving_objects.py`移动域；`pilot_control.py`实际移位K|
|旧QP及热启动没有完整SOCP结论|原QP保留作历史实现|`socp_anchor.py`真实Clarabel锥、显式零锚点、有理接受及拒绝续行|
|已有权限事务，缺新域/期限/策略版本|`m2b_protocol.Barrier`、内容身份|全状态、接收phase、策略U/预算绑定；三车组合并拆分及换权|
|FIFO/ALT不是m与m+U消融|旧日志、计时与验收结构|`run_pilot.py`五组、共同初始S策略、同版U守卫；`accept_execution.py`重演|
|归档图/预算只是文档数字|源包20模板、图、proof/math检查|`rebuild_sources.py`原件SHA及C/R复算；开发图和预算另行生成|

旧模块位于 `examples/head_on_mpc/scripts/`，公开入口和历史结果未修改。新入口限于受限开发认证库，不能当作旧生产入口已经升级。

## 核对及运行入口

- `results/source_rebuild_005/`：原20模板、图、proof/math、预算重建。
- `results/moving_objects_006/`：6456窗口、5861非零速度切点、3187同时非零轮角切点及依赖转交。
- `results/final_units_020/`：10项小核对、3项解析曲线、4项旧协议回归。
- `results/final_conic_021/`：正盒真实锥求解、边界锚点、非法锚点拒绝。
- `results/p0_moving_v2_028/`、`p0_stopped_v2_029/`：闭环、组变化、拒绝续行、到期接收和新持权车首拍，均有独立验收。
- `results/acceptance_damage_v2_031/`：七类损坏执行证据均拒绝。
- `P0_GATE_v2.json`：当前开发门禁，绑定源码、协议依赖、模型、案例和证据SHA，只允许开发运行。
- `run_development.py`：三案例×五组，每次后独立验收；阶段上限900秒，失败即停、完整日志、拒绝覆盖输出。

```bash
PYTHONPATH=/tmp/dmpc-p0-deps OPENBLAS_NUM_THREADS=1 \
python3 experiments/local_coordination_p0/run_development.py \
  --rebuild experiments/local_coordination_p0/results/source_rebuild_005 \
  --objects experiments/local_coordination_p0/results/pilot_objects_011 \
  --gate experiments/local_coordination_p0/P0_GATE_v2.json \
  --output /tmp/dmpc-development-repeat
```

这是显式重跑命令，当前不必重复。`requirements-development.txt`记录直接依赖版本，`dependency_provenance/`保存本轮下载wheel身份；`/tmp/dmpc-p0-deps`清理后须恢复依赖。

## 适用边界

全程源域覆盖位置/航向区间，粗资源节点v/delta精确为零，内部移动域为精确流像；没有声称任意五状态开域都受理。SOCP固定m、使用保守导数界，不搜索更短时间，也无优化最优性证书。

所有组使用同一单活动车规则和理想同步进程内消息。非零交接发生在路线内部，车间权限转交仍在停车资源节点。仅资源接口停车的强参考列在 `resource_only_stop_reference.json`；其闭环敏感性、兼容运动并行、计算并行和四车同组消融尚未运行。

没有正式留出批量、没有论文修改；三个开发案例不能支持普遍可靠性、实时或规模收益结论。
