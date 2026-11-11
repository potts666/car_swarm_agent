# 只读理论指标分析复算

输入是相邻的`M2b_fourcar_001`、`M2b_stage1_001`及既有`M2b_token_audit_001`。这是分析输出，不是原始实验的独立重打包：复算需要原计划文件与其共享正文、诊断对象目录和冻结源码；哈希不能替代正文。本目录没有复制全工程或重复原始计划。

Python及依赖版本、工作目录、实际命令、冻结物理源码SHA见execution.json/parameters.json。OSQP仅因原源码的传递导入而需要，不调用任何求解器。当前依赖在`/tmp/m2b_stage1_repro_deps`，该临时路径不是持久工件；可按stage1保存的原wheel版本准备其他依赖目录。所有脚本都在本结果目录，`OUT`相对脚本位置确定。

从仓库根目录设置：

```sh
export OPENBLAS_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH=/path/to/numerical_deps
```

按execution.json顺序执行对应脚本。test_analysis.py为7项针对性分析器检查。analyze.py拒绝覆盖已有status.json；新一轮复算应新建同级结果目录，只复制分析`.py`文件，再运行，不删除原分析结果。其他脚本仅读取既有数据/指标并生成汇总；verify_shared_diagnostics.py只恢复诊断逻辑字节并核SHA，不重建完整OCP可行候选。

大日志逐周期读取，移位计划只保留各车上一final恢复前缀；缺正文/哈希不符会明确失败。数值重复组用cycle_metrics/index.csv与event_metrics/index.csv映射到代表正文，各分支消息/ACK仍分别核对。图表只显示代表完整运动组，其他前缀/失败保留在表中，未补造完整曲线。

分母：初始4个seed不是移位候选；车周期与四车组合周期不同。candidate_coverage.csv有分层覆盖/通过比例；它们不是独立场景成功率。完整OCP未检查的通过比例为空，不是0%或100%。F/G未知项不标pass。

本次没有新闭环、场景/扰动扫描、控制方法或权限修改。原控制文件最终与stage1冻结清单逐文件相同。文献访问缺口和命题适用边界见literature_conditions.md/evidence_gaps.md。
