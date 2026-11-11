# 短交接：M2b理论指标分析已完成

边界：只读原M2b_fourcar_001/M2b_stage1_001与冻结源码；不改QP、场景、控制器、原审计，不重跑闭环。

交付：report.md、metric_dictionary.csv、run_inventory.csv、comparison.csv、candidate_coverage.csv、cycle_metrics/与event_metrics/索引及gzip正文、3张figures、evidence_gaps.md、文献条件与可复用代码。execution.json记录入口/版本，artifact_manifest.json记录本包SHA。

关键结果：1个物理配置，17类记录，8个含前缀/人工空轨迹的运动哈希组不能当独立场景。局部882组seed采样物理/协议通过，含首轮初始化；部分阶段30个车周期失效中1初始化、29真实移位切换。部分参考终端24个失效，22本车切换、2停车参考激活。局部重优化共同窗最大位置变化8.772422407m，在6.2s远端；同模式最大约0.380033m。v1停车缺口1.71923755e-7m；v2不由该必要条件解释。

解释：阶段/参考切换的候选衔接为优先理论缺口，邻车预测依赖条件次之。完整OCP、终端不变性、Lyapunov和鲁棒域未建立，不能标pass。三个自然拒绝出口仍有车在运动；离线退出不是安全停车。

身份：6份可取得源快照核对，8个复用分支另核消息ACK，六轮正式共享诊断恢复原SHA，748个正文对象。维护控制源与stage1冻结版本逐文件相同。7项分析器测试通过，无后台运行待等待。

恢复：只需读取report.md与completion.json；查看具体反例用seed_failure_events.csv/event_metrics，不重读整个原包。复算顺序/依赖见README.md和execution.json；analyze.py拒绝覆盖现有结果，要复算请新编号同级目录只复制分析.py。缺口：Dunbar全文未取得，Sun仅索引片段，debug_v2完整源版本与两例诊断正文缺失，均已明示。
