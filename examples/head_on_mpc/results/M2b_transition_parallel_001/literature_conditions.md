# 文献依据与不适用条件

- Zheng 等，IEEE TCST 2017，[正式 DOI](https://doi.org/10.1109/TCST.2016.2594588)，读取[作者 accepted final](https://zhengy09.github.io/papers/2016_J_DMPC_TCST.pdf)。Remark 8 明列初始若干步局部可行假设；Lemma 5 的移位候选依赖其终端条件。单向纵向编队及一致性权重条件不能自动推广到本项目的停车切换与矩形自行车。
- Katriniok 等，IEEE T-ITS，[正式 DOI](https://doi.org/10.1109/TITS.2022.3162038)，读取 [arXiv v2 accepted final](https://arxiv.org/html/2104.07383v2)。§IV-A 用唯一优先级仅对低优先级端装配冲突约束，并提醒同时独立决策可能冲突；§V-C 明确同步执行及交换预测。本文未采用其优先级解耦与软约束，必须另行对共同最终组合验收。本文沿用同轮信息与交换后执行的组织原则；其固定路径纵向加速度模型与本文五状态控制不同，不移植其安全结论。
- Chen 等，IEEE TAC 2024，[正式 DOI](https://doi.org/10.1109/TAC.2024.3393126)，读取 [arXiv v4](https://arxiv.org/html/2202.06071v4)。Theorem 3 的移位递归可行性依赖初始可行、特定几何分割和末端平衡延拓；§II-A 的模型及终端条件不同。本实现没有完成相应自行车命题与证明。

本轮工程措施为版本封存、有限一轮兼容检查及失败拒绝。未新加一致性代价、观测最大值约束或论文未要求的调参。8.77 m、0.38 m 和 STATE_TRUST 不被当作理论跨周期界。缓存与 DDS 批收发只归为实现改进。
