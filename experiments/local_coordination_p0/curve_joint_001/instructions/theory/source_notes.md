# 本轮来源与新增构造

日期：2026-11-14。本轮参考前一轮取得的Zotero索引全文，并核对正式出版来源。文献与本稿导数/合同构造分别列明。

1. Wu, J., Dai, L., Xia, Y. (2024). Iterative distributed model predictive control for heterogeneous systems with non-convex coupled constraints. Automatica166,111700。DOI [10.1016/j.automatica.2024.111700](https://doi.org/10.1016/j.automatica.2024.111700)。参考§3假设3、Lemma1的梯度Lipschitz凸上界；Zotero ZPL4CMRQ/UZN2CSYP。原模型为线性子系统，本稿为真实自行车流单独推导C4—C21。

2. Ghasemi, K., Sadraddini, S., Belta, C. (2024). Compositional synthesis for linear systems via convex optimization of assume-guarantee contracts. Automatica169,111816。DOI [10.1016/j.automatica.2024.111816](https://doi.org/10.1016/j.automatica.2024.111816)。参考定义5与局部保证组合；Zotero96QTKD6G/K73MBR7W。本稿固定外框架、内部版本与时窗的定理按矩形车身及实际资源构造。

原全文身份见`自主DMPC_闭环理论_2026-11-14/source_audit.json`及sources。全文留本地，分发包只附来源索引。

新增理论内容：利用v/delta精确仿射结构给出无矩阵指数的ZOH余项；三块矩保持输入构造与弯道正推进系数；原代价零输入反例和新推进目标的严格改善条件；外框架不变时的自然小组独立更新。

数学推导与SymPy核对均在本轮文件中给出。当前图上源与外部包络尚待实例认证，原控制目标和协议保持原样。
