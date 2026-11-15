# Zotero与正式论文依据

日期：2026-11-14。本轮先按插件接口检查Zotero；初次本机网络权限失败，经允许的只读重试，API及connector均HTTP 200。通过本地API检索并取得7篇附件的索引全文，未改写库。

## 1. 证明结构直接相关的来源

| 论文 | DOI与原文位置 | 在本稿中的作用 |
|---|---|---|
| Chen, Guo, Li, Deadlock Resolution and Recursive Feasibility in MPC-Based Multirobot Trajectory Generation, IEEE TAC 69(9), 6058–6073, 2024 | [10.1109/TAC.2024.3393126](https://doi.org/10.1109/TAC.2024.3393126)，定理1—4；Zotero PPB5U35Y/6E8HS4UU | 借鉴候选移位与由有界运动推导局部通信范围的证明思路。本稿按自行车矩形及完整后续单独推导 |
| Baldivieso-Monasterios & Trodden, Coalitional predictive control: Consensus-based coalition forming with robust regulation, Automatica 125, 109380, 2021 | [10.1016/j.automatica.2020.109380](https://doi.org/10.1016/j.automatica.2020.109380)，定义3—4、定理4.1—4.3及§5.1；JH59NITP/72X6YMXT | 有限改进与换组初始化；本稿用冻结有限池名次与候选迁移给出自己的归纳 |
| Sánchez-Amores等, Robust coalitional model predictive control with negotiation of mutual interactions, J Process Control 123, 64–75, 2023 | [10.1016/j.jprocont.2023.01.017](https://doi.org/10.1016/j.jprocont.2023.01.017)，§5.1假设2—3、§5.2定理1；J3SLGPRS/YJP3BQ6D | 关注集合更新的可行包含与失败回退；本稿采用相容几何合同和真实后续 |
| Ghasemi, Sadraddini, Belta, Compositional synthesis for linear systems via convex optimization of assume-guarantee contracts, Automatica 169, 111816, 2024 | [10.1016/j.automatica.2024.111816](https://doi.org/10.1016/j.automatica.2024.111816)，定理1、定义5—6；96QTKD6G/K73MBR7W | 局部证书与组合正确性。本稿的车身外扩索引及物理合同使用自行车实际流 |
| Ma, Hönig, Kumar, Ayanian, Koenig, Lifelong Path Planning with Kinematic Constraints for Multi-Agent Pickup and Delivery, AAAI 33(1), 7651–7658, 2019 | [10.1609/aaai.v33i01.33017651](https://doi.org/10.1609/aaai.v33i01.33017651)，p.7652良构端点、TP及SIPPwRT部分；本轮网上查阅 | 端点避让与预约规划启发INIT-BUF。原文使用圆盘、点转向及给定速度等模型；本稿将连接替换为有输入界、加速/转向可达和矩形扫掠证书，并另证2M转移上界 |

## 2. 补充查阅

- Salimi Lafmejani & Berman, Nonlinear MPC for collision-free and deadlock-free navigation of multiple nonholonomic mobile robots, Robotics and Autonomous Systems 141, 103774 (2021)，DOI [10.1016/j.robot.2021.103774](https://doi.org/10.1016/j.robot.2021.103774)。Zotero 3W4RLX7Y/7YZ3W7NR。查阅模型与理论分析结构，作为非完整约束MPC的比较背景；本稿采用自己的五状态与终端构造。
- Multi-Robot Trajectory Planning With Feasibility Guarantee and Deadlock Resolution: An Obstacle-Dense Environment，IEEE LRA (2023)，DOI [10.1109/LRA.2023.3248377](https://doi.org/10.1109/LRA.2023.3248377)。Zotero FGZUQNCF/MX5F5PJR。查阅安全走廊、可行候选与障碍场景结构。
- Wu, Dai, Xia, Iterative distributed model predictive control for heterogeneous systems with non-convex coupled constraints, Automatica 166, 111700 (2024)，DOI [10.1016/j.automatica.2024.111700](https://doi.org/10.1016/j.automatica.2024.111700)。Zotero ZPL4CMRQ/UZN2CSYP。关注有限迭代下的可行输出；本稿用证书和保存锚点处理次优输出。

## 3. 本轮证明与已有论文的关系

本稿的DAG初始化、buffer初始化、全保存后续索引覆盖、接收半径链、仿射量化格及最终归纳均在《完整证明.md》中给出推导。文献用于提出构造思路和核对所需前提。

Chen论文的死锁稳定性条件和模型与本场景各异；本稿以可执行期限下降证明有限任务完成。联盟论文中的稳定性结论按原模型前提保留。当前创新表述聚焦这些接口在自行车、矩形和动态组任务上的统一构造，尚未开展穷尽性创新检索。

来源元数据与附件全文SHA在sources/zotero_records.json。SHA对应本次API索引全文，PDF字节身份另有需要时再核对。索引全文覆盖页数记录在元数据中，本轮按相关章节阅读。

正式在线来源：[AAAI出版页](https://ojs.aaai.org/index.php/AAAI/article/view/4759)、[AAAI正式PDF](https://cdn.aaai.org/ojs/4759/4759-13-7798-1-10-20190707.pdf)、[契约论文作者存档](https://calinbelta.com/wp-content/uploads/2024/09/compositional-synthesis-for-linear-systems.pdf)、[联盟论文机构存档](https://eprints.whiterose.ac.uk/id/eprint/168005/)。
