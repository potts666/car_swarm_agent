# Zotero文献依据与推导映射

日期：2026-11-14。本轮本机API状态检查未响应；只读检索Zotero数据库得到10个相关题名，核对库内PDF与既有提取全文。原库、附件及条目保持原样。

## 实际核对方式

- 核对5份既有PDF的当前SHA256与原来源记录，身份均保持。
- 对其中契约、联盟、协商和输入迁移4篇阅读相关定义、定理与证明；保留原全文及本轮索引。
- 新检索到Wu等2024年论文，从Zotero附件UZN2CSYP的全文索引读取相关方法、收紧及可行性条件，另与正式出版页面核对。
- Freire与Xu自行车SOCP附件身份也核对，作为原研究来源保留，本稿核心证明未直接援用其路径时间样条定理。

MiKTeX文本提取工具本轮受环境访问限制，使用与当前PDF身份一致的既有全文及Zotero索引。`zotero_attachment_audit.json`逐项标明文本来源，本轮未重新提取的文件明确记录。

## 1. 契约相容

Ghasemi, Sadraddini, Belta (2024), *Compositional synthesis for linear systems via convex optimization of assume-guarantee contracts*, Automatica 169, 111816。

DOI：[10.1016/j.automatica.2024.111816](https://doi.org/10.1016/j.automatica.2024.111816)。[正式出版页](https://www.sciencedirect.com/science/article/pii/S0005109824003108)。Zotero条目96QTKD6G，附件K73MBR7W。

核对§4定义2—4、§5.1定义5及式(18)—(19)。借鉴局部保证落入相容集合的组织。本稿以连续占用面和资源契约形成产品可行集合，并对自行车实际流单独证明。

## 2. 合并、拆分与初始化

Baldivieso-Monasterios, Trodden (2021), *Coalitional predictive control: Consensus-based coalition forming with robust regulation*, Automatica 125, 109380。

DOI：[10.1016/j.automatica.2020.109380](https://doi.org/10.1016/j.automatica.2020.109380)。[正式页](https://doi.org/10.1016/j.automatica.2020.109380)。条目JH59NITP，附件72X6YMXT。

核对§5.1、命题5.1—5.6和§5.2入口。文中分析分区改变的可行性条件。本稿固定每车物理输入归属，迁移规范化契约和候选，并为物理合同改变定义事务锚点。

## 3. 交互协商与回退

Sánchez-Amores, Chanfreut, Maestre, Camacho (2023), *Robust coalitional model predictive control with negotiation of mutual interactions*, Journal of Process Control 123, 64–75。

DOI：[10.1016/j.jprocont.2023.01.017](https://doi.org/10.1016/j.jprocont.2023.01.017)。[正式页](https://www.sciencedirect.com/science/article/pii/S0959152423000252)。条目J3SLGPRS，附件YJP3BQ6D。

核对§4、§5假设2—3及定理1。其递归可行使用集合包含条件。本稿允许几何合同变化，使用完整新锚点和两层回退保证新旧版本各自一致。

## 4. 输入表示迁移

Farooqi, Fagiano, Colaneri, Barlini (2020), *Shrinking horizon parametrized predictive control with application to energy-efficient train operation*, Automatica 112, 108635。

DOI：[10.1016/j.automatica.2019.108635](https://doi.org/10.1016/j.automatica.2019.108635)。[正式页](https://www.sciencedirect.com/science/article/pii/S0005109819304960)。条目YCIUCCDF，附件CPMSRBJ5。

核对§3.1定义1、假设3及名义候选构造。借鉴表示可迁移要求；本稿由固定拍长ZOH、实际保存尾段及非零延续输入给出明确移位。

## 5. 次优分布式求解

Wu, Dai, Xia (2024), *Iterative distributed model predictive control for heterogeneous systems with non-convex coupled constraints*, Automatica 166, 111700。

DOI：[10.1016/j.automatica.2024.111700](https://doi.org/10.1016/j.automatica.2024.111700)。[正式页](https://www.sciencedirect.com/science/article/pii/S0005109824001948)。条目ZPL4CMRQ，附件UZN2CSYP。

核对§3内外层、假设3—4、可行性收紧与§4算法/分析。参考有限迭代输出需要可行性条件的思路。本稿第一版选择契约内局部SOCP与完整受理，论文的线性系统稳定性结论按其原前提保留。

## 本稿与文献的关系

文献提供契约组合、组变化初始化、交互协商、输入迁移与次优受理的依据。本稿把这些接口组织成自行车ZOH、矩形连续几何、每车输入所有权、局部事务与最大完成期限的统一构造。

本轮未做完整创新性普查。研究差异的实质价值将通过地图上的兼容域、多个局部新计划的组合、换组证书与公平对照进一步检验。
