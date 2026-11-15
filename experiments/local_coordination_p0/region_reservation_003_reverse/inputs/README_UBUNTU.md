# 区域预约第一组构造与认证

日期：2026-11-13。

先读`theory.md`，再运行两个只用Python标准库的检查程序：

```bash
python3 construct_certificate.py
python3 check_domains.py
```

预期：32段、64个区间叶子通过；计入每车0.05m误差管后的道路下界约0.109973m、车对净空下界约0.528223m。域与并行时序检查通过。它们是连续区间和解析代数检查，未调用Clarabel。

## 交付内容

- `theory.md`：等待域、五状态进入域、驶出域、目标接续、预约与三个闭环证明。
- `geometry_certificate.json`：完整参数区间的几何证书。
- `domain_certificate.json`：域衔接、输入界、拍数及并行连接的有理检查。
- `construct_certificate.py`、`check_domains.py`：可复现检查。
- `interval_source.py`：从原工程复制的整数向外舍入基础；作为模块导入。
- `counterexample_old_wait_parking.json`：早期原等待布局下停车连接未通过的记录。
- `overview.html`：图解。
- `next_implementation.md`：工程有理输入与预约接入的下一任务。

## 当前性质

已构造实数解析ZOH输入和适用域，并认证连续几何与域包含。工程有理输入导出、四车协议接入和控制重放尚待进行。旧C0_fix_002可复用来源绑定和拒绝续行，原结果保留。

本构造从原四车静止初态开始，增加55拍侧向等待准备。车辆优先规则保持在线可选，工作顺序“先来后到”与车辆调度规则分别记录。
