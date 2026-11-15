# autonomy_init_002

本轮链路通过：自主初始化 → 四车有效推进 → 四份真实MPC受理 → 执行首拍
→ 真实后继重建 → 再四份真实MPC受理 → 1822拍全部完成。

新预算8/8，八次均Solved、全部受理，无重求解。第一轮四份数值前缀均
改变，首拍因eta=.02需要a0=1而保持；第二轮四份前缀和首拍均改变。
两轮首拍均四车有效推进，起始连续25拍四车有效推进。

report.md、GATE.json、results/numerical_results.csv为主要入口。
原始模型、JSON传输、装配指纹、native入场握手、raw(x,s,z)、接收/K、
实际轨迹及两层拒绝续行均保留。旧autonomy_init_001的部分通过和失败
数据保持原样；本目录是独立新版本、新预算，未使用DG002 C/L预算。

```bash
python3 verify_delivery.py
python3 verify_delivery.py --full-replay
```

默认核对SHA、归档源码与八份更新的独立精确验收；--full-replay额外
重放锚点、第一轮完整后续、最终实际轨迹及两层拒绝轨迹，均零新增求解。
验证可从任意cwd使用脚本绝对路径执行，Python标准库足够。
原数值环境为/tmp/dmpc-p0-deps，版本见requirements.txt。

本轮范围为固定四车地图、起始直线/停车调轮窗口、同步逻辑时钟。
没有验证通用弯道在线优化或异步网络性能。禁止重跑已释放native事件；
任何后续数值重复应另设目录、版本和预算。无Git提交。
