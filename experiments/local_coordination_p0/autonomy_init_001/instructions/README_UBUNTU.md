# Ubuntu Codex入口：autonomy_init_001

日期：2026-11-14。此包为实验指令与理论材料，Windows端本轮未运行控制实验。

先读mainline_checkpoint.md、theory/README.md、theory/theory.md，再完整阅读experiment_instructions.md。定位Ubuntu现有dynamic_groups_001/002及region003工程，在新目录autonomy_init_001执行。

任务：从地图、车型、各车真实初态和目标构造第一份合法完成策略，先认证Omega_DAG/Omega_BUF条件，再接入局部占用发现、每车MPC、多活动合同、真实尾段和期限归纳。

先完成零调用认证及初始化完整重放。条件通过后进行一个连续两轮多车数值窗口，最多8次新优化器调用。保留历史与论文，Git提交留待用户指示。

解压及核对：

```bash
mkdir -p /home/sunflower/Downloads/DMPC_autonomy_init_001_2026-11-14
unzip -n /media/sf_DMPC_Theory/DMPC_autonomy_init_001_2026-11-14.zip -d /home/sunflower/Downloads/DMPC_autonomy_init_001_2026-11-14
cd /home/sunflower/Downloads/DMPC_autonomy_init_001_2026-11-14
python3 verify_package.py
```

若源码或必要对象缺失，先记录实际查找路径与缺项，再请求对应资料。该包附理论和指令，原控制工程继续使用Ubuntu已有版本。
