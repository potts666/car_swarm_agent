# encounter_coordination_001：活动车交会协商

日期：2026-11-14。本包为实验指令，Windows本轮未运行控制实验。

Ubuntu Codex按序阅读：mainline_checkpoint.md、theory/README.md、theory/theory.md、encounter_experiment_instructions.md。

定位autonomy_init_002原工程，在新目录encounter_coordination_001推进：零调用找到合法双活动车事件，构造通过/让行与恢复后续，完成局部事务，再开展连续两轮本车MPC。新增调用上限8。

保留现有逐车服务期限规则，另列原事务块最大期限条件。原论文、失败与历史保持，Git提交留待用户指示。

解压核对：

```bash
mkdir -p /home/sunflower/Downloads/DMPC_encounter_001_2026-11-14
unzip -n /media/sf_DMPC_Theory/DMPC_encounter_001_2026-11-14.zip -d /home/sunflower/Downloads/DMPC_encounter_001_2026-11-14
cd /home/sunflower/Downloads/DMPC_encounter_001_2026-11-14
python3 verify_package.py
```

条件受阻时交付具体源、约束和候选诊断。代码与控制依赖复用Ubuntu既有002，本包仅附理论和指令。
