# rolling_interval_001 Ubuntu首读

读取mainline_checkpoint.md、latest_report_notes.md、两份theory与experiment_instructions.md，然后核对Ubuntu的slack_release_001及真实依赖。

```bash
python3 verify_package.py
```

给Ubuntu Codex：新建rolling_interval_001，先做整个解析区间的零调用逐拍预检。正式区间从首个内部提前释放、目标车准备服务开始，到目标车实际Clear/换权、后继运动后再32拍结束，最长512拍。每车每拍由同一入口计算自由度及模型，需优化时独立求解；唯一输入/已证最优/目标保持附证书。正式总上限2048次worker尝试，含异常。先验证首32拍，再自动推进剩余区间。每拍真实后继重建、完整受理、局部认可与备份；释放规则保持运行。保留历史和论文，Git提交另行安排，交付逐拍清单、分项GATE、完整实际/拒绝轨迹与独立证书。
