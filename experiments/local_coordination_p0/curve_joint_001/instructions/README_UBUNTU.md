# Ubuntu首读：curve_joint_001

日期：2026-11-14。任务：弯道实际a/omega开放、四臂对照、连续两轮真实重新优化。

## 读取顺序

1. mainline_checkpoint.md：研究目标与当前证据。
2. theory/curve_theory.md：CURVE-JOINT-v1，特别是输入自由度、接收域、目标与可执行步幅。
3. experiment_instructions.md：正式执行任务与至多16次尝试预算。
4. theory/source_notes.md：理论来源；theory/local_frame_theory.md为后续独立任务资料。
5. Ubuntu真实工程的encounter_coordination_001、autonomy_init_002报告、GATE、方法和共同源码。

## 文件校验

解压目录内运行：

```bash
python3 verify_package.py
```

本包提供理论和实验指令；控制工程由Ubuntu中已有目录定位。符号核对输出随包保留；重跑符号核对需要本机已有SymPy与mpmath环境。verify_package.py仅用Python标准库。

## 可复制给Ubuntu Codex

先按README_UBUNTU.md顺序完整阅读本包，再定位真实工程。创建新实验目录curve_joint_001，依experiment_instructions.md逐项实施。先以零调用认证当前地图弯道源、导数界、接收域、可表示步幅、四臂模型及worker传输，再释放数值求解。四臂为F-E/O-E/F-P/O-P，首次同一真实源，每臂两轮、每轮两车独立求解，正式上限16次worker尝试。保留历史和论文，Git提交另行安排。交付完整实际后续、独立验收、原始数值来源、分项GATE和可搬移最小证据包。实际条件不足时保存最紧约束与具体缺口，先完成受影响的理论/共同规则修正。
