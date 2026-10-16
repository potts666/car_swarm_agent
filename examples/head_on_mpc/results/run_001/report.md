# 方向一：单通道双车相向通行最小案例

本次以同一地图、初值比较“主道停车”“旁侧暂停区”“倒车退让”。每个子目录都是按七项标准保存的一次模拟日志总结。方向二的插入案例不包含在本次交付中。

版本：上游`2d5f84f225899ae4b749cb2c851bfdbed86a9642` + 提取源码SHA-256 `685582862f5834ab7ada58cd53a80113cfe05fb227dcb135ad8ecffb810f8183`。最终Git提交由用户完成。

| 模拟报告 | 结果 | 仿真时长s | 最小采样SAT间隙m | B倒车m | 视频 |
|---|---|---:|---:|---:|---|
| [head_on_inline_stop](head_on_inline_stop/report.md) | blocked | 24.8 | 1.300 | 0.00 | [MP4](head_on_inline_stop/replay.mp4) |
| [head_on_pause_bay](head_on_pause_bay/report.md) | completed | 37.6 | 1.000 | 0.00 | [MP4](head_on_pause_bay/replay.mp4) |
| [head_on_reverse](head_on_reverse/report.md) | completed | 50.0 | 1.000 | 4.58 | [MP4](head_on_reverse/replay.mp4) |

本组固定初值下，旁侧暂停区策略比退回入口更快。只在主道停车的对照保持间距但没有完成通行。这些结果只验证纵向空间和时序，不能据此声称已自主选择策略或验证真实转向可达性。

标准项目：提交版本、地图与初始条件、控制周期、通信设置、原始日志与图片、RViz2回放、可播放MP4；三份模拟报告均逐项列明。

真实ROS发布/订阅验证通过，三种策略最终车辆位置与原始日志一致，收到相机TF。 详见 [ROS验证记录](rviz_validation.json)（完成验证后提供）、[离线验证](validation.json)、[来源与依赖](provenance.json)。

代码只有本方向的纵向优化、状态机、共享日志读取、报告/视频导出、RViz2回放与相关测试。没有旧地图规划器、C++原生二进制、其他方向或旧演示。
