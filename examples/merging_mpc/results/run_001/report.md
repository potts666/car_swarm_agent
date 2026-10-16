# 方向二：单C车插入A、B之间的merging最小案例

同一初值运行“不配合开间距”和“配合开间距”两个配置。目标是让邻道C进入主道A-B之间，最终形成A-C-B。每个子目录保存一次完整的七项标准模拟日志总结。

版本：运行时父提交`75b27b621781a7e00800b00e6f8c7a1952d622a5` + 执行源码SHA-256 `9e7c1c637e0a0be504fcc622bf83958cabd8876deee4bad5f3525bdaf01b6256`。新增目录尚待用户最终Git提交。

| 模拟报告 | 状态 | 获准时刻s | 完成并线s | 观察时长s | B最低速度m/s | 视频 |
|---|---|---:|---:|---:|---:|---|
| [merge_no_cooperation](merge_no_cooperation/report.md) | blocked | None | None | 30.0 | 1.500 | [MP4](merge_no_cooperation/replay.mp4) |
| [merge_cooperative](merge_cooperative/report.md) | completed | 2.8 | 8.8 | 25.2 | 0.684 | [MP4](merge_cooperative/replay.mp4) |

固定巡航的负对照没有满足许可条件，C保持邻道。协作配置在请求后将B的前车从A切换为C，通过间距代价/约束让B主动减速，留出间距后再执行预设横向动作。并线完成后继续观察巡航恢复及A-C-B顺序。

结论限于本地图、初值、纵向模型和集中许可规则；横向换道轨迹尚未通过自行车运动学验证，无分布式协商或连续安全保证。下一步可扫描初始间距、请求时刻与通信扰动，再验证横向可达性。

七项标准：提交版本、地图与初始条件、控制周期、通信设置、原始日志与图片、RViz2回放、可播放MP4。两份模拟报告均逐项给出。

两个案例的真实ROS发布订阅检查通过：三车末态、轨迹、状态标签和相机TF与日志一致。 证据见[ROS验证](rviz_validation.json)、[离线验证](validation.json)、[独立审计](verification.json)、[源码与依赖](provenance.json)。完整封存前，尚未运行的验证文件不会被当作已通过。

本目录独立保留三车纵向优化、许可状态机、共享日志读取、图片/视频/报告导出、ROS回放及相关测试。
