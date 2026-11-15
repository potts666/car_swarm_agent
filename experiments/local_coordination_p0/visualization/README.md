# 四车实际运动 RViz2 回放

默认显示最新已通过独立验收的encounter_coordination_001数值臂，1753拍/350.6秒、4倍速、循环。四车同步更新；沿用日志中的实际状态，展示B1等待、出口域换权和恢复。旧P0及A/B日志仍可用--run选择。只读取已有日志，不求解、不运行控制实验，不发布车辆控制指令。新增可视化文件不修改A/B/C/P0门禁及历史数据。

在Ubuntu桌面终端、工程根目录运行：

```bash
./experiments/local_coordination_p0/visualization/start_replay.sh
```

原速并在结束时保持最终画面：

```bash
./experiments/local_coordination_p0/visualization/start_replay.sh --rate 1 --once
```

切换到已验收的B阶段数值候选实际运行：

```bash
./experiments/local_coordination_p0/visualization/start_replay.sh \
 --run experiments/local_coordination_p0/stage_b/results/comparison_002/curve_B-ZG \
 --rate 1 --once
```

B例只有239拍首条连接和下一车首拍（48秒），界面标记PARTIAL INTERFACE RUN；它不是四车全部完成。A的实际接口记录也可用--run指定。C0没有接受的新JL运动，C2未执行，不能将其预测点用作实际回放。

- A1蓝、A2橙、B1绿、B2紫；立方体为4.5×1.8m矩形车身，白箭头为朝向，黑短线为前后轮。
- 车体按后轴到车身中心1.25m偏移，前轮显示psi+delta；B车使用真实航向pi+theta。
- 细淡彩色线为已验收完整路径，粗彩色线为已执行轨迹，彩色十字为目标；灰色为原地图道路，金色竖线标出实际核心资源边界x=±16。
- 标题显示回放时刻、方法、完整/局部状态；第二行显示最后执行车辆、分组及剩余预算。
- 数据是每0.2秒保存的实际状态，视觉上取位置/航向区间中点并逐帧保持，没有用未接受计划生成“实际轨迹”。该显示不替代原连续安全验收。

鼠标滚轮缩放，拖动画面查看车辆。左侧MarkerArray的Topic是/dmpc_replay/markers，Fixed Frame为map。窗口初始俯视全地图。

脚本沿用已安装ROS2 Jazzy，无需重新colcon构建。默认独立ROS_DOMAIN_ID=93（终端已设置时沿用）；只启用一个回放器，避免两个记录同时发布同一主题。Ctrl+C结束启动命令会同时关闭其RViz窗口；关闭RViz窗口也会结束它的回放节点。

仅启动发布节点：`start_replay.sh --no-rviz`；再手动打开：

```bash
ROS_DOMAIN_ID=93 rviz2 -d experiments/local_coordination_p0/visualization/fourcar.rviz
```

已有回放运行时核对真实ROS消息（不会启动实验）：

```bash
ROS_DOMAIN_ID=93 /usr/bin/python3 experiments/local_coordination_p0/visualization/check_replay.py
```

verification.json保存四车尺寸、B航向、后轴偏移、完整/局部记录、真实动态消息及发布主题核对结果。若从没有DISPLAY的远程终端启动，应在Ubuntu桌面终端运行；VirtualBox OpenGL异常可在启动前设置`LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=xcb`。

默认最新数据源：`../encounter_coordination_001/results/actual.jsonl.gz`，先检查GATE、独立物理验收与轨迹SHA。B车朝向为pi+theta，车身后轴偏移1.25m。界面速度表示实际运动状态，不等同于eta有效进度计数。回放不重跑MPC，不改写冻结证据。
