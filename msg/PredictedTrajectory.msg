# stamp：本条预测的时间基准；frame_id：所有点所在坐标系
std_msgs/Header header

string vehicle_id

car_swarm_agent/TrajectoryPoint[] points

# 离线闭环轮次，-1 为初始预测；用于区分同一纪元的初始与第 0 轮预测
int64 prediction_sequence
