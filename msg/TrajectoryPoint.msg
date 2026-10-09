# 相对于整条轨迹 header.stamp 的预计到达时间
# builtin_interfaces/Duration：builtin_interfaces 包提供的 Duration 类型。
builtin_interfaces/Duration time_from_start

# 后轴中心的位姿；orientation 表示车头朝向
geometry_msgs/Pose pose

int8 REVERSE=-1
int8 STOP=0
int8 FORWARD=1

# 从前一点运动到本点的方向；第一个点为 STOP
int8 direction