#新增双车 launch，并明确区分话题命名空间与 TF 名称
#数据流如下：
# /car_1/trajectory_comm
#   发布 /car_1/predicted_trajectory
#   接收 /car_2/predicted_trajectory
#   显示 /car_1/received_neighbor_path

# /car_2/trajectory_comm
#   发布 /car_2/predicted_trajectory
#   接收 /car_1/predicted_trajectory
#   显示 /car_2/received_neighbor_path
# TF 则是：
# map
# ├── car_1/base_link
# └── car_2/base_link
# 节点命名空间不会自动修改消息里的 frame_id。 所以话题用 namespace 区分，TF 子坐标系由代码显式拼出车辆前缀。两车共享 map 是为了让位置具有共同参照。
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="car_swarm_agent",
            executable="trajectory_comm_node",
            namespace="car_1",
            name="trajectory_comm",
            output="screen",
            parameters=[{
                "vehicle_id": "car_1",
                "neighbor_topic": "/car_2/predicted_trajectory",
                "lane_y": 0.0,
                "use_sim_time": False,
                "goal_x": 18.0,
                "forward_speed": 1.0,
                "reverse_speed": 0.5,
            }],
        ),
        Node(
            package="car_swarm_agent",
            executable="trajectory_comm_node",
            namespace="car_2",
            name="trajectory_comm",
            output="screen",
            parameters=[{
                "vehicle_id": "car_2",
                "neighbor_topic": "/car_1/predicted_trajectory",
                "lane_y": 3.0,
                "use_sim_time": False,
                "goal_x": 18.0,
                "forward_speed": 1.0,
                "reverse_speed": 0.5,
            }],
        ),
    ])