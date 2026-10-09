cmake_minimum_required(VERSION 3.8)
project(car_swarm_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajectoryPoint.msg"
  "msg/PredictedTrajectory.msg"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

add_library(hybrid_astar_planner
  src/reeds_shepp.cpp
  src/hybrid_astar_planner.cpp # ROS 节点可以调用这个库
)

target_include_directories(hybrid_astar_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(hybrid_astar_planner PUBLIC cxx_std_17)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp
)

add_library(trajectory_collision src/trajectory_collision.cpp)
target_include_directories(trajectory_collision PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(trajectory_collision PUBLIC cxx_std_17)
target_link_libraries(trajectory_collision PUBLIC "${cpp_typesupport_target}")

add_library(trajectory_yield src/trajectory_yield.cpp)
target_link_libraries(trajectory_yield PUBLIC
  trajectory_collision
  hybrid_astar_planner
)

add_library(trajectory_tracking src/trajectory_tracking.cpp src/quintic_trajectory.cpp)
target_link_libraries(trajectory_tracking PUBLIC trajectory_yield)

add_executable(compare_spatial_temporal test/compare_spatial_temporal.cpp)
target_link_libraries(compare_spatial_temporal trajectory_tracking)

add_executable(yield_demo_node src/yield_demo_node.cpp)
target_link_libraries(yield_demo_node trajectory_tracking hybrid_astar_planner)
ament_target_dependencies(yield_demo_node
  rclcpp nav_msgs visualization_msgs geometry_msgs std_msgs tf2_ros
)

add_executable(agent_node
  src/agent_node.cpp
)

target_link_libraries(agent_node
  hybrid_astar_planner
)

ament_target_dependencies(agent_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)

add_executable(vehicle_tf_broadcaster
  src/vehicle_tf_broadcaster.cpp
)

ament_target_dependencies(vehicle_tf_broadcaster
  rclcpp
  geometry_msgs
  tf2_ros
  nav_msgs
)

install(TARGETS
  hybrid_astar_planner
  trajectory_collision
  trajectory_yield
  trajectory_tracking
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  agent_node
  vehicle_tf_broadcaster
  yield_demo_node
  compare_spatial_temporal
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)
#install launch files.
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME})
#构建时，把 launch 文件夹复制到 ROS package 的安装目录中；否则 ros2 launch 找不到它。

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_spatial_temporal_math
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_spatial_temporal_math.py)
  add_test(NAME test_decentralized_mpc
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_decentralized_mpc.py)

  add_executable(diagnose_hybrid_astar test/diagnose_hybrid_astar.cpp)
  target_link_libraries(diagnose_hybrid_astar hybrid_astar_planner)

  add_executable(benchmark_hybrid_astar test/benchmark_hybrid_astar.cpp)
  target_link_libraries(benchmark_hybrid_astar hybrid_astar_planner)

  ament_add_gtest(
    test_hybrid_astar_planner
    test/test_hybrid_astar_planner.cpp
  )

  target_link_libraries(
    test_hybrid_astar_planner
    hybrid_astar_planner
  )

  ament_add_gtest(test_trajectory_collision test/test_trajectory_collision.cpp)
  target_link_libraries(test_trajectory_collision trajectory_collision)

  ament_add_gtest(test_trajectory_yield test/test_trajectory_yield.cpp)
  target_link_libraries(test_trajectory_yield trajectory_yield hybrid_astar_planner)
  ament_add_gtest(test_continuous_trajectory
  test/test_continuous_trajectory.cpp
  )
  target_link_libraries(test_continuous_trajectory
    trajectory_yield
  )
  ament_add_gtest(test_quintic_trajectory test/test_quintic_trajectory.cpp)
  target_link_libraries(test_quintic_trajectory trajectory_tracking)

  ament_add_gtest(test_trajectory_tracking test/test_trajectory_tracking.cpp)
  target_link_libraries(test_trajectory_tracking trajectory_tracking)
endif()

# 先生成自定义消息的 C++ 类型，再让新节点链接消息类型支持。这里消息和节点放在同一个包，使用 rosidl_get_typesupport_target 连接二者。#
add_executable(trajectory_comm_node
  src/trajectory_comm_node.cpp
)

target_compile_features(trajectory_comm_node PUBLIC cxx_std_17)

ament_target_dependencies(trajectory_comm_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

target_link_libraries(trajectory_comm_node
  hybrid_astar_planner
  "${cpp_typesupport_target}"
)

install(TARGETS trajectory_comm_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
