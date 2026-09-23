cmake_minimum_required(VERSION 3.8)
project(car_swarm_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(hybrid_astar_planner
  src/hybrid_astar_planner.cpp # ROS 节点可以调用这个库
)

target_include_directories(hybrid_astar_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(hybrid_astar_planner PUBLIC cxx_std_17)

add_executable(agent_node
  src/agent_node.cpp
)

target_link_libraries(agent_node
  hybrid_astar_planner
)

ament_target_dependencies(agent_node
  rclcpp
  std_msgs
)

install(TARGETS
  hybrid_astar_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  agent_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)
#install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})
#构建时，把 launch 文件夹复制到 ROS package 的安装目录中；否则 ros2 launch 找不到它。

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()