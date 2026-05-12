cmake_minimum_required(VERSION 3.10)
project(navflex_costmap_nav)

# 编译参数
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(angles REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(navflex_utility REQUIRED)
find_package(navflex_cmdbehavior REQUIRED)

# 包含头文件目录
include_directories(
  include
)

# 添加组件库（共享库）
add_library(navflex_costmap_nav SHARED
  src/navflex_costmap_nav/navflex_costmap_nav.cpp
  src/navflex_costmap_nav/planner_action_costmap_server.cpp
  src/navflex_costmap_nav/controller_action_costmap_server.cpp
  src/navflex_costmap_nav/behavior_action_costmap_server.cpp
  src/navflex_base/planner_action.cpp
  src/navflex_base/planner_execution.cpp
  src/navflex_base/navflex_execution_base.cpp
  src/navflex_base/controller_action.cpp
  src/navflex_base/controller_execution.cpp
  src/navflex_base/behavior_action.cpp
  src/navflex_base/behavior_execution.cpp
)

set(NAVFLEX_COSTMAP_DEPS
  rclcpp
  rclcpp_lifecycle
  nav2_util
  nav2_core
  nav2_costmap_2d
  rclcpp_components
  nav2_msgs
  nav_msgs
  geometry_msgs
  pluginlib
  tf2_ros
  angles
  rclcpp_action
  nav_2d_utils
  nav_2d_msgs
  navflex_utility
)

ament_target_dependencies(navflex_costmap_nav ${NAVFLEX_COSTMAP_DEPS})

# 注册组件节点
rclcpp_components_register_nodes(navflex_costmap_nav "navflex_costmap_nav::CostmapNavNode")

# 添加可执行节点（通常作为组件管理器）
add_executable(costmap_nav_node src/main.cpp)

ament_target_dependencies(costmap_nav_node ${NAVFLEX_COSTMAP_DEPS})

# 链接组件库到可执行节点
target_link_libraries(costmap_nav_node
  navflex_costmap_nav
)

# 安装可执行文件
install(TARGETS costmap_nav_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
  scripts/nav_test.py
  scripts/route_nav.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装组件库
install(TARGETS navflex_costmap_nav
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})


# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
