cmake_minimum_required(VERSION 3.10)
project(navflex_exclusion_zone)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------- 依赖 ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(DEPS
  rclcpp
  rclcpp_lifecycle
  nav2_util
  nav2_costmap_2d
  geometry_msgs
  std_msgs
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ---------- 插件共享库 ----------
add_library(${PROJECT_NAME} SHARED
  src/exclusion_zone_layer.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} ${DEPS})

# 向 pluginlib 注册插件描述文件
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

# ---------- 安装 ----------
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

# ---------- ament ----------
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPS})

ament_package()
