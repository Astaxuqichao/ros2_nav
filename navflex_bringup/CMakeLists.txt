cmake_minimum_required(VERSION 3.10)
project(navflex_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/odom_fake_localization.py
  scripts/publish_geojson_marker_array.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
