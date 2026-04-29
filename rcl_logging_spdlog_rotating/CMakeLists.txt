cmake_minimum_required(VERSION 3.5)
project(rcl_logging_spdlog_rotating)

# ✅ 使用 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rcl_logging_interface REQUIRED)
find_package(rcutils REQUIRED)
find_package(spdlog REQUIRED)

add_library(rcl_logging_spdlog_rotating SHARED
  src/rcl_logging_spdlog_rotating.cpp
)

target_link_libraries(rcl_logging_spdlog_rotating
  spdlog::spdlog
)

# ⚠️ 兼容老 GCC（一般不需要，但加上更稳）
if(CMAKE_COMPILER_IS_GNUCXX)
  target_link_libraries(rcl_logging_spdlog_rotating stdc++fs)
endif()

ament_target_dependencies(rcl_logging_spdlog_rotating
  rcl_logging_interface
  rcutils
)

install(TARGETS rcl_logging_spdlog_rotating
  LIBRARY DESTINATION lib
)

ament_package()