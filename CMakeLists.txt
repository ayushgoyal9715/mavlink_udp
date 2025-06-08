cmake_minimum_required(VERSION 3.8)
project(mavlink_udp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 and MAVLink dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(MAVLink REQUIRED)
include_directories($ENV{HOME}/mavlink/install/include)

add_executable(mavlink_udp src/mavlink_udp.cpp)

# Link include directories and dependencies
ament_target_dependencies(mavlink_udp
  rclcpp
  nav_msgs
)

target_link_libraries(mavlink_udp  MAVLink::mavlink)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS mavlink_udp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
