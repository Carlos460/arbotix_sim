cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

# compile command on for lsp
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(udp_joint_bridge src/udp_joint_bridge.cpp)
ament_target_dependencies(udp_joint_bridge rclcpp std_msgs sensor_msgs)

install(TARGETS
  udp_joint_bridge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../hexapod_urdf.xacro
  DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../meshes
  DESTINATION share/${PROJECT_NAME})

ament_package()
