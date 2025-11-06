cmake_minimum_required(VERSION 3.16)
project(cpp_control_pkg)

# C++
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Ament / ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(nlohmann_json REQUIRED)   # provides nlohmann_json::nlohmann_json
find_package(sensor_msgs REQUIRED)

# (Optional, only if you start adding headers under include/)
# target_include_directories will be added per-target if needed:
# set(PROJ_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ---- Nodes ----
add_executable(controller_pub_node src/server_publisher.cpp)
ament_target_dependencies(controller_pub_node rclcpp std_msgs geometry_msgs rcutils sensor_msgs)
# target_include_directories(controller_pub_node PUBLIC ${PROJ_INCLUDE_DIR})
target_link_libraries(controller_pub_node nlohmann_json::nlohmann_json)

add_executable(master_node src/masternode.cpp)
ament_target_dependencies(master_node rclcpp std_msgs geometry_msgs rcutils sensor_msgs)
# target_include_directories(master_node PUBLIC ${PROJ_INCLUDE_DIR})
target_link_libraries(master_node nlohmann_json::nlohmann_json)

add_executable(local_controller_node src/localcontroller.cpp)
ament_target_dependencies(local_controller_node rclcpp std_msgs geometry_msgs sensor_msgs)
# link nlohmann only if that file actually uses it; otherwise omit
# target_link_libraries(local_controller_node nlohmann_json::nlohmann_json)

# ---- Install ----
install(TARGETS
  controller_pub_node
  master_node
  local_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()