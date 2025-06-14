cmake_minimum_required(VERSION 3.22)
project(iisy_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(teleoperator_node src/teleoperator_node.cpp)

# Link dependencies
ament_target_dependencies(
  teleoperator_node
  moveit_ros_planning_interface
  rclcpp
  geometry_msgs
)

# Install targets
install(TARGETS teleoperator_node
  DESTINATION lib/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()