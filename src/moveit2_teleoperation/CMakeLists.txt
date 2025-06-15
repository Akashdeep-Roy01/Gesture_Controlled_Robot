cmake_minimum_required(VERSION 3.22)
project(moveit2_teleoperation)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)

add_executable(joint_angle_publisher src/joint_angle_publisher.cpp)
ament_target_dependencies(joint_angle_publisher rclcpp std_msgs sensor_msgs)

install(TARGETS
  joint_angle_publisher
  DESTINATION lib/${PROJECT_NAME})

add_executable(teleoperator_node src/teleoperator_node.cpp)

# Link dependencies
ament_target_dependencies(
  teleoperator_node
  moveit_ros_planning_interface
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Install targets
install(TARGETS teleoperator_node
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
