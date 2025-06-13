cmake_minimum_required(VERSION 3.5)

project(kuka_lbr_iisy_support)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

install(DIRECTORY launch meshes urdf config
  DESTINATION share/${PROJECT_NAME})

ament_package()
