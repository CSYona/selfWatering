cmake_minimum_required(VERSION 3.5)
project(robot_localization_config)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_localization REQUIRED)

# Install configuration files
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# ament package
ament_package()
