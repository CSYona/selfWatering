################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(stella_navigation2)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch map param rviz waypoints
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(
  PROGRAMS
    scripts/waypoint_nav.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()