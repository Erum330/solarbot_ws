cmake_minimum_required(VERSION 3.8)
project(solarbot_navigation)

find_package(ament_cmake REQUIRED)

# Install configuration and launch files into the share directory
install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

# Install executable python scripts into lib/solarbot_navigation
install(
  PROGRAMS
    scripts/solarbot_coverage_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()