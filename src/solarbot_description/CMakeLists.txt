cmake_minimum_required(VERSION 3.8)
project(solarbot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    rviz
    urdf
    config
    scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()