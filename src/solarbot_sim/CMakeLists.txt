cmake_minimum_required(VERSION 3.8)
project(solarbot_sim)



# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    worlds
    models
    materials
    config
    scripts
    plugins
  DESTINATION
    share/${PROJECT_NAME}
)
ament_package()
