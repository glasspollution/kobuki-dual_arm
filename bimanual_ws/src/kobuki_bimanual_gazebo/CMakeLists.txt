cmake_minimum_required(VERSION 3.16)
project(kobuki_bimanual_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds models launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
