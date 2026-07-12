cmake_minimum_required(VERSION 3.8)
project(qcar_ackermann)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  models
  rviz
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()