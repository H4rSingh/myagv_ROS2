cmake_minimum_required(VERSION 3.5)
project(myagv)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config urdf maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
