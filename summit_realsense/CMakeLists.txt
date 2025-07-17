
cmake_minimum_required(VERSION 3.5)
project(summit_realsense)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY launch robots
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
