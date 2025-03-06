
cmake_minimum_required(VERSION 3.5)
project(summit_cornfield)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
