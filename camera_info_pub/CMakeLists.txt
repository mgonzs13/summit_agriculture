cmake_minimum_required(VERSION 3.5)
project(camera_info_pub)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  camera_info_pub/camera_info_pub_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_info_pub_node
)

ament_package()
