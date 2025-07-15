
cmake_minimum_required(VERSION 3.5)
project(summit_cornfield)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  summit_cornfield/add_corns_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME add_corns_node
)

install(PROGRAMS
  summit_cornfield/add_maizes_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME add_maizes_node
)

ament_package()
