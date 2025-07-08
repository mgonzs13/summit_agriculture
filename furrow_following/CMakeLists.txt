
cmake_minimum_required(VERSION 3.5)
project(furrow_following)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  furrow_following/depth_image_centering_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME depth_image_centering_node
)

install(PROGRAMS
  furrow_following/furrow_following_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME furrow_following_node
)

ament_package()
