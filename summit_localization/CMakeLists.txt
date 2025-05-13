cmake_minimum_required(VERSION 3.5)
project(summit_localization)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)


# install the launch directory
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  summit_localization/interactive_waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME interactive_waypoint_follower
)

install(PROGRAMS
  summit_localization/gps_waypoint_logger.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gps_waypoint_logger
)

install(PROGRAMS
  summit_localization/logged_waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME logged_waypoint_follower
)

install(PROGRAMS
  summit_localization/loop_waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME loop_waypoint_follower
)

install(PROGRAMS
  summit_localization/set_compass_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME set_compass_pose
)

ament_package()
