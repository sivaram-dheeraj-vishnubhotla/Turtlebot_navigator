cmake_minimum_required(VERSION 3.16)
project(turtlebot_navigator)

find_package(ament_cmake REQUIRED)

# Install the simulation launch files, robot assets and supporting configuration.
install(DIRECTORY config launch models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# The shared planner is imported by the ROS entry point from this directory.
install(FILES scripts/astar_search.py scripts/motion_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/navigate_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
