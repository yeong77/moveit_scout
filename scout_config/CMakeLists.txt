cmake_minimum_required(VERSION 3.1.3)
project(scout_config)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  moveit_msgs
  tf2
  tf2_geometry_msgs
  moveit_ros_planning_interface
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs moveit_msgs tf2 tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(trajectory_to_cmd_vel src/trajectory_to_cmd_vel.cpp)
target_link_libraries(trajectory_to_cmd_vel ${catkin_LIBRARIES})

add_executable(moveit_set_goal_pose_cpp src/SetGoalpose.cpp)
target_link_libraries(moveit_set_goal_pose_cpp ${catkin_LIBRARIES})


# Install launch and config folders
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

