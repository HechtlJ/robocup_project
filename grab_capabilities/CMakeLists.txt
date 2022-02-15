cmake_minimum_required(VERSION 3.0.2)
project(grab_capabilities)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  move_base_msgs
  moveit_ros_planning_interface
  std_msgs
  std_srvs
  gpd_ros
  trajectory_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES move_object
  CATKIN_DEPENDS actionlib move_base_msgs roscpp std_msgs moveit_ros_planning_interface std_srvs trajectory_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR} #Needed to find all headers
)

add_executable(grabber src/grabber_node.cpp src/grabber.cpp)
target_link_libraries(grabber ${catkin_LIBRARIES} move_object)


