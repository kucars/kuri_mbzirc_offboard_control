cmake_minimum_required(VERSION 2.8.3)
project(kuri_mbzirc_offboard_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf
  tf_conversions
  eigen_conversions
)

catkin_package(
INCLUDE_DIRS include
)

include_directories(include
${catkin_INCLUDE_DIRS}
)

add_executable(pub_att src/pub_att.cpp)
target_link_libraries(pub_att ${catkin_LIBRARIES})

add_executable(offb_node src/offb_node.cpp)
target_link_libraries(offb_node ${catkin_LIBRARIES})

add_executable(pub_setpoints src/pub_setpoints.cpp)
target_link_libraries(pub_setpoints ${catkin_LIBRARIES})

add_executable(offb_node_mod src/offb_node_mod.cpp)
target_link_libraries(offb_node_mod ${catkin_LIBRARIES})

add_executable(offb_node_mod_zcons src/offb_node_mod_zcons.cpp)
target_link_libraries(offb_node_mod_zcons ${catkin_LIBRARIES})

add_executable(offb_node_pid src/offb_node_pid.cpp)
target_link_libraries(offb_node_pid ${catkin_LIBRARIES})

