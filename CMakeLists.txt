cmake_minimum_required(VERSION 2.8.3)
project(explore_borders)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  nav_core
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   loop_array.msg
#   Message2.msg
# )

# add_service_files(
#    FILES
#	GetClicks.srv
#)

 generate_messages(
   DEPENDENCIES
	geometry_msgs	#actionlib_msgs#      nav_msgs#   std_msgs#   visualization_msgs
 )
 
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES explore_borders
   CATKIN_DEPENDS  nav_core geometry_msgs message_generation  roscpp std_msgs rospy #actionlib #actionlib_msgs #nav_msgs #tf #visualization_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(explore_node src/explore_node.cpp)
target_link_libraries(explore_node  ${catkin_LIBRARIES})
add_dependencies(explore_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(laser_hot_fix src/laser_hot_fix.cpp)
target_link_libraries(laser_hot_fix  ${catkin_LIBRARIES})
add_dependencies(laser_hot_fix ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
