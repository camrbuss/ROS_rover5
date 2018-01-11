cmake_minimum_required(VERSION 2.8.3)
project(mecanum_driver)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
sensor_msgs
genmsg
message_generation)

## Declare ROS messages and services
add_message_files(FILES wheelSpeed.msg)



## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs mecanum_driver)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


install(DIRECTORY msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.msg"
)


