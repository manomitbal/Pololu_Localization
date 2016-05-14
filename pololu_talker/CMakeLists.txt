cmake_minimum_required(VERSION 2.8.3)
project(pololu_talker)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS tf roscpp rospy std_msgs sensor_msgs genmsg geometry_msgs)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/pololuPub.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker pololu_talker_generate_messages_cpp)
