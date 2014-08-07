cmake_minimum_required(VERSION 2.8.3)
project(highlevel_planner)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation)

## Declare ROS messages and services
add_message_files(FILES CoorData.msg SpWaypoints.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker highlevel_planner_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener highlevel_planner_generate_messages_cpp)

add_executable(simple src/simple_planner.cpp src/simpleAlgorithm.cpp src/dataManager.cpp)
target_link_libraries(simple ${catkin_LIBRARIES})
add_dependencies(simple highlevel_planner_generate_messages_cpp)

add_executable(flock src/flock_planner.cpp src/flockAlgorithm.cpp src/dataManager.cpp)
target_link_libraries(flock ${catkin_LIBRARIES})
add_dependencies(flock highlevel_planner_generate_messages_cpp)
