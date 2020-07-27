cmake_minimum_required(VERSION 2.8.3)
project(turtlesim_vacuum)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# 1. Give a name to the node, specify the file.            
add_executable(turtlesim_cleaner_node src/robot_vacuum.cpp)
target_link_libraries (turtlesim_cleaner_node ${catkin_LIBRARIES})