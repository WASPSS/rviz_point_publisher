cmake_minimum_required(VERSION 2.8.3)
project(rviz_point_publisher)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  visualization_msgs
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
