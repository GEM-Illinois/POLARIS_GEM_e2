cmake_minimum_required(VERSION 3.0.2)
project(gem_lane_detect_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
# We want boost/format.hpp, which isn't in its own component.
find_package(Boost REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES SimpleLane.msg SimpleLaneStamped.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
  DEPENDS Boost)
