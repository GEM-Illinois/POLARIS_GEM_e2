cmake_minimum_required(VERSION 3.0.2)
project(gem_pcl)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  rospy
  roslib
  std_msgs
  sensor_msgs
)

find_package(PCL REQUIRED)

## Add include directories
include_directories(include
  ${PCL_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS} 
)


add_executable(gem_velodyne src/gem_velodyne.cpp)
target_link_libraries(gem_velodyne ${catkin_LIBRARIES} ${PCL_LIBRARIES})



















