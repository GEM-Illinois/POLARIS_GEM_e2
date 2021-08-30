cmake_minimum_required(VERSION 2.8.3)
project(gem_lanenet)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        rospy
)

find_package(OpenCV REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(
	PROGRAMS nodes/lanenet_node.py
  	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

foreach(dir configs lanenet_weights launch)
	install(DIRECTORY ${dir}/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

