cmake_minimum_required(VERSION 2.8.3)

project(gem_scenario_runner)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
   nodes/set_scene_three_100m_straight_roads.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch worlds)
	install(DIRECTORY ${dir}/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
