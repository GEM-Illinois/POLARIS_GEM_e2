cmake_minimum_required(VERSION 2.8.3)

project(gem_scenario_runner)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

install(PROGRAMS
   nodes/set_scene_three_100m_straight_roads.py
   nodes/sim_trace_three_100m_straight_roads.py
   nodes/sim_trace_one_100m_curved_road.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir configs launch worlds)
	install(DIRECTORY ${dir}/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
