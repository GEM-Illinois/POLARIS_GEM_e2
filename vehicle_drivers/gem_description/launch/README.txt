

$ roslaunch gem_description gem_vehicle.launch

$ rostopic list
/gem_vehicle/joint_states
/rosout
/rosout_agg
/tf
/tf_static


$ rosservice list
/gem_vehicle/gem_state_publisher/get_loggers
/gem_vehicle/gem_state_publisher/set_logger_level
/rosout/get_loggers
/rosout/set_logger_level



$ roslaunch gem_description gem_vehicle_rviz.launch  # depend on gem_vehicle.launch
