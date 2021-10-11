#!/bin/bash

DEVEL_DIR=$HOME/Public/gem_ws/devel
OUT_DIR=$HOME/Desktop  # Using absolute path is recommended

source ${DEVEL_DIR}/setup.bash
export ROS_MASTER_URI=http://localhost:61808
export GAZEBO_MASTER_URI=http://localhost:61828

# Launch ROS nodes Gazebo simulator in background
roslaunch gem_scenario_runner sim_traces_montecarlo.launch out_dir:=${OUT_DIR} \
    gui:=false controller:=pure_pursuit &
ROSLAUNCH_PID=$!

# Wait for all nodes finish initialization
sleep 15s

# Unpause to start simulation
rosservice call --wait /gazebo/unpause_physics

# Wait for any background command to finish
wait -n ${ROSLAUNCH_PID}
kill -s SIGINT ${ROSLAUNCH_PID}
