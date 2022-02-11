#!/bin/bash

DEVEL_DIR=$HOME/Public/gem_ws/devel
BAG_DIR=$HOME/Desktop  # Using absolute path is recommended
DURATION=1h
TRUTH_FILE=truths_configs/121_truths-evenly_spaced_11x11-1.2m-pi_12.yaml

source ${DEVEL_DIR}/setup.bash
export ROS_MASTER_URI=http://localhost:61804
export GAZEBO_MASTER_URI=http://localhost:61824

WORLD_NAME="$(rospack find gem_scenario_runner)/worlds/one_100m_left_curved_road.world"

# Launch ROS nodes Gazebo simulator in background
roslaunch gem_scenario_runner collect_images_w_truths.launch bag_dir:=${BAG_DIR} \
    truth_file:=${TRUTH_FILE} gui:=true \
    world_name:=${WORLD_NAME} \
    record:=true duration:=${DURATION} reset_period:=0.25 &
ROSLAUNCH_PID=$!

# Wait for all nodes finish initialization
sleep 15s

# Unpause to start simulation
rosservice call --wait /gazebo/unpause_physics

# Wait for any background command to finish
wait -n ${ROSLAUNCH_PID}
kill -s SIGINT ${ROSLAUNCH_PID}
