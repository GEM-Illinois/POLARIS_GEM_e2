+ `launch/*.launch`  
  ROSlaunch scripts to spin up the Gazebo simulation world and models,
  particular component ROS nodes (lane detection, lane keeping, etc.),
  a ROS node that mimic resetting simulation runs (See description below),
  and a ROS node that records ROS topics with rosbag.

+ `nodes/set_scenes.py`  
  An ad-hoc script for directly setting the model state of the robot ignoring the
  physics. These scripts are used to set the vehicle to desired poses and record
  camera images.
  The file name for the specific Gazebo world is passed as a ROS parameter.
  The world files can be found under the `worlds` folder.

+ `nodes/sim_traces_*.py`  
  Ad-hoc scripts for directly setting the model state of the robot ignoring the
  physics. These scripts are used to simulate a run following a user-defined
  discrete dynamic equations.
  Each script is only for a specific Gazebo world file as indicated by the file
  name of the script. Using the script with unmatched world files can cause
  the model initialized within obstacles, etc.
  The world files can be found under the `worlds` folder.
