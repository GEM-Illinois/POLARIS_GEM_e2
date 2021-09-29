+ `launch/*.launch`  
  ROSlaunch scripts to spin up the Gazebo simulation world and models,
  particular component ROS nodes (lane detection, lane keeping, etc.),
  a ROS node that mimic resetting simulation runs (See description below),
  and a ROS node that records ROS topics with rosbag.

+ `nodes/set_pose_*.py`  
  Ad-hoc scripts for directly setting the model state of the robot ignoring the
  physics. These scripts are used to mimic resetting a simulation run with
  a desired initial pose of the robot.
  Each script is only for a specific Gazebo world file as indicated by the file
  name of the script. Using the script with unmatched world files can cause
  the model initialized within obstacles, etc.
  The world files can be found under the `worlds` folder.
