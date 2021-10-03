# Polaris GEM e2 Simulator

This repository provides a simulated vehicle model of [Polaris GEM e2 Electric Cart](https://gem.polaris.com/en-us/e2/) in the [Gazebo](http://gazebosim.org/) simulation environment as well as [ROS](https://www.ros.org/) based sensors and controllers for autonomous-driving. The Polaris GEM e2 vehicle model was measured and modeled using Solidworks by Hang Cui and Jiaming Zhang. Hang Cui further constructed the URDF files of the vehicle model compatible with ROS, RViz, and Gazebo.

The simulator was initially developed for personal research with ROS Melodic and Gazebo 9 in Ubuntu 18.04 in Fall 2019. This simulator then became an essential teaching material for the course, [Principles of Safe Autonomy @ Illinois](https://publish.illinois.edu/safe-autonomy/), and the project subsequently received funding from the [Center for Autonomy](https://autonomy.illinois.edu/) at University of Illinois at Urbana-Champaign. Hang Cui further developed and upgraded the simulator to use ROS Noetic and Gazebo 11 in Summer 2021. This simulator is currently under active development for research and teaching.


## Requirements

Our simulation setup is currently tested only with the following system and ROS packages.

**System**: Ubuntu 20.04 + ROS Noetic (Gazebo 11)

We refer readers to http://wiki.ros.org/noetic/Installation/Ubuntu and follow the instructions to install ROS noetic and Gazebo 11.
We also recommend **Desktop-Full Install** as suggested in the instructions.

**Required ROS Packages**:

+ ackermann_msgs
+ geometry2
+ hector_gazebo
+ hector_models
+ jsk_rviz_plugins
+ ros_control
+ ros_controllers
+ velodyne_simulator

After the installation of ROS Noetic and Gazebo 11 on Ubuntu 20.04, we recommend installing ROS packages using APT as follows
```bash
$ sudo apt install ros-noetic-ackermann-msgs ros-noetic-geometry2 \
    ros-noetic-hector-gazebo ros-noetic-hector-models ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-velodyne-simulator
```

## Compile Polaris GEM e2 Simulator

We assume the Catkin workspace is under `~/gem_ws`. We first clone this repository to `~/gem_ws/src`.
For example,
```bash
$ mkdir -p ~/gem_ws/src
$ cd ~/gem_ws/src
$ git clone https://github.com/GEM-Illinois/POLARIS_GEM_e2.git
```

Then we compile the whole workspace use `catkin_make`
```bash
$ source /opt/ros/noetic/setup.bash
$ cd ~/gem_ws
$ catkin_make
```
For more detail about Catkin workspac, please refer to the tutorialsat http://wiki.ros.org/catkin/Tutorials/create_a_workspace.

## Usage

### Simple Track Environment

```bash
$ source devel/setup.bash
$ roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:="true"
```

<a href="url"><img src="./images/simple_track_rviz.png" width="600"></a>

<a href="url"><img src="./images/simple_track_gazebo.png" width="600"></a>


### Geometric based Lateral Controller

```bash
$ source devel/setup.bash
$ roslaunch gem_gazebo gem_gazebo_rviz.launch

$ source devel/setup.bash
$ roslaunch gem_gazebo gem_sensor_info.launch

$ source devel/setup.bash
$ rosrun gem_pure_pursuit_sim pure_pursuit_sim.py
```

<a href="url"><img src="./images/pp_controller.gif" width="600"></a>

```bash
$ source devel/setup.bash
$ rosrun gem_stanley_sim stanley_sim.py
```

<a href="url"><img src="./images/stanley_controller_rviz.gif" width="600"></a>

<a href="url"><img src="./images/stanley_controller_gazebo.gif" width="600"></a>


### Highbay Environment

```bash
$ source devel/setup.bash
$ roslaunch gem_gazebo gem_gazebo_rviz.launch world_name:="highbay_track.world" x:=-5.5 y:=-21 velodyne_points:="true"

$ source devel/setup.bash
$ roslaunch gem_gazebo gem_sensor_info.launch
```

<a href="url"><img src="./images/highbay_rviz.png" width="600"></a>

<a href="url"><img src="./images/highbay_gazebo.png" width="600"></a>

## Modules

Coming soon...

## Developers:
+ Hang Cui <hangcui1201@gmail.com>

## Contributors
+ Jiaming Zhang <jz73@illinois.edu>
