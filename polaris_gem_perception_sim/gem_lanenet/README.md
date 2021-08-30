# LaneNet Lane Detection for Polaris GEM E2 Simulator

## Installation

We assume users already set up the catkin workspace and compile the ROS packages.
The following are additional steps to install Python packages and set up the pretrained model.

### Install Python Dependencies

```bash
pip3 install -r requirements.txt
```
You can add the `--user` option if you are installing without virtual environments or root permissions. 

### Download the pretrained LaneNet model

Thanks to the original developer @MaybeShewill-CV,
the model trained with TUSimple dataset is available at his Dropbox 
https://www.dropbox.com/sh/0b6r0ljqi76kyg9/AADedYWO3bnx4PhK1BmbJkJKa?dl=0

Please download the files and put them under the `lanenet_weights` folder.
As of Aug. 30, 2021, the following files are included:

+ checkpoint
+ tusimple_lanenet.ckpt.data-00000-of-00001
+ tusimple_lanenet.ckpt.index
+ tusimple_lanenet.ckpt.meta

## Demo

Use the following ROS launch script to start up Gazebo simulation and
a ROS node using LaneNet to process camera images and detect lane markings.
```bash
source devel/setup.bash
roslaunch gem_lanenet demo_lane_detection.launch use_gpu:=true 
```
The argument `use_gpu:=true` is optional. You can specify `use_gpu:=false` to use CPU for TensorFlow 2 instead.

This will start Gazebo simulation in a **paused** state.
Unpause the simulation once the initialization messages are done.
Check the rviz for the annotated image with estimated lane curvature, offset,
and the region between fitted lane edge curves.

Open another terminal to run GPS base pure pursuit controller using the commands below,
and observe the behavior of the lane detection result from LaneNet.
```bash
source devel/setup.bash
rosrun gem_pure_pursuit_sim pure_pursuit_sim.py
```

A demo video is available at https://drive.google.com/file/d/1pqA2Z7z9lszptnTtkZlI2waJsMKwNKCf/view?usp=sharing
