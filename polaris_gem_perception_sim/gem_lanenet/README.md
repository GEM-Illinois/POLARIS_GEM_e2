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

