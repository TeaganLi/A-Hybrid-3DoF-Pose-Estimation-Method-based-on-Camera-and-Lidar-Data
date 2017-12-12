# A Hybrid 3DoF Pose Estimation Method based on Camera and Lidar Data

This repository implements a hybrid 3DoF pose estimation method combining RGB camera and single-channel Lidar data. The detailed introduction can be found in ＂A Hybrid 3DoF Pose Estimation Method based on Camera and Lidar Data＂, ROBIO 2017.

## Overview

This repository is based on ROS and includes three packages, where library contains driver and basic process for Hokuyo urg-04lx-ug01 Lidar, lidar_based_method and vision_based_method estimate targets' pose using Lidar and camera, respectively. 

### Prerequisites

Since ROS is utilized to subscribe data, please make sure you have installed ROS indigo/kinetic.

### Installing

After you have download this repository, make the workspace at the root directory using

```
catkin_make
```

And source 

```
source ./devel/setup.bash
```

## Running 

After you successfully make all codes, you can run the code using (make sure you have pluged in the camera and Lidar)

```
roslaunch lidar_based_method lidar.launch
```

```
roslaunch vision_based_method vision.launch
```

Now you can check the rostopic list 

```
rostopic list
```

where "block_single_location" and "camera_pose" give the estimation from lidar and camera, respectively. And now you can further fuse these two estimation to get a more robust result.

## Noted

If you use this code for academic purposes, please cite it as:
```
@inproceedings{Tingguang:2017,
  author          = {Tingguang Li, Delong Zhu, and Max Q.-H. Meng},
  title           = {A Hybrid 3DoF Pose Estimation Method based Camera and Lidar Data},
  booktitle       = {Robotics and Biomimetics (ROBIO), 2017 IEEE International Conference on},
  year            = {2017},
  organization      ={IEEE}
}
```
