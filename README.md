# LIVO_SLAM

### 1.0 Framework

<div align="center"><img src="image/Framework.png" width=100% /></div>

## 2. Prerequisited

### 2.1 Ubuntu and ROS

Ubuntu 16.04~20.04.  [ROS Installation](http://wiki.ros.org/ROS/Installation).

### 2.2 PCL && Eigen && OpenCV

PCL>=1.6, Follow [PCL Installation](https://pointclouds.org/). 

Eigen>=3.3.4, Follow [Eigen Installation](https://eigen.tuxfamily.org/index.php?title=Main_Page).

OpenCV>=3.2, Follow [Opencv Installation](http://opencv.org/).

### 2.3 Sophus

 Sophus Installation for the non-templated/double-only version.

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build && cmake ..
make
sudo make install
```

### 2.4 Vikit

Vikit contains camera models, some math and interpolation functions that we need. Vikit is a catkin project, therefore, download it into your catkin workspace source folder.

```bash
cd catkin_ws/src
git clone https://github.com/uzh-rpg/rpg_vikit.git
```

### 2.5 **livox_ros_driver**

Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

## 3. Build

Clone the repository and catkin_make:

```
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/FAST-LIVO
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 4. Run
```

cd realsense_ws   打开相机ROS驱动
source devel/setup.bash
source ~/imu_transformer/devel/setup.bash
roslaunch realsense2_camera rs_livox.launch 

source ~/ws_livox/devel/setup.bash    打开激光雷达ROS驱动
roslaunch livox_ros_driver livox_lidar_msg.launch

source ~/catkin_ws/devel/setup.bash    执行LIVO_SLAM
roslaunch fast_livo mapping_livoxMid.launch
```

## 4. Example

<div align="center"><img src="image/建图结果.png" width=100% /></div>
