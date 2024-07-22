# ScoutAir Planner
Maintainer: Yuchen Xia

## Introduction 

<p align="center">
<img src="./doc/FIS.png" alt="Frontier Map" width="300" height="200">
</p>

With this Repo, you can ...

1. Refering build section fo setup the package:
    - [Prerequisites](#prerequisites)
    - [Build](#build)
    - [Run](#run)

2. Following the brief introduction for



## Prerequisites
1. __Ubuntu and ROS__

   This package is intended to be used with **Ubuntu 20.04** and **ROS Noetic**.

2. __Dependencies__

   First you need to install the following dependencies:

   * [voxblox](https://github.com/ethz-asl/voxblox)   ( See [Installation Dokumentation here](https://voxblox.readthedocs.io/en/latest/pages/Installation.html) )

   (Voxblox: A library for flexible voxel-based mapping, mainly focusing on truncated and Euclidean signed distance fields.)

   * [mav_voxblox_planning](https://github.com/ethz-asl/mav_voxblox_planning?tab=readme-ov-file) 

   (Mav Voxblox Planning: MAV planning tools using voxblox as the map representation.)

   * [rotors_simulator](https://github.com/ethz-asl/rotors_simulator#installation-instructions---ubuntu-1604-with-ros-kinetic) 

   (Rotors Simulator: An MAV simulator built on top of gazebo.This will allow us to fully simulate a typical MAV, with a visual-inertial sensor mounted on it.)

## Build
Create a ROS workspace:

```
mkdir -p ~/vox_ws/src
```


If already done so, clone repo `ScoutAir_Planner` to your workspace and build packages:

```
cd vox_ws/src
git clone https://...
cd ..

catkin build scoutair_planner
source devel/setup.bash
```

## Run

First time running the simulator, then run the launch file:

```
roslaunch mav_local_planner firefly_sim.launch

// Due to an oversight by the mav_voxblox_planning developers, you need to publish the 
// following TF in order to receive the ESDF map when using this package.
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 firefly/vi_sensor/camera_depth_optical_center_link camera_depth_optical_center_link

roslaunch scoutair_planner frontier.launch
```


<p align="center">
<img src="./doc/sim_world.png" alt="Simulation World" width="600" height="500">
</p>


<p align="center">
<img src="./doc/frontier_tour.png" alt="Frontier Tour" width="600" height="500">
</p>

## Parameters

You can change the parameters in `./launch/frontier.launch`.



<p align="center">
<img src="./doc/explorated_map.png" alt="Explorated Map" width="600" height="500">
</p>


   
