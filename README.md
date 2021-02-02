# LEICA SCANSTATION C5 #

[![License](https://img.shields.io/badge/License-Apache%202-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build Status](https://travis-ci.com/fada-catec/leica_gazebo_simulation.svg?branch=master)](https://travis-ci.com/fada-catec/leica_gazebo_simulation)

This repo contains source code to develop a simulator in Gazebo for Leica Scanstation C5.
It is involved in the ROSIN project [Large_3D_inspection](http://wiki.ros.org/large_3d_inspection)

![simulator](simulator.gif)

## Set up ##

* Clone

        mkdir -p ~/catkin_ws/src
        cd ~/catkin_ws/src
        git clone https://github.com/fada-catec/leica_gazebo_simulation.git

* Install dependencies 

* Compile

        cd ~/catkin_ws
        catkin_make
        source devel/setup.bash

## Usage ##

Generate default world and place the device at your convenience.

    roslaunch leica_gazebo_simulation c5_system_spawn.launch world:=assembly_line

The plugin created for LeicaScanstation C5 on Gazebo will publish LaserScan msg. Use laserscan_to_pointcloud to convert it to PointCloud2. Run the plugin to start movement and complete a scan on Gazebo.

    roslaunch leica_gazebo_simulation c5_init.launch namespace:=c5

Tell simulator to start scan with desired [window parameters](http://wiki.ros.org/leica_scanstation#Scan_with_Leica_Scanstation_C5). Point cloud is published simultaneously. Once scan is finished, it will store the complete point cloud with given file_name on folder specified in ROS param server as `/pointcloud_folder` (default: package leica_scanstation_utils/pointclouds).

    rosservice call /c5/scan "{file_name: 'scan_sim', vertical_res: 512, horizontal_res: 1024, pan_center: 0.0, tilt_center: 0.0, width: 2, height: 2}"

## Dependencies ##

* ROS [Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) // ROS [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) 

* PCL:

    1. Boost, Flann, Eigen3

            sudo apt-get install libboost1.58* libflann1.8 libeigen3-dev

    2. PCL-ROS y PCL-CONVERSIONS

            sudo apt-get install ros-$ROS_DISTRO-pcl-*

* Leica packages:

    1. [leica_scanstation_msgs](https://github.com/fada-catec/leica_scanstation/tree/master/leica_scanstation_msgs)

    2. [leica_scanstation_utils](https://github.com/fada-catec/leica_scanstation/tree/master/leica_scanstation_utils)

## Doc ##

        cd ~/catkin_ws/src/leica_gazebo_simulation
        rosdoc_lite .
        google-chrome doc/html/index.html

## Acknowledgement

***
<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287. 

## Help ##
Ines M. Lara - imlara@catec.aero
Other community or team contact
