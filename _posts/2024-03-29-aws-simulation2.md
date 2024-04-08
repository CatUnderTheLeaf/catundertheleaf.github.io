---
layout: post
title: "AWS DeepRacer simulation. Part 2 - Launch race track in Gazebo"
subtitle: "Part 2 - Launch race track in Gazebo"
categories: aws
---

## Quick Overview of ROS1 concepts

- __Packages__: Are the software organization unit of ROS code. Each package can contain libraries, executables, scripts, or other artifacts.
- __Nodes__: A node is an executable that uses ROS to communicate with other nodes.
- __Messages__: ROS data type used when subscribing or publishing to a topic.
- __Topics__: Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.
- __Master__: Name service for ROS (i.e. helps nodes find each other)
- __catkin__: A method for organizing and building your ROS code
- __Workspace__: A working directory where all packages are stored and build with catkin

## Launch Gazebo using ROS

1. The very first step is to create a catkin workspace if it not already exists.
```
$ mkdir -p ~/deep_ws/src
$ cd ~/deep_ws/
$ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```
2. Create a package with name `simulation`. Here will be gathered all code for launching Gazebo and loading race track worlds.
```
$ cd ~/deep_ws/src

# create package with name ‘simulation’ and dependency on ‘gazebo_ros’
$ catkin_create_pkg simulation gazebo_ros
$ cd ~/deep_ws

$ catkin_make
```
> After every `catkin_make` be sure to source your workspace!!!
> ```
> $ . ~/deep_ws/devel/setup.bash
> ```
3. Gazebo in ROS has 2 nodes: `gzserver` and `gzclient`. We need them both, but they can't be launched without ROS MASTER.
In order to launch it you should:
   - run roscore(ROS MASTER), gzserver and gzclient in three separate terminals
   - or make a `launch` file
4. I prefer `launch` files. They make running ROS code so much easier
