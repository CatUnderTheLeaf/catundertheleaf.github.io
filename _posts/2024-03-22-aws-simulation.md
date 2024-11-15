---
layout: post
title: "AWS DeepRacer simulation: Setup"
subtitle: "Setup"
categories: aws
---
## Roadmap
{% assign posts = site.categories["aws"] | sort %}
<ul>
    {% for post in posts %}
      {% if post.subtitle==page.subtitle%}
      {% assign next_post = post.next %}
         <li>{{ post.subtitle }}
            <ul>
               <li><a href="#introduction">Introduction</a></li>
               <li><a href="#finished-simulation">Finished simulation</a></li>
               <li><a href="#software-setup">Software setup</a></li>
               <li><a href="#installation">Installation</a></li>
            </ul>
         </li>
      {% else %}
         <li><a href="{{ post.url }}">{{ post.subtitle }}</a></li>
      {% endif %}
    {% endfor %}
</ul>

## Introduction

A lot of people have heard of AWS DeepRacer, but if not, here is short information from their site:
> [AWS DeepRacer](https://aws.amazon.com/deepracer/?nc=sn&loc=0) gives you an interesting and fun way to get started with reinforcement learning (RL). RL is an advanced machine learning (ML) technique that takes a very different approach to training models than other machine learning methods. Its super power is that it learns very complex behaviors without requiring any labeled training data, and can make short term decisions while optimizing for a longer term goal.

> Developers of all skill levels can get hands on with machine learning through a cloud-based 3D racing simulator, fully autonomous 1/18th scale race car driven by reinforcement learning, and global racing league.

But... only 10 hours free for 30 days with the AWS Free Tier... Maybe there is something that can be launched locally?

> [deepracer-for-cloud](https://aws-deepracer-community.github.io/deepracer-for-cloud/) provides a quick and easy way to get up and running with a DeepRacer training environment in AWS or Azure, using either the Azure N-Series Virtual Machines or AWS EC2 Accelerated Computing instances, or locally on your own desktop or server.

That is a good option: train your car locally for any number of hours and then just upload the model.

But what if you want to gain more control of your car? Or you want to somehow incorporate `Behavioral cloning` and train your car with manual steering and velocity inputs in hope that it will have a better performance than with random inputs of RL? Or you just want to simply simulate a car, race track and play with it?

## Finished simulation

If you want just to run the simulation you can clone my repository and run it with Docker.
{% highlight shell %}
# Download this repository
git clone https://github.com/CatUnderTheLeaf/deepRacerSim.git

# change dir to this repository
cd deepRacerSim

# Build docker image with
docker build -f deepRacerSim.Dockerfile -t deep-simulator .

# change directory
cd deepRacerSim/deep_ws

# launch a Docker container
./launch_docker.sh

# launch simulation
roslaunch simulation simulation.launch
{% endhighlight %}

To control a car in launch another terminal
{% highlight shell %}
# change directory
cd deepRacerSim/deep_ws

# launch a Docker container
./launch_docker.sh

# launch keyboard teleoperation
roslaunch teleop_ackermann key_teleop.launch

# or launch joy teleoperation
roslaunch teleop_ackermann joy_teleop.launch
{% endhighlight %}

If you don't want to use Docker, then read instructions on [GitHub](https://github.com/CatUnderTheLeaf/deepRacerSim).

But if you want to make it from scratch - continue reading. 

## Software setup

It turned out there are three possible software configurations:

- __DeepRacer__: Ubuntu 16.04, ROS1 Kinetic(EOL) and Python 2(EOL)
- __DeepRacer with update__(see note): Ubuntu 20.04, ROS2 Foxy Fitzroy(EOL) and Python 3.8
- __deepracer-for-cloud__: Ubuntu 20.04, ROS1 Noetic and Python 3.8

As can be seen the last configuration is the right option to simulate and train a car locally. Noetic is the last version of ROS1 and Python 3.8 will meet its EOL in October 2024.

> Note from [AWS docs](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-ubuntu-update.html): "Update is required to run AWS DeepRacer open-source projects but is otherwise optional. AWS DeepRacer only supports Ubuntu 20.04 Focal Fossa and ROS2 Foxy Fitzroy."

> The reasons why they chose to update to ROS2 Foxy Fitzroy which has reached its EOL are unclear. There are still ROS1 Noetic (EOL May, 2025) and stable and supported distros of ROS2 (Humble and Iron).

> The ROS version should not affect training and using RL model as it has only four velocity and two steering values as inputs. And you can pass it using whichever ROS version you want.

## Installation

- Windows users
   - You will need to be on Windows 11 Build 22000 or later.
   - Install driver for vGPU to run Linux GUI apps
      * [Intel GPU driver for WSL](https://www.intel.com/content/www/us/en/download/19344/intel-graphics-windows-10-windows-11-dch-drivers.html)
      * [AMD GPU driver for WSL](https://www.amd.com/en/support/kb/release-notes/rn-rad-win-wsl-support)
      * [NVIDIA GPU driver for WSL](https://developer.nvidia.com/cuda/wsl)
   - Install [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install). Open `Microsoft Store` and install Ubuntu 20.04.
   - Run Ubuntu 20.04
- Install [ROS1 Noetic](http://wiki.ros.org/noetic/Installation)
- Update/Install Git and Python3
{% highlight shell %}
# Install Git and Python3 if not installed
apt-get update && apt-get install -y git python3-pip
{% endhighlight %}
- Install [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install) for simulation
- Install [gazebo-ros-pkgs and gazebo-ros-control](https://classic.gazebosim.org/tutorials?tut=ros_installing)
- Source ROS1
{% highlight shell %}
# or just add it to ~/.bashrc with
# echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source /opt/ros/noetic/setup.bash
{% endhighlight %}

<a href="{{next_post.url | escape}}">Next: {{ next_post.subtitle }}</a>