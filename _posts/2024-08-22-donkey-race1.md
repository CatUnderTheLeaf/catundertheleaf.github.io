---
layout: post
title: "rosRoboCar: Installation"
subtitle: "Installation"
categories: donkey
---
## Roadmap
{% assign posts = site.categories["donkey"] | sort %}
<ul>
    {% for post in posts %}
      {% if post.subtitle==page.subtitle%}
      {% assign next_post = post.next %}
         <li>{{ post.subtitle }}
            <ul>
               <li><a href="#introduction">Introduction</a></li>
               <li><a href="#software-setup">Software setup</a></li>
               <li><a href="#installation">Installation</a>
                  <ul>
                     <li><a href="#on-pc">On PC</a></li>
                     <li><a href="#on-a-car">On a car</a></li>
                  </ul>
               </li>
               <li><a href="#running-ros-across-multiple-machines">Running ROS across multiple machines</a></li>

            </ul>
         </li>
      {% else %}
         <li><a href="{{ post.url }}">{{ post.subtitle }}</a></li>
      {% endif %}
    {% endfor %}
</ul>

## Introduction

In Berlin we had a meetup group [Autonomous Robots Berlin](https://www.meetup.com/autonomous-robots-berlin/). Now it is not active, but earlier this group held a competition. The rules were simple:
1. Every car must make at least 3 loops in a row without human intervention.
2. Fastest of these loops counts (so you can gradually increase speed on the track;).
3. There are two paths available with the same starting point: a long loop for lane holding and an obstacle avoidance loop. The obstacle avoidance loop is shorter than just lane holding, but for each cone hit 2 seconds are added to your total score. Each participant is free to choose which loop to take.
4. The vehicle can leave the track if it comes back and it did not take a shortcut.

The race took place at [The Drivery's](https://maps.app.goo.gl/U5LchxJhbCm45UDn9) Gravity Gym track

## Software setup

I was given a [DonkeyCar](https://docs.donkeycar.com/) which had only:
- 1GB RAM RaspberryPi
- two __motors__, which recieved only PWM signals
  - throttle
  - steering
- RaspberryPi __camera__

It is a very basic setup, because no sensors for obstacle detection, or just for odometry to get some localization.

Robocar uses ROS1 Noetic. For testing and training purposes it can be run across multiple machines, when a car sends camera images and on computer I can visualize and work with them.

## Installation

### On PC
- Windows users
   - You will need to be on Windows 11 Build 22000 or later.
   - Install driver for vGPU to run Linux GUI apps
      * [Intel GPU driver for WSL](https://www.intel.com/content/www/us/en/download/19344/intel-graphics-windows-10-windows-11-dch-drivers.html)
      * [AMD GPU driver for WSL](https://www.amd.com/en/support/kb/release-notes/rn-rad-win-wsl-support)
      * [NVIDIA GPU driver for WSL](https://developer.nvidia.com/cuda/wsl)
   - Install [WSL2](https://learn.microsoft.com/en-us/windows/wsl/install). Open `Microsoft Store` and install Ubuntu 20.04.
   - Download and install [VcXsrv Windows X Server](https://sourceforge.net/projects/vcxsrv/). Open and set:
      - check multiple windows and set display number to 0;
      - in next window check start no client;
      - finally check all extra settings;
      - enable Outgoing Connection from Windows Firewall
   - Run Ubuntu 20.04
   - In WSL run `export DISPLAY=127.0.0.1:0.0 #  you can also add it to ~/.bashrc`
   - Create a .xsession file in the user home directory e.g. `echo xfce4-session > ~/.xsession`
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
- Download and build my repository
{% highlight shell %}
git clone --recurse-submodules https://github.com/CatUnderTheLeaf/rosRoboCar.git

source /opt/ros/noetic/setup.bash
cd /path/to/robocar_ws

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro noetic

# Building packages
# Add CATKIN_IGNORE to robot_only packages, e.g. raspicam_node
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

# Source this workspace
source devel/setup.bash
{% endhighlight %}

### On a car
- A DonkeyCar has RaspberryPi, so first make it [work](https://docs.donkeycar.com/guide/robot_sbc/setup_raspberry_pi/). If there are problems check [resolve connectivity problems](https://github.com/CatUnderTheLeaf/rosRoboCar/wiki/Connectivity-problem)
- Install ROS1 Noetic from [sources](http://wiki.ros.org/noetic/Installation/Source). In the `catkin_make_isolated` step add `-j2` flag, so it will not stuck.
- Download this repository.
- Unfortunately dependencies can not be resolved, as there are no binary packages. So download in `~/ros_catkin_ws/src` following repositories: [image_transport](https://github.com/ros-perception/image_transport_plugins.git), replace [image_common](https://github.com/ros-perception/image_common.git), [image_pipeline](https://github.com/ros-perception/image_pipeline.git), replace [vision_opencv](https://github.com/ros-perception/vision_opencv.git).
- Install `compressed_image_transport`, `image_transport_plugins`, `camera_calibration_parsers`, `camera_info_manager`, `image_geometry`, `image_proc` with a command
{% highlight shell %}
./src/catkin/bin/catkin_make_isolated -j2 --install -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 --install-space /opt/ros/noetic --pkg YOUR_PACKAGE_NAME
{% endhighlight %}
- Install libs for i2cpwm_board node:
   - install `libi2c-dev`
   - add `target_link_libraries(i2cpwm_board i2c)` to `CMakeLists.txt`
   - add to `i2cpwm_controller.cpp`
{% highlight c++ %}
extern "C" {
    #include <unistd.h>
    #include <fcntl.h>
    #include <sys/ioctl.h>
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h> // additional header file
}
{% endhighlight %}   
   > Code of this package was copied from [this](https://github.com/tizianofiorenzani/ros_tutorials.git) tutorial, because original was deleted(?). That tutorial was made when `libi2c-dev` was version 3. Now it is version 4 and some big changes were made.

## Running ROS across multiple machines

[Connect](http://wiki.ros.org/ROS/Tutorials/MultipleMachines) your car and PC. Don't forget to export `ROS_IP` and `ROS_MASTER_URI`

Now a car can be launched with
{% highlight shell %}
roslaunch donkeycar donkey.launch simulation:=0
{% endhighlight %}

On PC:
{% highlight shell %}
 rosrun image_view image_view image:=//raspicam/image _image_transport:=compressed
{% endhighlight %}

<a href="{{next_post.url | escape}}">Next: {{ next_post.subtitle }}</a>