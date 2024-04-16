---
layout: post
title: "AWS DeepRacer simulation. Part 2 - Launch race track in Gazebo"
subtitle: "Part 2 - Launch race track in Gazebo"
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
               <li><a href="#quick-overview-of-ros1-concepts">Quick Overview of ROS1 concepts</a></li>
               <li><a href="#launch-gazebo-using-ros">Launch Gazebo using ROS</a></li>
               <li><a href="#load-a-race-track-world">Load a race track world</a></li>
            </ul>
         </li>
      {% else %}
         <li><a href="{{ post.url }}">{{ post.subtitle }}</a></li>
      {% endif %}
    {% endfor %}
</ul>

## Quick Overview of ROS1 concepts

- __Packages__: Are the software organization unit of ROS code. Each package can contain libraries, executables, scripts, or other artifacts.
- __Nodes__: A node is an executable that uses ROS to communicate with other nodes.
- __Messages__: ROS data type used when subscribing or publishing to a topic.
- __Topics__: Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.
- __Master__: Name service for ROS (i.e. helps nodes find each other)
- __catkin__: A method for organizing and building your ROS code
- __Workspace__: A working directory where all packages are stored and build with catkin

## Launch Gazebo using ROS

The very first step is to create a catkin workspace if it not already exists.

{% highlight shell %}
$ mkdir -p ~/deep_ws/src
$ cd ~/deep_ws/
$ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
{% endhighlight %}

Create a package with name `simulation`. Here we will gather all code for launching Gazebo and loading race track worlds.

{% highlight shell %}
$ cd ~/deep_ws/src
# create package with name 'simulation' and dependency on 'gazebo_ros'
$ catkin_create_pkg simulation gazebo_ros
$ cd ../
$ catkin_make
{% endhighlight %}

> After every `catkin_make` be sure to source your workspace!!!
{% highlight shell %}
$ . ~/deep_ws/devel/setup.bash
{% endhighlight %}

Gazebo in ROS has 2 nodes: `gzserver` and `gzclient`. As you remember in ROS to launch a node(script) you should first launch ROS MASTER. In order to launch both Gazebo nodes you should:
   - run roscore(ROS MASTER), gzserver and gzclient in three separate terminals
   - or make a `launch` file
I prefer `launch` files. They make running ROS code so much easier. Go to `deep_ws/src/simulation` folder and make `launch` directory, in which create a file `gazebo.launch`. This example code launches two Gazebo nodes with the `world` argument, which now leads to empty world:

{% highlight xml %}
<!-- deep_ws/src/simulation/launch/gazebo.launch -->
<?xml version="1.0"?>
<launch>
  <arg name="world_name" default="worlds/empty.world"/>
  <node name="gazebo" pkg="gazebo_ros" type="gzserver" respawn="false" output="screen" args="$(arg world_name) "/>
  <node name="gazebo_gui" pkg="gazebo_ros" type="gzclient" respawn="false" output="screen" />
</launch>
{% endhighlight %}
> `world` - is how our simulation looks, all objects, lights, shadows, etc. Empty world has nothing - only ground plane, 3-d axes and a source of light.

Now we can launch empty world in Gazebo

{% highlight shell %}
$ cd ~/deep_ws
$ source devel/setup.bash
$ roslaunch simulation gazebo.launch
{% endhighlight %}

![empty world](/assets/empty_world.png)

That was not so difficult. Now lets load a race track world. 

##  Load a race track world

For this task you need folders `meshes`, `models`, `worlds` and `routes` from [deepracer_simapp](https://github.com/aws-deepracer-community/deepracer-simapp/tree/master/bundle). Download them all to `deep_ws/src/simulation` and create a `simulation.launch` file in `deep_ws/src/simulation/launch` folder. This launch file will include previous `gazebo.launch` and will pass a `world` argument.

{% highlight xml %}
<!-- deep_ws/src/simulation/launch/simulation.launch -->
<?xml version="1.0"?>
<launch>
  <arg name="world_name" default="$(find simulation)/worlds/2022_march_pro.world"/>
  <include file="$(find simulation)/launch/gazebo.launch">
    <arg name="world_name" value="$(arg world_name)"/>
  </include>
</launch>
{% endhighlight %}

Similarly run it from `deep_ws` folder with 
{% highlight shell %}
$ roslaunch simulation simulation.launch
{% endhighlight %}

But.... wait. There is a warning:

{% highlight shell %}
[Wrn] [ModelDatabase.cc:340] Getting models from[http://models.gazebosim.org/]. This may take a few seconds.
{% endhighlight %}

Ok, it takes more than a few seconds and it seems to look for models in internet not in our simulation folder. 
__Important:__ To avoid this you need to:
{% highlight shell %}
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/deep_ws/src/simulation/
$ export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/deep_ws/src/simulation/
{% endhighlight %}

> Yes, there will be another warning about folder not being a model folder etc., but it can be ignored

After path exports `simulation.launch` will launch a race track world. To launch other world - just change its name in file or pass it as argument in shell.

![race_track](/assets/race_track.png)

<a href="{{next_post.url | escape}}">Next: {{ next_post.subtitle }}</a>