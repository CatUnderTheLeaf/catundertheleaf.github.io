---
layout: post
title: "AWS DeepRacer simulation. Part 3 - Car model"
subtitle: "Part 3 - Car model"
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
               <li><a href="#create-a-car-model">Create a car model</a></li>
               <li><a href="#load-car-model-to-gazebo-simulation">Load car model to Gazebo simulation</a></li>
            </ul>
         </li>
      {% else %}
         <li><a href="{{ post.url }}">{{ post.subtitle }}</a></li>
      {% endif %}
    {% endfor %}
</ul>

## Create a car model

Car (robot) models are written in [URDF](https://wiki.ros.org/urdf/XML) format, it means it has:
- links - building block of a model. Each link has its own position, orientation, inertia, visual features, and collision properties
- joints connect links. Each joint has its own position, axis and type(fixed, continuous, etc.)

> More about URDF can be found [here](https://wiki.ros.org/urdf/Tutorials)

I took car model from [aws-deepracer](https://github.com/aws-deepracer/aws-deepracer/tree/main/deepracer_description) ROS2 version and rewrote some parts to work in ROS1.

Create a package with name `deepracer_car`. Here we will gather all code for car model and its control.

{% highlight shell %}
$ cd ~/deep_ws/src
# create package with name deepracer_car and dependencies
$ catkin_create_pkg deepracer_car gazebo_ros ackermann_msgs std_msgs rospy
$ cd ../
$ catkin_make
{% endhighlight %}

> After every `catkin_make` be sure to source your workspace!!!

Copy in `deepracer_car` folders `meshes`, `urdf` and `rviz` from my [repo](https://github.com/CatUnderTheLeaf/deepRacerSim/tree/main/deep_ws/src/deepracer_car).

To view this model we will use RVIZ - ROS 3D Robot Visualizer. Create `rviz.launch` in `deepracer_car/launch` folder and paste there:
{% highlight xml %}
<!-- deep_ws/src/deepracer_car/launch/rviz.launch -->
<?xml version="1.0"?>
<launch>
<!-- load car model to parameter server -->
<!-- xacro parses your macros and constants;
      e.g. you can write one macros for a wheel
      and call it for times instead of 
      four almost identical pieces of code -->
  <param name="robot_description" 
   command="$(find xacro)/xacro '$(find deepracer_car)/urdf/xacro/deepracer/deepracer.xacro'"/>

<!-- A source that publishes car joint positions as a sensor_msgs/JointState -->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />

<!-- robot state publisher internally has 
      a kinematic model of the robot; 
      so given the joint positions of the robot, 
      the robot state publisher can compute and 
      broadcast the 3D pose of each link in the robot. -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" 
        type="robot_state_publisher" respawn="false" output="screen"/>

<!-- load rviz with configuration file -->
  <arg name="rvizconfig" default="$(find deepracer_car)/rviz/rviz.rviz" />
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)"/>
</launch>
{% endhighlight %}

Now run 
{% highlight shell %}
$ roslaunch deepracer_car rviz.launch
{% endhighlight %}
and you will see a car model without visual meshes
![car model](/assets/rviz_model.png)

## Load car model to Gazebo simulation

To properly load a model to Gazebo make sure to have GAZEBO references in your `urdf/xacro` folders:
- `collision` and `macro` - wheel physical attributes and collision
- `material` - which Gazebo material to use
- `sensor` - camera sensor (h, w, fov) and camera plugin (update rate, topics, frame, distortion) to publish camera images from Gazebo simulation to ROS topic

Create `racecar.launch` in `deepracer_car/launch` folder and paste there:
{% highlight xml %}
<!-- deep_ws/src/deepracer_car/launch/racecar.launch -->
<?xml version="1.0"?>
<launch>

<!-- load car model to parameter server -->
  <param name="robot_description" command="$(find xacro)/xacro '$(find deepracer_car)/urdf/xacro/deepracer/deepracer.xacro'"/>

  <!-- push robot_description to factory and spawn robot in gazebo -->
  <node name="racecar_spawn" pkg="gazebo_ros" type="spawn_model" output="screen" args="-urdf -param /robot_description -model deepracer -x 0.46 -y -0.36 -z 0.03 -Y -0.088633" />
 
</launch>
{% endhighlight %}

And include this file in `gazebo.launch`:
{% highlight xml %}
<!-- deep_ws/src/simulation/launch/gazebo.launch -->
<?xml version="1.0"?>
<include file="$(find deepracer_car)/launch/racecar.launch"/>
{% endhighlight %}

Now run
{% highlight shell %}
$ roslaunch simulation simulation.launch
{% endhighlight %}
and you will see a car on a race track, now we have a car model and need to somehow control it. 
![car model in world](/assets/car_track.png)

<a href="{{next_post.url | escape}}">Next: {{ next_post.subtitle }}</a>