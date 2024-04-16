---
layout: post
title: "AWS DeepRacer simulation. Part 5 - Teleoperation"
subtitle: "Part 5 - Teleoperation"
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
               <li><a href="#teleoperation">Teleoperation</a></li>
               <li><a href="#keyboard-teleoperation">Keyboard teleoperation</a></li>
               <li><a href="#joy-teleoperation">Joy teleoperation</a></li>
            </ul>
         </li>
      {% else %}
         <li><a href="{{ post.url }}">{{ post.subtitle }}</a></li>
      {% endif %}
    {% endfor %}
</ul>

## Teleoperation

It is much more convenient to control a car with a joy or a keyboard. There are official ROS1 teleoperation packages [teleop_twist_keyboard](https://wiki.ros.org/teleop_twist_keyboard) and [teleop_twist_joy](https://wiki.ros.org/teleop_twist_joy). But they convert keyboard/joy input into `Twist` message, and we need it to be an `AckermannDriveStamped` message. I have found [ackermann-drive-teleop](https://github.com/gkouros/ackermann-drive-teleop) and it is pretty good, but:
- you can't change increment/decrement step
- publish rate is fixed
- despite publishing an `AckermannDriveStamped` message, it is published without stamp and frame_id
- and params are set through arguments not parameter server

So, you guess it right, now we will write an improved teleoperation package for robots with ackermann steering.

Create new package in `deep_ws/src` folder
{% highlight shell %}
$ cd ~/deep_ws/src
# create package with name 'teleop_ackermann' and its dependencies
$ catkin_create_pkg teleop_ackermann rospy ackermann_msgs joy
$ cd ../
$ catkin_make
{% endhighlight %}

> After `catkin_make` be sure to source your workspace!!!

## Keyboard teleoperation

If you don't have a joy, than its your option. Create a `key_op.py` in a `script` folder
{% highlight python %}
# deep_ws/src/teleop_ackermann/scripts/key_op.py
#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped

import sys, select, termios, tty

# key bindings
keys = {
    'up'    : '\x41',
    'down'  : '\x42',
    'right' : '\x43',
    'left'  : '\x44',
    'space' : '\x20',
    'tab'   : '\x09'}

class KeyTeleopAckermann():

    def __init__(self):

        # get and set params
        self.max_speed = rospy.get_param("~max_speed", 4.0)
        self.max_steering_angle = rospy.get_param("~max_steering_angle", 0.523599)
        self.scale_speed = rospy.get_param("~scale_speed", 10)
        self.scale_angle = rospy.get_param("~scale_angle", 10)
        self.update_rate = rospy.get_param("~update_rate", 50) # Hz

        # set custom increment/decrement step
        self.key_ops = {
            '\x41' : ( self.max_speed/self.scale_speed, 0.0),
            '\x42' : (-self.max_speed/self.scale_speed , 0.0),
            '\x43' : ( 0.0 ,-self.max_steering_angle/self.scale_angle),
            '\x44' : ( 0.0 , self.max_steering_angle/self.scale_angle)}

        self.speed = 0
        self.steering_angle = 0

        self.ack_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)
        
        self.print_info()

        # publish ackermann_msg with a custom rate
        rospy.Timer(rospy.Duration(1.0/self.update_rate), self.publish_message)

        # control key input        
        self.control()

if __name__ == '__main__':
    try:
        rospy.init_node('key_teleop_ackermann_drive', anonymous=True, log_level=rospy.INFO)
        node = KeyTeleopAckermann()
    except KeyboardInterrupt:
        print("Shutting down ROS key_teleop_ackermann_drive node")
{% endhighlight %}

at the start user is greeted with text information about key-bindings and current input state
{% highlight python %}
    def print_info(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\rUse up/down arrows to change speed')
        rospy.loginfo('\x1b[1M\rUse left/right arrows to change steering angle')
        rospy.loginfo('\x1b[1M\rUse space to brake and tab to align wheels')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\r'
                      '\033[34;1mSpeed: \033[32;1m%0.2f m/s, '
                      '\033[34;1mSteering Angle: \033[32;1m%0.2f rad\033[0m',
                      self.speed, self.steering_angle)
{% endhighlight %}

with a chosen rate `rospy.Timer` will publish `AckermannDriveStamped` message
{% highlight python %}
    def publish_message(self, event):
        # publish message with stamp and frame_id
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/base_link'
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering_angle
        self.ack_pub.publish(msg)\
{% endhighlight %}

Control loop is running until node shutdown, reads inputs from terminal window, clips inputs so they are in a min/max range and prints updated info
{% highlight python %}
    def control(self):        
        self.settings = termios.tcgetattr(sys.stdin)
        while not rospy.is_shutdown():
            key = self.read_key()
            if key in keys.values():
                if key == keys['space']:
                    self.speed = 0.0
                elif key == keys['tab']:
                    self.steering_angle = 0.0
                else:
                    a_speed, a_angle = self.key_ops[key]
                    self.speed = self.speed + a_speed
                    self.steering_angle = self.steering_angle + a_angle
                    # clip in between min and max values
                    self.speed = max(min(self.max_speed, self.speed), 0)                        
                    self.steering_angle = max(min(self.max_steering_angle, self.steering_angle), -self.max_steering_angle)
                    if self.speed==0:
                        self.steering_angle=0
                self.print_info()            
            elif key == '\x03':  # ctr-c
                break
            else:
                continue

        # publish last zero commands
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 0
        self.steering_angle = 0
        self.publish_message(self.speed)
        sys.exit()

    def read_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
{% endhighlight %}

Create a `key_teleop.launch` in `launch` folder
{% highlight xml %}
<!-- deep_ws/src/teleop_ackermann/launch/key_teleop.launch -->
<?xml version="1.0"?>
<launch>

  <arg name="max_speed" default="4.0"/>
  <arg name="max_steering_angle" default="0.523599"/>
  <arg name="scale_speed" default="10"/>
  <arg name="scale_angle" default="10"/>
  <arg name="update_rate" default="50"/>
   
  <node name="teleop" pkg="teleop_ackermann" type="key_op.py" output="screen">
    <param name="max_speed" value="$(arg max_speed)"/>
    <param name="max_steering_angle" value="$(arg max_steering_angle)"/>
    <param name="scale_speed" value="$(arg scale_speed)"/>
    <param name="scale_angle" value="$(arg scale_angle)"/>
    <param name="update_rate" value="$(arg update_rate)"/>
  </node>

</launch>
{% endhighlight %}

Now you can launch simulation and teleoperation
{% highlight shell %}
$ cd ~/deep_ws/src
$ . ~/deep_ws/devel/setup.bash
$ roslaunch simulation simulation.launch

# in another terminal window
$ cd ~/deep_ws/src
$ . ~/deep_ws/devel/setup.bash
$ roslaunch teleop_ackermann key_teleop.launch
{% endhighlight %}

![teleoperation](/assets/teleoperation.png)

## Joy teleoperation

With joy it is easier to control a car than with a keyboard. Create a `joy_op.py` in a `script` folder
{% highlight python %}
# deep_ws/src/teleop_ackermann/scripts/joy_op.py
#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy

import sys, termios

class JoyTeleopAckermann():

    def __init__(self):

        # get and set params and joy button bindings
        self.max_speed = rospy.get_param("~max_speed", 4.0)
        self.max_steering_angle = rospy.get_param("~max_steering_angle", 0.523599)
        self.steering_axis = rospy.get_param("axis_steering", 0)
        self.speed_axis = rospy.get_param("axis_speed", 4)
        self.stop_button = rospy.get_param("stop_button", 4)
        self.align_button = rospy.get_param("align_button", 6)

        self.update_rate = rospy.get_param("~update_rate", 50) # Hz
        self.speed = 0
        self.steering_angle = 0

        self.print_info()

        # subscribe to a topic which publishes which buttons/sticks on a joy where pressed or moved
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)

        self.ack_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)

        # publish ackermann_msg with a rate
        rospy.Timer(rospy.Duration(1.0/self.update_rate), self.publish_message)


if __name__ == '__main__':
    try:
        rospy.init_node('joy_teleop_ackermann_drive', anonymous=True, log_level=rospy.INFO)
        node = JoyTeleopAckermann()
    except KeyboardInterrupt:
        print("Shutting down ROS joy_teleop_ackermann_drive node")
{% endhighlight %}

Print text informatio
{% highlight python %}
    def print_info(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r*********************************************')
        rospy.loginfo('\x1b[1M\rUse up/down stick to change speed')
        rospy.loginfo('\x1b[1M\rUse left/right stick to change steering angle')
        rospy.loginfo('\x1b[1M\rUse L1 button to brake and L2 button to align wheels')
        rospy.loginfo('\x1b[1M\r*********************************************')
{% endhighlight %}

Update stearing angle and speed from joy or stop control on exit
{% highlight python %}
    def joy_callback(self, data):
        """Callback on each change in joy message

        Args:
            data (sensor_msgs.Joy): joy message

        """    
        if (data.buttons[self.stop_button]==1):
            self.stop()
        elif(data.buttons[self.align_button]==1):
            self.steering_angle = 0.0
        else:
            self.speed = max(round(data.axes[self.speed_axis], 2) * self.max_speed, 0.0)
            self.steering_angle = round(data.axes[self.steering_axis], 2) * self.max_steering_angle 
            if self.speed==0:
                self.steering_angle=0

    def stop(self):
        # publish last zero commands
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 0
        self.steering_angle = 0
        self.publish_message(self.speed)
        sys.exit()
{% endhighlight %}

Publish an `AckermannDriveStamped` message with a custom rate
{% highlight python %}
    def publish_message(self, event):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/base_link'
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering_angle
        self.ack_pub.publish(msg)
        rospy.loginfo('\x1b[1M\r'
                '\033[34;1mSpeed: \033[32;1m%0.2f m/s, '
                '\033[34;1mSteering Angle: \033[32;1m%0.2f rad\033[0m',
                self.speed, self.steering_angle)
{% endhighlight %}

Create a `joy.yaml` config file to put there joy bindings
{% highlight xml %}
<!-- deep_ws/src/teleop_ackermann/config/joy.yaml -->
axis_speed: 4
axis_steering: 0
stop_button: 4
align_button: 6
{% endhighlight %}

Install ROS nodes and drivers for a joystick
{% highlight shell %}
$ sudo apt-get install ros-noetic-joy
{% endhighlight %}

> In order to use a joystick, it must have read and write permissions.
> You can grant such permissions by executing the following command:
{% highlight shell %}
$ sudo chmod a+rw /dev/input/js0
{% endhighlight %}

Create a `joy_teleop.launch` in `launch` folder
{% highlight xml %}
<!-- deep_ws/src/teleop_ackermann/launch/joy_teleop.launch -->
<?xml version="1.0"?>
<launch>
 
 <!-- joy args -->
  <arg name="joy_dev" default="/dev/input/js0" />
  <arg name="config_filepath" default="$(find teleop_ackermann)/config/joy.yaml" />
  <arg name="joy_topic" default="joy" />

<!-- load node for publishing joy messages -->
  <node pkg="joy" type="joy_node" name="joy_node">
    <param name="dev" value="$(arg joy_dev)" />
    <param name="deadzone" value="0.3" />
    <param name="autorepeat_rate" value="20" />
    <remap from="joy" to="$(arg joy_topic)" />
  </node>
  
  <!-- load params of the joy -->
  <rosparam command="load" file="$(arg config_filepath)" />

  <!-- additional node args -->
  <arg name="max_speed" default="4.0"/>
  <arg name="max_steering_angle" default="0.523599"/>
  <arg name="update_rate" default="50"/>
   
  <node name="teleop" pkg="teleop_ackermann" type="joy_op.py" output="screen">
    <param name="max_speed" value="$(arg max_speed)"/>
    <param name="max_steering_angle" value="$(arg max_steering_angle)"/>
    <param name="update_rate" value="$(arg update_rate)"/>
    <remap from="joy" to="$(arg joy_topic)" />
  </node>

</launch>
{% endhighlight %}

Now you can launch simulation and teleoperation
{% highlight shell %}
# in every terminal window source your workspace
$ cd ~/deep_ws/src
$ . ~/deep_ws/devel/setup.bash
$ roslaunch simulation simulation.launch

# in another terminal window
$ cd ~/deep_ws/src
$ . ~/deep_ws/devel/setup.bash
$ roslaunch teleop_ackermann joy_teleop.launch
{% endhighlight %}