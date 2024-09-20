---
layout: post
title: "rosRoboCar: Calibration and Setup"
subtitle: "Calibration and Setup"
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
               <li><a href="#camera-calibration">Camera calibration</a></li>
               <li><a href="#steering-a-throttle-calibration">Steering a throttle calibration</a></li>
               <li><a href="#add-teleoperation">Add teleoperation</a></li>
               <li><a href="#record-bag-files">Record bag files</a></li>

            </ul>
         </li>
      {% else %}
         <li><a href="{{ post.url }}">{{ post.subtitle }}</a></li>
      {% endif %}
    {% endfor %}
</ul>

## Camera calibration

- Print a [checkboard](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration?action=AttachFile&do=view&target=check-108.pdf).

- Make 10-15 camera images in different angles, distance from camera and position in the image.

- Run a simple [code](https://github.com/CatUnderTheLeaf/rosRoboCar/blob/main/additional_files/calibrate.py).

![checkboards](/assets/rosRoboCar/checkboards.png)

- Camera matrix, projection matrix and distortion coefficients should be printed in terminal.

- Now the image can be undistorted.

![Undistorted_image](/assets/rosRoboCar/Undistorted_image.png)
> All data should be added to `robocar_ws/src/donkeycar/config/camera_info.yaml`

## Steering a throttle calibration

Follow [instructions](https://docs.donkeycar.com/guide/calibrate/).
> All channels and PWM values should be added to `robocar_ws/src/donkey_actuator/config/servos.yaml`

## Add teleoperation

#### Keyboard control

Just install `ros-noetic-teleop-twist-keyboard`

#### Joystick control

- Connect your joystick to a car.

- test your joystick with original `donkey` software

   - run `donkey createjs` command in `~/mycar` directory

   - map buttons with actions

   - modify `myconfig.py` to set `CONTROLLER_TYPE="custom"` to use your `my_joystick.py` controller

   - run your car with `python3 manage.py drive --js` command

## Record bag files

- Launch donkeycar on the car
{% highlight shell %}
roslaunch donkeycar teleop.launch
{% endhighlight %}
- On PC go to the folder `robocar_ws/src/path_from_image/bagfiles/` and record messages
{% highlight shell %}
rosbag record -a
# to record only image topic
rosbag record -O subset /raspicam/image/compressed
{% endhighlight %}
- If bag file has several topics , but you need only one, just filter rosbag with
{% highlight shell %}
rosbag filter subset.bag <NEW_NAME>.bag 'topic == "<YOUR_TOPIC>"'
{% endhighlight %}
- to extract image files from bag a create launch file with
{% highlight xml %}
<launch>
  <node pkg="rosbag" type="play" name="rosbag" required="true" args="$(find path_from_image)/bagfiles/subset.bag"/>
  <node name="extract" pkg="image_view" type="extract_images" respawn="false" required="true" output="screen" cwd="<FOLDER_TO_SAVE_IMAGES>">
    <remap from="image" to="raspicam/image_raw"/>
  </node>
</launch>
{% endhighlight %}
- to decompress images also run in terminal
{% highlight shell %}
rosrun image_transport republish compressed in:=raspicam/image raw out:=raspicam/image_raw
{% endhighlight %}

<a href="{{next_post.url | escape}}">Next: {{ next_post.subtitle }}</a>