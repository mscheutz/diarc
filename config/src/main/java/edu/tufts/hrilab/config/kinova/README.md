The purpose of this directory is to house common DIARC configuration examples. 
These examples show useful configs for demos, or for starting up a 
single piece of functionality to provide a quick reference/starting point.

This directory contains:
- KinovaDemo
- Todo

### KinovaDemo

A Simple demo of starting up the Kinova and open/closing the gripper in various
positions. Make sure you've followed the rosjava setup instructions first, so
that you have the ROS Kortex, ROS MoveIt, and ROS Robotiq messages. You should
then be able to launch the Kortex ROS launch files You will need to build the
manipulator and the arm:
```bash
ant kortex
```

I'm launching the ROS Kinova driver using the code found in the [ros_kortex
repo](https://github.com/Kinovarobotics/ros_kortex), using the command
recommended by the documentation there:
```
roslaunch kortex_driver kortex_driver.launch gripper:=robotiq_2f_85
```

Once they are running, you can launch the KinovaDemo config file:
```bash
ant launch -Dmain=com.config.examples.robots.KinovaDemo
```

Tested on ROS Noetic, Ubuntu 20.04, Java 11, and Kinova Gen3, running the 2.3.0 firmware.
