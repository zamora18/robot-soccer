Soccer Simulator
================

Based off of the work from Jacob White, TA. (See [walle repo](https://github.com/embeddedprogrammer/soccer/tree/gazebo)).

We added the ability to use our ROS package with the Gazebo simulator, without having to move any code. We simply create a `roslaunch` file that remaps topics between our code and the simulator so that the correct items are being sent.

#### Ignoring This Package on the ODROID ####

In order to ignore this package in an environment where `ros-*-desktop-full` is not installed (i.e., no Gazebo), use the `CATKIN_IGNORE` file by doing the following:

```bash
touch robot-soccer/ros/src/soccersim/CATKIN_IGNORE
```

This file is ignored by git. By simply creating it, `catkin_make` will no longer try to build it.
