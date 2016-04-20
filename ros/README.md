catkin workspace
================

This is the workspace for catkin_make, ROS's build manager based on CMake.

Important commands are:

```bash
$ catkin_make  # do this inside of ./catkin_ws
$ source devel/setup.bash # Do this to be able to run scripts and binaries from the packages
```

It's also useful to put the following in your `~/.bashrc`:

```bash
# ROS Init
source /opt/ros/indigo/setup.bash
source /path/to/catkin_ws/devel/setup.bash
```

### Resources ###
- [A Gentle Intro to ROS](https://cse.sc.edu/~jokane/agitr/)
- [Creating a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
- [Creating a catkin package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
- [Pub-sub example](http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber)
- [Running on multiple machines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)
