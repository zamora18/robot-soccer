MATLAB Command Center GUI
=========================

Getting bi-directional communication with MATLAB's ROS: http://www.mathworks.com/matlabcentral/answers/119559-why-is-the-ros-subscriber-callback-in-matlab-not-triggered-when-messages-are-published-from-an-exter

Alternatively, add the correct hosts to your `/etc/hosts` file

### Building Custom Messages and Services with MATLAB ###
First thing, make sure that you have support for ROS Custom Messages (run `roboticsAddons` and go through the installer).

If you try and build the messages on MATLAB you may get an error about not being able to follow a symlink to the `toplevel.cmake`. In this case, create the path to the cmake file and add the correct contents (from a machine with the actual file).

```bash
$ sudo mkdir -p /opt/ros/indigo/share/catkin/cmake
```


Edit Java Class Path
edit('~/.matlab/R2015b/javaclasspath.txt')