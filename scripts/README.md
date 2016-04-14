Scripts
=======

This folder has lots of important and convenient setup scripts. The idea is that the robots run the `odroid_setup.bash` script, and other machines can run the `simulator_setup.bash` script. It would probably be beneficial to create a `host_setup.bash` script for the vision host.

You can autorun these scripts by `source`-ing them in your `~/.bashrc` file (which is run on every login):

```bash
# ~/.bashrc

source ~/dev/robot-soccer/scripts/simulator_setup.bash
```

This allows user to simply type `simulator_1v1` or `simulator_2v2` in their simulation environment to start the Gazebo simulator. When you are ready, you can type `sim_go` (all in the same terminal). Then, by running `sim_command_center`, the `Chicken McThuggets Command Center` will open, allowing you to control the robots, or press the `Spacebar Toggle` button to start the AI.