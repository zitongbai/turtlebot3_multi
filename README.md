# multi_turtlebot3

ROS2 package for using multiple turtlebot3. It uses namespace to distinguish between multiple turtlebot3, without modifying the original turtlebot3 package.

# Environment

* Ubuntu 20.04
* ROS2 Foxy

# Dependences

* [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)

you can download it by using the following command.

```
sudo apt install ros-foxy-turtlebot3*
```

# Usage 

Launch the simulation environment with four turtlebot3.

```shell
ros2 launch turtlebot3_multi bringup.launch.py
```

Use namespace to launch teleop keyboard node for each turtlebot3.

```shell
ros2 run turtlebot3_teleop teleop_keyboard --ros-args --remap __ns:=/tb3_1
```

