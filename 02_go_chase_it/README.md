# Assignment 2: Go Chase it!

To build and launch:
```
$ cd catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roslaunch my_robot world.launch
```

To publish velocity commands to the robot’s wheel actuators:
```
rostopic pub /cmd_vel geometry_msgs/Twist "[0.1, 1, 1]" "[0, 0, 0.1]"
```
