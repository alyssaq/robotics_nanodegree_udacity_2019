# Assignment 3: Where Am I

## To build and launch robo world with AMCL & rviz:
```
$ catkin_make
$ source devel/setup.bash
$ roslaunch robo world.launch
```

## Controlling robo:
(1) Set navigation goal in rviz:
- Click on `2D Nav Goal` on the topbar in rviz

(2) Add teleop package to navigate robo with keyboard:
```
$ cd src && git clone https://github.com/ros-teleop/teleop_twist_keyboard
$ cd .. && catkin_make
$ source devel/setup.bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
![robo_amcl](robo_amcl.jpg)

## References 
PGM map creator:
```
$ apt-get install libignition-math2-dev protobuf-compiler
$ git clone https://github.com/udacity/pgm_map_creator
Follow readme instructions
```

AMCL:
- AMCL configs:
http://wiki.ros.org/navigation/Tutorials/RobotSetup
- AMCL parameters:
http://wiki.ros.org/amcl#Parameters
