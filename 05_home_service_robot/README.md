# Assignment 5: Home Service Robot

1) Robot navigates to a pick up location where a marker is displayed.
2) Once robot has reached the pick up destination, the marker is deleted.
3) Robot navigates to a drop off location.
4) Once robot has reach the drop off destination, marker reappears.

Localization of `robo` with a map is performed using [AMCL](http://wiki.ros.org/amcl).   
Map is generated with [PGM map creator](https://github.com/hyfan1116/pgm_map_creator).   
Navigation in `pick_objects` node is performed using [actionlib](http://wiki.ros.org/actionlib).

## To launch robo with navigation goals
```sh
$ ./scripts/home_service.sh
```

## Screenshots
Displaying marker at drop off destination
![Navigation Goal](images/navigation_goal.jpg)

## References
* [Map based navigation](http://edu.gaitech.hk/turtlebot/map-navigation.html)
* [Callbacks and Spinning](http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning)
* [Displaying Markers](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes)
