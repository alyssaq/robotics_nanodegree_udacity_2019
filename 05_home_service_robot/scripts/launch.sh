#!/bin/sh
xterm  -e  " source devel/setup.bash; roslaunch homerobo world.launch " &
sleep 5
xterm  -e  " source devel/setup.bash; roslaunch pick_objects pick_objects.launch" &
