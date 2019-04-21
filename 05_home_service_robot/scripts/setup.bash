#!/bin/bash

virtualenv -p /usr/bin/python2 venv
source venv/bin/activate
pip install catkin_pkg pyyaml empy rospkg defusedxml numpy
