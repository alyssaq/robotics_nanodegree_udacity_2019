#!/usr/bin/env bash

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

apt-get update
apt-get -y install ros-kinetic-desktop-full
apt-get -y install python-rosinstall python-rosinstall-generator python-wstool build-essential

#initialize RosDep
rosdep init
rosdep update

#Install turtlebot
apt-get install -y ros-kinetic-librealsense
apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-interactions ros-kinetic-turtlebot-simulator ros-kinetic-kobuki-ftdi ros-kinetic-ar-track-alvar-msgs -y

#Setup ROS environment variables
echo "source /opt/ros/kinetic/setup.bash" >> /home/vagrant/.bashrc
source /home/vagrant/.bashrc
