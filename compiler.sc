#!/bin/bash

echo " "
echo "--------------Running [catkin_make pose_nav]:"
echo " "

cd ~/catkin_ws/

catkin_make pose_nav

echo " "
echo "--------------running [source devel/setup.bash]:"
echo " "

source devel/setup.bash

cd ~/catkin_ws/src/pose_nav/src/

echo " "
echo "--------------finished compiling and building code!"
echo " "
