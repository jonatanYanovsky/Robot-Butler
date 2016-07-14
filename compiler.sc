#!/bin/bash

cd ~/catkin_ws/

echo " "
echo "--------------Running [source devel/setup.bash]:"
echo " "

source devel/setup.bash

echo " "
echo "--------------Running [catkin_make]:"
echo " "

catkin_make

cd ~/catkin_ws/src/master/src/

echo " "
echo "--------------Finished Compiling & Building code!"
echo " "
