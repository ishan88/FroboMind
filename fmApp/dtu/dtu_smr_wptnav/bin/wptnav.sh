#!/bin/sh

# parameters
WPTLIST='square_2m.txt'

# change dir to the location of the shell script
APPDIR=`dirname $(readlink -f $0)`
cd $APPDIR

# copy waypoint list to the ROS working directory
cp ../waypoints/$WPTLIST ~/.ros/waypoints.txt

# launch the ROS node
roslaunch ../launch/wptnav.launch
