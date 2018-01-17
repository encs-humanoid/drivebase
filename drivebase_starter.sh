#!/bin/bash
while [ 1 ]; do

  if [ $(ip add sh dev wlan0 | grep inet | wc -l) -ne 0 ]; then
     break
  fi

  sleep 1

done

source /home/pi/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.0.0.108:11311/
export ROS_IP=10.0.0.108
#export ROS_MASTER_URI=http://192.168.0.196:11311/
#export ROS_IP=`hostname --all-ip-addresses | xargs | cut -d \  -f 1`

sleep 5 # give roscore a chance to start

roslaunch --wait drivebase drivebase.launch > /home/pi/drivebase.log 2>&1 &
