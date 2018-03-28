#!/bin/bash
#while [ 1 ]; do
#
#  if [ $(ip add sh dev wlan0 | grep inet | wc -l) -ne 0 ]; then
#     break
#  fi
#
#  sleep 1
#
#done

source /home/pi/.bashrc

sleep 5 # give roscore a chance to start

roslaunch --wait drivebase drivebase.launch > /home/pi/drivebase.log 2>&1 &
