#!/bin/bash
source /home/pi/.bashrc

#while [ 1 ]; do
#
#  if [ $(ip add sh dev wlan0 | grep inet | wc -l) -ne 0 ]; then
#     break
#  fi
#
#  sleep 1

# done

sleep 30

roscore > /home/pi/roscore.log 2>&1 &
