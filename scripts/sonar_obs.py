#!/usr/bin/python
#===================================================================
# Ultrasound Obstacle Detector Node
#
# Subscribes to:
#   /ultrasound
#
# Publishes to:
#   /obs
#
# Translate ultrasound messages into obstacle messages
#===================================================================
from __future__ import division
import argparse
import atexit
import drivebase.msg
import math
import numpy as np
import rospy
import sensor_msgs.msg
import sys


class SonarObstacleNode(object):

    def __init__(self):
	rospy.init_node('sonar_obs_node')
	rospy.myargv(sys.argv) # process ROS args and return the rest

	self.msg = None

	range_topic = self.get_param("~in", "/ultrasound")
	obs_topic = self.get_param("~out", "/obs")

	self.obs_pub = rospy.Publisher(obs_topic, drivebase.msg.Obstacle, queue_size=50)
	range_sub = rospy.Subscriber(range_topic, sensor_msgs.msg.Range, self.on_range)


    def get_param(self, param, default):
	value = rospy.get_param(param, default)
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param), value)
	return value


    def run(self):
    	rospy.spin()


    def on_range(self, range):
	frame_id = range.header.frame_id
	index = int(frame_id.split("_")[-1]) - 1
	if range.range > 0 and range.range <= 0.40:
	    obs = drivebase.msg.Obstacle()
	    obs.header = range.header
	    obs.dx = 0
	    obs.dy = 1
	    obs.dz = 0
	    obs.range = range.range
	    self.obs_pub.publish(obs)


if __name__ == '__main__':
    try:
	node = SonarObstacleNode()
	node.run()
    except rospy.ROSInterruptException:
	pass
