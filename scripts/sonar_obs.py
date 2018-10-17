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


def vec(angle):
    rad = angle * math.pi / 180.0
    return np.array([math.cos(rad), math.sin(rad)])


class SonarObstacleNode(object):

    def __init__(self):
	rospy.init_node('sonar_obs_node')
	rospy.myargv(sys.argv) # process ROS args and return the rest

	self.msg = None

	range_topic = self.get_param("~in", "/ultrasound")
	obs_topic = self.get_param("~out", "/obs")

	self.obs_pub = rospy.Publisher(obs_topic, drivebase.msg.Obstacle, queue_size=50)
	range_sub = rospy.Subscriber(range_topic, sensor_msgs.msg.Range, self.on_range)

	self.ranges = [99.0, 99.0, 99.0, 99.0, 99.0, 99.0]  # in meters
	# the sensor vectors are based on the orientation of the sensors as attached to the hardware
	self.sensors = [ vec(90), vec(90), vec(45), vec(135), vec(0), vec(180) ]


    def get_param(self, param, default):
	value = rospy.get_param(param, default)
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param), value)
	return value


    def run(self):
    	rospy.spin()


    def on_range(self, range):
	frame_id = range.header.frame_id
	index = int(frame_id.split("_")[-1]) - 1
	if range.range > 0:
	    self.ranges[index] = range.range
	    ov = self.sensors[index]
	    obs = drivebase.msg.Obstacle()
	    obs.header = range.header
	    obs.dx = ov[0]
	    obs.dy = ov[1]
	    obs.dz = 0
	    obs.range = range.range
	    self.obs_pub.publish(obs)
	rospy.loginfo(["{0:.2f}".format(r) for r in self.ranges])


if __name__ == '__main__':
    try:
	node = SonarObstacleNode()
	node.run()
    except rospy.ROSInterruptException:
	pass
