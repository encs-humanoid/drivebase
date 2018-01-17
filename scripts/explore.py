#!/usr/bin/python
#===================================================================
# Explore Node
#
# Subscribes to:
#   /ultrasound
#
# Publishes to:
#   /cmd_vel
#
# Generates Twist commands to cause the robot to explore randomly
#===================================================================
from __future__ import division
import argparse
import atexit
import geometry_msgs.msg
import numpy as np
import math
import random
import rospy
import sensor_msgs.msg
import serial
import std_msgs.msg
import sys


class ExploreNode(object):
    def __init__(self):
	rospy.init_node('explore_node')
	rospy.myargv(sys.argv) # process ROS args and return the rest

	range_topic = rospy.get_param("~range", "/ultrasound")
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~range'), range_topic)
	control_topic = rospy.get_param("~control", "/control")
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~control'), control_topic)
	cmd_topic = rospy.get_param("~cmd", "/cmd_vel")
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~cmd'), cmd_topic)

	self.pub = rospy.Publisher(cmd_topic, geometry_msgs.msg.Twist, queue_size=50)
	range_sub = rospy.Subscriber(range_topic, sensor_msgs.msg.Range, self.on_range)
	control_sub = rospy.Subscriber(control_topic, std_msgs.msg.String, self.on_control)

	self.ranges = [99.0, 99.0, 99.0, 99.0, 99.0, 99.0]  # in meters


    def run(self):
    	count = 0
	direction = 0.0
    	rate = rospy.Rate(2) # Hz
	while not rospy.is_shutdown():
	    if count <= 0:
	    	if abs(direction) >= 3:
		    direction = 0
		else:
		    sample = random.uniform(-1, 1)
		    if sample < -0.33:
			direction = max(-4, direction - 1)
		    elif sample >= 0.33:
			direction = min(4, direction + 1)
		count = random.randrange(0, 4)
	    twist = geometry_msgs.msg.Twist()
	    twist.linear.x = 0.0
	    twist.linear.y = 0.5
	    twist.linear.z = 0.0
	    twist.angular.x = 0.0
	    twist.angular.y = 0.0
	    twist.angular.z = direction * 0.1
	    self.pub.publish(twist)
	    count -= 1
	    rospy.loginfo("%d, %d", count, direction)
	    rate.sleep()


    def on_range(self, range):
	frame_id = range.header.frame_id
	index = int(frame_id.split("_")[-1]) - 1
	if range.range > 0:
	    self.ranges[index] = range.range
	#rospy.loginfo(self.ranges)


    def on_control(self, msg):
	if msg == "reset" and self.full_stop:
	    rospy.loginfo('received reset control message')


if __name__ == '__main__':
    try:
	node = ExploreNode()
	node.run()
    except rospy.ROSInterruptException:
	pass


