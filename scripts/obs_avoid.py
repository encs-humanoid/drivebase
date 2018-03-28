#!/usr/bin/python
#===================================================================
# Obstacle Avoidance Node
#
# Subscribes to:
#   /cmd_vel
#   /ultrasound
#
# Publishes to:
#   /cmd_vel_raw
#
# Modifies the twist commands using distance information from
# the ultrasound sensors.
#===================================================================
from __future__ import division
import argparse
import atexit
import geometry_msgs.msg
import numpy as np
import math
import rospy
import sensor_msgs.msg
import serial
import std_msgs.msg
import sys


FAST_THRESHOLD = 0.5  # 1 meter
SLOW_THRESHOLD = 0.2  # 40 cm
STOP_THRESHOLD = 0.10 # 15 cm
SLOW_FACTOR = 0.4


def vec(angle):
    rad = angle * math.pi / 180.0
    return np.array([math.cos(rad), math.sin(rad)])


class ObstacleAvoidanceNode(object):
    def __init__(self):
	rospy.init_node('obs_avoid_node')
	rospy.myargv(sys.argv) # process ROS args and return the rest

	cmd_topic = rospy.get_param("~cmd", "/cmd_vel_raw")
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~cmd'), cmd_topic)
	range_topic = rospy.get_param("~range", "/ultrasound")
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~range'), range_topic)
	out_topic = rospy.get_param("~out", "/cmd_vel")
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~out'), out_topic)
	control_topic = rospy.get_param("~control", "/control")
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~control'), control_topic)

	self.pub = rospy.Publisher(out_topic, geometry_msgs.msg.Twist, queue_size=50)
	cmd_sub = rospy.Subscriber(cmd_topic, geometry_msgs.msg.Twist, self.on_twist)
	range_sub = rospy.Subscriber(range_topic, sensor_msgs.msg.Range, self.on_range)
	control_sub = rospy.Subscriber(control_topic, std_msgs.msg.String, self.on_control)

	self.ranges = [99.0, 99.0, 99.0, 99.0, 99.0, 99.0]  # in meters
	# the sensor vectors are based on the orientation of the sensors as attached to the hardware
	self.sensors = [ vec(90), vec(90), vec(45), vec(135), vec(0), vec(180) ]
	self.twist = None
	self.scale = 1.0


    def run(self):
	rospy.spin()


    def on_twist(self, twist):
    	self.twist = twist
	twist = self.adjust_twist(self.twist)
	if twist is not None:
	    self.pub.publish(twist)
	#scale = self.compute_scale(twist)
	#twist.linear.x = scale * twist.linear.x
	#twist.linear.y = scale * twist.linear.y
	#twist.angular.z = 0.5 * twist.angular.z
	#self.pub.publish(twist)
	#self.twist = twist
	#self.scale = scale

    def adjust_twist(self, twist):
    	''' return a modified twist message that takes into account the range information from obstacle sensors '''
	if twist is None:
	    return None
	#min_index = np.argmin(self.ranges)  # adjust direction by removing the component associated with the closest sensor
	#scale = self.compute_scale(twist, min_index)
	v = np.array([twist.linear.x, twist.linear.y])
	n = np.linalg.norm(v)
	if n > 0:   # if velocity is nonzero
	    for index, range in enumerate(self.ranges):
		scale = self.compute_scale(twist, index)
		p = np.dot(v, self.sensors[index])  # calculate projection on distance sensor vector
		if (p > 0):		# if a component of velocity is going toward the sensor
		    v -= (1.0 - scale) * p * self.sensors[index]
	adj_twist = geometry_msgs.msg.Twist()
	adj_twist.linear = geometry_msgs.msg.Vector3()
	adj_twist.linear.x = v[0]
	adj_twist.linear.y = v[1]
	adj_twist.linear.z = twist.linear.z
	adj_twist.angular = geometry_msgs.msg.Vector3()
	adj_twist.angular.x = twist.angular.x
	adj_twist.angular.y = twist.angular.y
	adj_twist.angular.z = 0.5 * twist.angular.z
	return adj_twist


    def compute_scale(self, twist, index):
    	global FAST_THRESHOLD, SLOW_THRESHOLD, STOP_THRESHOLD, SLOW_FACTOR
	range = self.ranges[index]
	scale = 1.0
	if range < STOP_THRESHOLD:
	    scale = 0.0
	elif range < SLOW_THRESHOLD:
	    scale = SLOW_FACTOR * (range - STOP_THRESHOLD) / (SLOW_THRESHOLD - STOP_THRESHOLD)
	elif range < FAST_THRESHOLD:
	    scale = SLOW_FACTOR + (1 - SLOW_FACTOR) * (range - SLOW_THRESHOLD) / (FAST_THRESHOLD - SLOW_THRESHOLD)
#	v = np.array([twist.linear.x, twist.linear.y])
#	n = np.linalg.norm(v)
#	if n > 0:   # if velocity is nonzero
#	    u = v / n  # get velocity unit vector (direction of travel)
#	    p = np.dot(u, self.sensors[min_index])  # calculate projection on minimum distance sensor
#	    if (p < 0):
#		scale = 1.0
#	    	#scale = scale + (1 - scale) * (1 - p)
#	#rospy.loginfo("%d, %g, %g", min_index, range, scale)
	return scale


    def on_range(self, range):
	frame_id = range.header.frame_id
	index = int(frame_id.split("_")[-1]) - 1
	if range.range > 0:
	    self.ranges[index] = range.range
	    twist = self.adjust_twist(self.twist)
	    if twist is not None:
	    	self.pub.publish(twist)
#	    if self.twist is not None:
#		scale = self.compute_scale(self.twist)
#		if self.scale > 0:
#		    twist = self.twist
#		    twist.linear.x = scale * twist.linear.x / self.scale
#		    twist.linear.y = scale * twist.linear.y / self.scale
#		    #twist.angular.z = scale * twist.angular.z / self.scale
#		    self.scale = scale
#		    self.pub.publish(twist)
	    	
	#a = ["_" for _ in self.ranges]
	#a[index] = "*"
	#rospy.loginfo(a)
	rospy.loginfo(["{0:.2f}".format(r) for r in self.ranges])


    def on_control(self, msg):
	if msg == "reset" and self.full_stop:
	    rospy.loginfo('received reset control message')


if __name__ == '__main__':
    try:
	node = ObstacleAvoidanceNode()
	node.run()
    except rospy.ROSInterruptException:
	pass


