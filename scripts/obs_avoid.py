#!/usr/bin/python
#===================================================================
# Obstacle Avoidance Node
#
# Subscribes to:
#   /cmd_vel
#   /ultrasound
#   /control
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
import drivebase.msg
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


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
       return v
    return v / norm


def obs_vec_xy(obs):
    return normalize(np.array([obs.dx, obs.dy]))


class ObstacleAvoidanceNode(object):
    def __init__(self):
	rospy.init_node('obs_avoid_node')
	rospy.myargv(sys.argv) # process ROS args and return the rest

	cmd_topic = self.get_param("~cmd", "/cmd_vel_raw")
	obs_topic = self.get_param("~in", "/obs")
	out_topic = self.get_param("~out", "/cmd_vel")
	control_topic = self.get_param("~control", "/control")
	self.window = float(self.get_param("~window", "0.5")) # seconds

	self.pub = rospy.Publisher(out_topic, geometry_msgs.msg.Twist, queue_size=50)
	cmd_sub = rospy.Subscriber(cmd_topic, geometry_msgs.msg.Twist, self.on_twist)
	obs_sub = rospy.Subscriber(obs_topic, drivebase.msg.Obstacle, self.on_obstacle)
	control_sub = rospy.Subscriber(control_topic, std_msgs.msg.String, self.on_control)

	self.obstacles = []
	self.twist = None
	self.scale = 1.0
	self.joy_override = False


    def get_param(self, param, default):
	value = rospy.get_param(param, default)
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param), value)
	return value


    def run(self):
	rospy.spin()


    def on_twist(self, twist):
	if self.joy_override:
	    self.pub.publish(twist)
	    return

    	self.twist = twist
	twist = self.adjust_twist(self.twist)
	if twist is not None:
	    self.pub.publish(twist)


    def adjust_twist(self, twist):
    	''' return a modified twist message that takes into account obstacle information '''
	if twist is None:
	    return None
	
	v = np.array([twist.linear.x, twist.linear.y])
	n = np.linalg.norm(v)
	if n > 0:   # if velocity is nonzero
	    for obs in self.obstacles:
	    	ov = obs_vec_xy(obs)
		p = np.dot(v, ov)  			# calculate projection on obstacle vector
		if (p > 0):				# if a component of velocity is going toward the sensor
		    scale = self.compute_scale(obs.range)
		    v -= (1.0 - scale) * p * ov		# subtract a scaled amount depending on the range

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


    def compute_scale(self, range):
    	global FAST_THRESHOLD, SLOW_THRESHOLD, STOP_THRESHOLD, SLOW_FACTOR
	scale = 1.0
	if range < STOP_THRESHOLD:
	    scale = 0.0
	elif range < SLOW_THRESHOLD:
	    scale = SLOW_FACTOR * (range - STOP_THRESHOLD) / (SLOW_THRESHOLD - STOP_THRESHOLD)
	elif range < FAST_THRESHOLD:
	    scale = SLOW_FACTOR + (1 - SLOW_FACTOR) * (range - SLOW_THRESHOLD) / (FAST_THRESHOLD - SLOW_THRESHOLD)
	return scale


    def on_obstacle(self, obs):
    	# purge old messages
    	keep = [obs]
	for o in self.obstacles:
	    if rospy.get_time() - o.header.stamp.to_sec() < self.window:
	    	keep.append(o)
	self.obstacles = keep


    def on_control(self, msg):
	if msg == "reset" and self.full_stop:
	    rospy.loginfo('received reset control message')
	elif msg == "joy_override":
	    self.joy_override = True
	elif msg == "joy_normal":
	    self.joy_override = False


if __name__ == '__main__':
    try:
	node = ObstacleAvoidanceNode()
	node.run()
    except rospy.ROSInterruptException:
	pass


