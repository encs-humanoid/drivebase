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
import tf


FAST_THRESHOLD = 0.5  # 1 meter
SLOW_THRESHOLD = 0.2  # 40 cm
STOP_THRESHOLD = 0.10 # 15 cm
SLOW_FACTOR = 0.4


def interpolate_points(points, points_per_edge):
    """
    Generate points_per_edge new points in between pairs of existing points
    and return the expanded point set.  Assumes the points list contains
    adjacent points.
    """
    new_points = []
    if len(points) > 0:
    	new_points.append(points[0])
	for i in xrange(len(points) - 1):
	    p1 = np.array(points[i])
	    p2 = np.array(points[i + 1])
	    for j in xrange(points_per_edge):
		interval = 1.0 / (points_per_edge + 1)
		p = p1 + ((j + 1) * interval) * (p2 - p1)
		new_points.append(p)
	    new_points.append(p2)

    return new_points


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
	self.base_frame = self.get_param("~base_frame", "base_link")

	# Initialize the tf listener
	self.tf_listener = tf.TransformListener()

	self.obstacles = []
	self.twist = None
	self.scale = 1.0
	self.joy_override = False

	# Initialize robot boundary points (in base frame)
	# TODO derive robot boundary points from the URDF
	self.boundary = []
	# "base_link"
	# <origin xyz="0 0 0" rpy="0 0 0" />
      	# <box size="0.725 0.815 0.08" />
	# "platform"
	# <origin xyz="0 0.095 0" rpy="0 0 0" />
	# <box size="0.745 1.03 0.02" />
	self.boundary.append((-0.745/2, -1.030/2 + 0.095))	# back left
	self.boundary.append(( 0.725/2, -1.030/2 + 0.095))	# back right
	self.boundary.append(( 0.725/2,  1.030/2 - 0.095))	# front right
	self.boundary.append(( 0.000  ,  1.030/2 + 0.095))	# front middle
	self.boundary.append((-0.745/2,  1.030/2 - 0.095))	# front left
	self.boundary = interpolate_points(self.boundary, 8)

	self.pub = rospy.Publisher(out_topic, geometry_msgs.msg.Twist, queue_size=50)
	cmd_sub = rospy.Subscriber(cmd_topic, geometry_msgs.msg.Twist, self.on_twist)
	obs_sub = rospy.Subscriber(obs_topic, drivebase.msg.Obstacle, self.on_obstacle)
	control_sub = rospy.Subscriber(control_topic, std_msgs.msg.String, self.on_control)


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
	
	v = np.array([twist.linear.x, twist.linear.y])  # linear velocity
	w = twist.angular.z  # angular velocity
	n = np.linalg.norm(v)
	if n > 0:   # if velocity is nonzero
	    for obs in self.obstacles:
	    	# map the obstacle to the base frame
		if obs.header.frame_id != self.base_frame:
		    pointStamped = geometry_msgs.msg.PointStamped()
		    pointStamped.header = obs.header
		    pointStamped.point.x = obs.range * obs.dx
		    pointStamped.point.y = obs.range * obs.dy
		    pointStamped.point.z = obs.range * obs.dz
		    p = self.tf_listener.transformPoint(self.base_frame, pointStamped)
		    obs.header.frame_id = self.base_frame
		    obs.dx = p.point.x
		    obs.dy = p.point.y
		    obs.dz = p.point.z

		# find closest point on robot
		#b = np.array([obs.dx, obs.dy])	# obstacle position
		#r_index = np.argmin([np.linalg.norm(b - r) for r in self.boundary])
		#r = self.boundary[r_index]	# closest point on robot boundary
	    	ov = obs_vec_xy(obs)
		p = np.dot(v, ov)  			# calculate projection on obstacle vector
		if (p > 0):				# if a component of velocity is going toward the sensor
		    scale = self.compute_scale(obs.range)
		    v -= (1.0 - scale) * p * ov		# subtract a scaled amount depending on the range
		#    if abs(r[0]) > 1.0e-5:		# rx != 0
		#	w -= (1.0 - scale) * w * ((r[0] * ov[1] - r[1] * ov[0]) * ov[1]) / r[0]
		#	print('rx: w=' + str(w))
		#    elif abs(r[1]) > 1.0e-5:		# ry != 0
		#	w += (1.0 - scale) * w * ((r[0] * ov[1] - r[1] * ov[0]) * ov[0]) / r[1]
		#	print('ry: w=' + str(w))

	adj_twist = geometry_msgs.msg.Twist()
	adj_twist.linear = geometry_msgs.msg.Vector3()
	adj_twist.linear.x = v[0]
	adj_twist.linear.y = v[1]
	adj_twist.linear.z = twist.linear.z
	adj_twist.angular = geometry_msgs.msg.Vector3()
	adj_twist.angular.x = twist.angular.x
	adj_twist.angular.y = twist.angular.y
	adj_twist.angular.z = w
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
	if msg.data == "reset" and self.full_stop:
	    rospy.loginfo('received reset control message')
	elif msg.data == "joy_override":
	    rospy.loginfo('received joy_override control message')
	    self.joy_override = True
	elif msg.data == "joy_normal":
	    rospy.loginfo('received joy_normal control message')
	    self.joy_override = False


if __name__ == '__main__':
    try:
	node = ObstacleAvoidanceNode()
	node.run()
    except rospy.ROSInterruptException:
	pass


