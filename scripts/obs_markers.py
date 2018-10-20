#!/usr/bin/python
#===================================================================
# Obstacle Marker Node
#
# Subscribes to:
#   /obs
#
# Publishes to:
#   /obs_markers
#
# Creates markers for visualizing the detected obstacles.
#===================================================================
from __future__ import division
import argparse
import atexit
import drivebase.msg
import geometry_msgs.msg
import visualization_msgs.msg
import numpy as np
import math
import rospy
import sensor_msgs.msg
import serial
import std_msgs.msg
import sys


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
       return v
    return v / norm


def obs_vec_xy(obs):
    return normalize(np.array([obs.dx, obs.dy]))


class ObstacleMarkerNode(object):
    def __init__(self):
	rospy.init_node('obs_marker_node')
	rospy.myargv(sys.argv) # process ROS args and return the rest

	obs_topic = self.get_param("~in", "/obs")
	out_topic = self.get_param("~out", "/obs_marker")
	self.window = float(self.get_param("~window", "0.5")) # seconds
	self.rate = self.get_param('~rate', 10)
	self.range_mult = float(self.get_param("~range_mult", 1.0))
	self.fixed_frame = self.get_param("~fixed_frame", "/odom")
	scale = self.get_param('~scale', 0.2)
	lifetime = self.get_param('~lifetime', 0) # 0 is forever
	ns = self.get_param('~ns', 'obs')
	id = self.get_param('~id', 0)
	color = self.get_param('~color', {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 0.5})

	self.pub = rospy.Publisher(out_topic, visualization_msgs.msg.Marker, queue_size=50)
	obs_sub = rospy.Subscriber(obs_topic, drivebase.msg.Obstacle, self.on_obstacle)

	self.obstacles = []
	self.markers = visualization_msgs.msg.Marker()
	self.markers.header.frame_id = self.fixed_frame
	self.markers.ns = ns
	self.markers.id = id
	self.markers.type = visualization_msgs.msg.Marker.POINTS
	self.markers.action = visualization_msgs.msg.Marker.ADD
	self.markers.lifetime = rospy.Duration(lifetime)
	self.markers.scale.x = scale
	self.markers.scale.y = scale
	self.markers.color.r = color['r']
	self.markers.color.g = color['g']
	self.markers.color.b = color['b']
	self.markers.color.a = color['a']


    def get_param(self, param, default):
	value = rospy.get_param(param, default)
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param), value)
	return value


    def run(self):
	r = rospy.Rate(self.rate)

	# Begin the main loop
	while not rospy.is_shutdown():
	    # Set the markers header
	    self.markers.header.stamp = rospy.Time.now()

	    # Clear the markers point list
	    self.markers.points = list()

	    # Add obstacle positions
	    for obs in self.obstacles:
	    	position = geometry_msgs.msg.Point()
		#try:
		    # TODO enable this to work through TF
		    #(trans, rot)  = tf_listener.lookupTransform(self.fixed_frame, obs.header.frame_id, rospy.Time(0))
		    # TODO apply the transformation to the obstacle position to get the marker position
		    #position.x = trans[0]
		    #position.y = trans[1]
		    #position.z = trans[2]
		ov = self.range_mult * obs_vec_xy(obs)

		position.x = obs.range * ov[0]
		position.y = obs.range * ov[1]
		position.z = obs.dz

		    # Set a marker at the origin of this frame
		self.markers.points.append(position)
		#except:
		    #pass

	    # Publish the set of markers
	    self.pub.publish(self.markers)
	    r.sleep()


    def on_obstacle(self, obs):
    	# purge old messages
    	keep = [obs]
	for o in self.obstacles:
	    if rospy.get_time() - o.header.stamp.to_sec() < self.window:
	    	keep.append(o)
	self.obstacles = keep


if __name__ == '__main__':
    try:
	node = ObstacleMarkerNode()
	node.run()
    except rospy.ROSInterruptException:
	pass


