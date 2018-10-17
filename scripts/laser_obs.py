#!/usr/bin/python
#===================================================================
# Laser Obs
#
# Subscribes to:
#   /scan
#
# Publishes to:
#   /obs
#
# Translate laser scan messages into obstacle messages
#===================================================================
from __future__ import division
import argparse
import atexit
import geometry_msgs.msg
import math
import numpy as np
import rospy
import sensor_msgs.msg
import sys

class LaserObstacleNode(object):

    def __init__(self):
	rospy.init_node('laser_obs_node')
	rospy.myargv(sys.argv) # process ROS args and return the rest

	self.msg = None

	scan_topic = self.get_param("~in", "/scan")
	obs_topic = self.get_param("~out", "/obs")
	self.min_obs_angle = int(self.get_param("~min_obs_angle", "90"))
	self.max_obs_angle = int(self.get_param("~max_obs_angle", "270"))

	#self.cmd_pub = rospy.Publisher(cmd_topic, geometry_msgs.msg.Twist, queue_size=50)
	scan_sub = rospy.Subscriber(scan_topic, sensor_msgs.msg.LaserScan, self.on_scan)

	self.cos_theta = {}
	self.sin_theta = {}
	for theta in xrange(self.min_obs_angle, self.max_obs_angle):
	    self.cos_theta[theta] = math.cos(theta*math.pi/180)
	    self.sin_theta[theta] = math.sin(theta*math.pi/180)


    def get_param(self, param, default):
	value = rospy.get_param(param, default)
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param), value)
	return value


    def run(self):
	r = rospy.Rate(20) # 20hz 
	while not rospy.is_shutdown():
	#    if self.msg:
	#	self.cmd_pub.publish(self.msg)
	    r.sleep()


    def on_scan(self, scan):
    	# angle_min = 0
	# angle_max = 2*pi
	# angle_increment = 1 deg (pi/180)
	# #ranges = 360
	# 0 deg points directly opposite the motor
	# 180 deg points directly to the motor
	# looking top down (bottom up as attached to the robot) the numbers increase counter clockwise
	# the range of interest for obstacle avoidance is [90-270]
	# 90 points to the left of the robot (from the top) and 270 points to the right and 180 is straight ahead
	#points = {}
	#for theta in xrange(self.min_obs_angle, self.max_obs_angle):
	#    x = scan.ranges[theta] * self.cos_theta[theta] 
	#    y = scan.ranges[theta] * self.sin_theta[theta] 
	#    points[theta] = (x, y)
	
	print(scan.ranges[90])
	print(scan.ranges[180])

	#min_index = self.min_obs_angle + np.argmin(scan.ranges[self.min_obs_angle:self.max_obs_angle])
	#print("Minimum range angle " + str(min_index) + " deg")
	#if (scan.ranges[min_index] < 0.2 and scan.ranges[min_index] >= scan.range_min):
	#    print("Obstacle at range " + str(scan.ranges[min_index]) + " m, angle " + str(min_index) + " deg")
	#else:
	#    print("No obstacle: range " + str(scan.ranges[min_index]) + " m, angle " + str(min_index) + " deg")



if __name__ == '__main__':
    try:
	node = LaserObstacleNode()
	node.run()
    except rospy.ROSInterruptException:
	pass
