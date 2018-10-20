#!/usr/bin/python
#===================================================================
# Laser Obstacle Detector Node
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
import drivebase.msg
import math
import numpy as np
import rospy
import sensor_msgs.msg
import sys


# from https://stackoverflow.com/a/32681075
def rle(inarray):
    """ run length encoding. Partial credit to R rle function. 
	Multi datatype arrays catered for including non Numpy
	returns: tuple (runlengths, startpositions, values) """
    ia = np.asarray(inarray)                  # force numpy
    n = len(ia)
    if n == 0: 
	return (None, None, None)
    else:
	y = np.array(ia[1:] != ia[:-1])     # pairwise unequal (string safe)
	i = np.append(np.where(y), n - 1)   # must include last element posi
	z = np.diff(np.append(-1, i))       # run lengths
	p = np.cumsum(np.append(0, z))[:-1] # positions
	return(z, p, ia[i])


class LaserObstacleNode(object):

    def __init__(self):
	rospy.init_node('laser_obs_node')
	rospy.myargv(sys.argv) # process ROS args and return the rest

	self.msg = None

	scan_topic = self.get_param("~in", "/scan")
	obs_topic = self.get_param("~out", "/obs")
	self.min_obs_angle = int(self.get_param("~min_obs_angle", "0"))
	self.max_obs_angle = int(self.get_param("~max_obs_angle", "360"))
	# define the threshold for detecting an obstacle
	self.obs_threshold = float(self.get_param("~obs_threshold", "1.0"))
	# define the minimum distance threshold to ignore
	self.min_threshold = float(self.get_param("~min_threshold", "0.06"))

	self.obs_pub = rospy.Publisher(obs_topic, drivebase.msg.Obstacle, queue_size=50)
	scan_sub = rospy.Subscriber(scan_topic, sensor_msgs.msg.LaserScan, self.on_scan)

	self.cos_theta = {}
	self.sin_theta = {}
	for theta in xrange(360):
	    self.cos_theta[theta] = math.cos(theta*math.pi/180)
	    self.sin_theta[theta] = math.sin(theta*math.pi/180)


    def get_param(self, param, default):
	value = rospy.get_param(param, default)
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param), value)
	return value


    def run(self):
    	rospy.spin()
	#r = rospy.Rate(20) # 20hz 
	#while not rospy.is_shutdown():
	#    if self.msg:
	#	self.cmd_pub.publish(self.msg)
	    #r.sleep()


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
	
	ranges = np.array(scan.ranges)
	within_threshold = (ranges < self.obs_threshold) & (ranges >= self.min_threshold)
	run_lengths, start_pos, values = rle(within_threshold)
	number_of_runs = len(run_lengths)
	for i in xrange(number_of_runs):
	    if values[i]:  # run is within threshold
	    	start = start_pos[i]
		if start >= self.min_obs_angle and start <= self.max_obs_angle:
		    length = run_lengths[i]
		    # find index of minimum distance
		    min_index = start + np.argmin(ranges[start:start + length])
		    theta = min_index
		    obs = drivebase.msg.Obstacle()
		    obs.header = scan.header
		    obs.dx = self.cos_theta[theta] 
		    obs.dy = self.sin_theta[theta] 
		    obs.dz = 0
		    obs.range = ranges[min_index]
		    #print(obs.range)
		    self.obs_pub.publish(obs)


if __name__ == '__main__':
    try:
	node = LaserObstacleNode()
	node.run()
    except rospy.ROSInterruptException:
	pass
