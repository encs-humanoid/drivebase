#!/usr/bin/python
#===================================================================
# Ultrasound Downsample Node
#
# Subscribes to:
#   /ultrasound
#
# Publishes to:
#   /sonar_array_10hz
#
# Compress ultrasound messages into a single message containing
# an array of values which is published every 10 seconds.
#===================================================================
from __future__ import division
import drivebase.msg
import geometry_msgs.msg
import math
import rospy
import sensor_msgs.msg
import std_msgs.msg
import sys
import tf
import time


class Sonar10HzNode(object):

    def __init__(self):
	rospy.init_node('sonar_10hz_node')
	rospy.myargv(sys.argv) # process ROS args and return the rest

	self.msg = None
	self.ranges = []
	self.timestamps = []

	range_topic = self.get_param("~in", "/ultrasound")
	out_topic = self.get_param("~out", "/sonar_array_10hz")
	obs_topic = self.get_param("~obs", "/obs")
	self.pub_freq = int(self.get_param("~hz", "10"))
	self.num_sensors = int(self.get_param("~num_sensors", "6"))
	self.range_expire_secs = float(self.get_param("~expire", "0.5"))

	for _ in range(self.num_sensors):
	    self.ranges.append(-1)  # initialize to negative value to indicate undefined range
	    self.timestamps.append(time.time())  # initialize timestamp value for aging range information

	# Initialize the tf listener
	self.tf_listener = tf.TransformListener()

	self.pub = rospy.Publisher(out_topic, std_msgs.msg.String, queue_size=10)
	range_sub = rospy.Subscriber(range_topic, sensor_msgs.msg.Range, self.on_range)
	obs_sub = rospy.Subscriber(obs_topic, drivebase.msg.Obstacle, self.on_obstacle)


    def get_param(self, param, default):
	value = rospy.get_param(param, default)
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param), value)
	return value


    def run(self):
    	rate = rospy.Rate(self.pub_freq)
	while not rospy.is_shutdown():
	    sensor_values = " ".join([str(int(r * 1000)) for r in self.ranges])
	    self.pub.publish(sensor_values)
	    for i in range(len(self.ranges)):
	    	if self.ranges[i] >= 0 and time.time() - self.timestamps[i] > self.range_expire_secs:
		    self.ranges[i] = 5	 # set range to 5m (no obstacle) if timestamp is too old
	    rate.sleep()


    def on_range(self, range):
	frame_id = range.header.frame_id
	index = int(frame_id.split("_")[-1]) - 1
	self.ranges[index] = range.range
	self.timestamps[index] = time.time()


    def on_obstacle(self, obs):
    	if obs.header.frame_id == 'neato_laser':
	    # convert Obstacle (dir + range) to PointStamped (x, y, z)
	    p = geometry_msgs.msg.PointStamped()
	    p.header = obs.header
	    p.point.x = obs.range * obs.dx
	    p.point.y = obs.range * obs.dy
	    p.point.z = obs.range * obs.dz
	    sonar_frames = ['/ultrasound_1', '/ultrasound_2', '/ultrasound_3', '/ultrasound_4']
	    points = {}
	    sq_distances = {}
	    for frame in sonar_frames:
		points[frame] = self.tf_listener.transformPoint(frame, p)
		p1 = points[frame]
		sq_distances[frame] = p1.point.x * p1.point.x + p1.point.y * p1.point.y
		#if frame == '/ultrasound_1':
		#    rospy.loginfo('Obstacle at point (%lf, %lf) and dist %lf wrt %s', p1.point.x, p1.point.y, math.sqrt(sq_distances[frame]), frame)
	    
	    positive_points = {
	    	frame: point for frame, point in points.items() if point.point.y > 0
	    }

	    for frame in positive_points.keys():
	    	sensor_index = int(frame.split('_')[-1]) - 1
		pp = points[frame].point
		if abs(math.atan2(pp.x, pp.y)) < math.pi/6:  # x, y swapped because we want angle to y-axis
		    distance = math.sqrt(sq_distances[frame])
		    if self.ranges[sensor_index] < 0 or distance < self.ranges[sensor_index]:
			self.ranges[sensor_index] = distance
			self.timestamps[sensor_index] = time.time()
		#rospy.loginfo('Obstacle reported at sensor position %d with range %lf', sensor_index + 1, points[frame].point.y)
	    rospy.loginfo(str(self.ranges))


if __name__ == '__main__':
    try:
	node = Sonar10HzNode()
	node.run()
    except rospy.ROSInterruptException:
	pass
