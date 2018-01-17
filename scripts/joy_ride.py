#!/usr/bin/python
#===================================================================
# Joy Ride
#
# Subscribes to:
#   /joy
#
# Publishes to:
#   /cmd_vel
#
# Translate joystick signals into movement commands (Joy -> Twist)
#===================================================================
from __future__ import division
import argparse
import atexit
import geometry_msgs.msg
import rospy
import sensor_msgs.msg
import sys

AXIS_LX = 0
AXIS_LY = 1
AXIS_RZ = 3

class ObstacleAvoidanceNode(object):

    def __init__(self):
	rospy.init_node('joy_ride_node')
	rospy.myargv(sys.argv) # process ROS args and return the rest

	joy_topic = rospy.get_param("~joy", "/joy")
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~joy'), joy_topic)
	cmd_topic = rospy.get_param("~cmd", "/cmd_vel")
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~cmd'), cmd_topic)

	self.cmd_pub = rospy.Publisher(cmd_topic, geometry_msgs.msg.Twist, queue_size=50)
	joy_sub = rospy.Subscriber(joy_topic, sensor_msgs.msg.Joy, self.on_joy)


    def run(self):
	rospy.spin()


    def on_joy(self, joy):
    	global AXIS_LX, AXIS_LY, AXIS_RZ;
	# TODO figure out how to get constent units:  m/s and rad/s
	msg = geometry_msgs.msg.Twist()
	msg.linear.x = -joy.axes[AXIS_LX]
	msg.linear.y = joy.axes[AXIS_LY]
	msg.linear.z = 0
	msg.angular.x = 0
	msg.angular.y = 0
	msg.angular.z = -joy.axes[AXIS_RZ]
	self.cmd_pub.publish(msg)


if __name__ == '__main__':
    try:
	node = ObstacleAvoidanceNode()
	node.run()
    except rospy.ROSInterruptException:
	pass


