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
BUTTON_RB = 5
BUTTON_LB = 4

MAX_NORMAL_SPEED = 0.32
MAX_TURBO_SPEED = 1.0

MAX_NORMAL_ROTATE = 0.3
MAX_TURBO_ROTATE = 1.0

class ObstacleAvoidanceNode(object):

    def __init__(self):
	rospy.init_node('joy_ride_node')
	rospy.myargv(sys.argv) # process ROS args and return the rest

	self.msg = None

	joy_topic = rospy.get_param("~joy", "/joy")
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~joy'), joy_topic)
	cmd_topic = rospy.get_param("~cmd", "/cmd_vel_raw")
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~cmd'), cmd_topic)

	self.cmd_pub = rospy.Publisher(cmd_topic, geometry_msgs.msg.Twist, queue_size=50)
	joy_sub = rospy.Subscriber(joy_topic, sensor_msgs.msg.Joy, self.on_joy)


    def run(self):
	r = rospy.Rate(20) # 10hz 
	while not rospy.is_shutdown():
	    if self.msg:
		self.cmd_pub.publish(self.msg)
	    r.sleep()


    def on_joy(self, joy):
    	global AXIS_LX, AXIS_LY, AXIS_RZ;
	# TODO figure out how to get consistent units:  m/s and rad/s

        if ((joy.buttons[BUTTON_RB] > 0) and (joy.buttons[BUTTON_LB] > 0)):
           max_speed = MAX_TURBO_SPEED
           max_rotate = MAX_TURBO_ROTATE
        else:
           max_speed = MAX_NORMAL_SPEED
           max_rotate = MAX_NORMAL_ROTATE
   
	msg = geometry_msgs.msg.Twist()
	msg.linear.x = max_speed * -joy.axes[AXIS_LX]
	msg.linear.y = max_speed * joy.axes[AXIS_LY]
	msg.linear.z = 0
	msg.angular.x = 0
	msg.angular.y = 0
	msg.angular.z = max_rotate * -joy.axes[AXIS_RZ]
	self.msg = msg


if __name__ == '__main__':
    try:
	node = ObstacleAvoidanceNode()
	node.run()
    except rospy.ROSInterruptException:
	pass


