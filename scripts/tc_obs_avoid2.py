import unittest
import drivebase.msg
import geometry_msgs.msg
import std_msgs.msg
import numpy as np
import obs_avoid2
import obs_avoid



class TestObstacleAvoidance(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestObstacleAvoidance, self).__init__(*args, **kwargs)
	"""
	  Y
	  ^
	  | front
	  |
	1 *-------*
	  |       |
    left  | Robot |  right
	  |       |
	0 *-------*------->X
	  0       1
	    back
	"""
    	self.boundary = []
	self.boundary.append((0, 0))  # back left
	self.boundary.append((1, 0))  # back right
	self.boundary.append((1, 1))  # front right
	self.boundary.append((0, 1))  # front left

	self.center_of_rotation = (0.5, 0.5)

	self.obs = geometry_msgs.msg.PointStamped()
	self.obs.header = std_msgs.msg.Header()
	self.obs.header.frame_id = "base_link"
	self.obs.point.z = 0



    def test_vel_adj(self, newx, newy, expdv):
    	
	# center of robot is 0.5, 0.5
	# set velocity
	# set obstacle co-ordinates for x and y
	# start with 0.5 and 1 so obs is just touching the front edge
        # call valocity adj to get value of adjustment needed

	w = 0
	v = np.array((0.0, 1.0))  # 1 m/s velocity in Y direction
	self.obs.point.x = newx
	self.obs.point.y = newy
	dv, dw = obs_avoid2.calculate_velocity_adjustment(v, w, self.obs, self.boundary, self.center_of_rotation)
        self.assertEquals("[ 0. expdv]", str(dv))

        slow = obs_avoid.SLOW_THRESHOLD
	fast = obs_avoid.FAST_THRESHOLD
	stop = obs_avoid.STOP_THRESHOLD
	obs_pos = [0., stop/2, stop, slow/2, slow, fast/2, fast, fast+0.5]
	exp_dv =  [-1, -1, -1, -0.8, -0.6, -0.3, 0, 0]
	self.assertEquals(str(exp_dv), str(dv))
	
	
	/*
	w = 0
	v = np.array((0.0, 1.0))  # 1 m/s velocity in Y direction
	self.obs.point.x = 0.5
	self.obs.point.y = 1
	dv, dw = obs_avoid2.calculate_velocity_adjustment(v, w, self.obs, self.boundary, self.center_of_rotation)
        self.assertEquals("[ 0. -1.]", str(dv))
        */
       
    def test_should_stop_if_obs_in_front1(self):
	# change obstacle - move it farther out by 1 meter
	w = 0
	v = np.array((0.0, 1.0))  # 1 m/s velocity in Y direction
	self.obs.point.x = 0.5
	self.obs.point.y = 2
	dv, dw = obs_avoid2.calculate_velocity_adjustment(v, w, self.obs, self.boundary, self.center_of_rotation)
        self.assertEquals("[ 0. 0.]", str(dv))

    def test_should_stop_if_obs_in_front2(self):
	# change obstacle - move it farther out by 2 meters
	w = 0
	v = np.array((0.0, 1.0))  # 1 m/s velocity in Y direction
	self.obs.point.x = 0.5
	self.obs.point.y = 3
	dv, dw = obs_avoid2.calculate_velocity_adjustment(v, w, self.obs, self.boundary, self.center_of_rotation)
        self.assertEquals("[ 0. 0.]", str(dv))

    def test_should_stop_if_obs_in_front05(self):
	# change obstacle - move it closer to 0.5 meter in front of robot
	w = 0
	v = np.array((0.0, 1.0))  # 1 m/s velocity in Y direction
	self.obs.point.x = 0.5
	self.obs.point.y = 1.5
	dv, dw = obs_avoid2.calculate_velocity_adjustment(v, w, self.obs, self.boundary, self.center_of_rotation)
        self.assertEquals("[ 0. 0.]", str(dv))

    def test_should_stop_if_obs_in_front00(self):
	# change obstacle - move it to the center of robot
	w = 0
	v = np.array((0.0, 1.0))  # 1 m/s velocity in Y direction
	self.obs.point.x = 0.5
	self.obs.point.y = 0.5
	dv, dw = obs_avoid2.calculate_velocity_adjustment(v, w, self.obs, self.boundary, self.center_of_rotation)
        self.assertEquals("[ -1. -1.]", str(dv))

    def test_should_stop_if_obs_behind1(self):
	# change obstacle - move it behind
	w = 0
	v = np.array((0.0, 1.0))  # 1 m/s velocity in Y direction
	self.obs.point.x = 0.5
	self.obs.point.y = -1
	dv, dw = obs_avoid2.calculate_velocity_adjustment(v, w, self.obs, self.boundary, self.center_of_rotation)
        self.assertEquals("[ 0. 0.]", str(dv))

    def test_should_go_if_obs_to_side(self):
        pass

if __name__ == '__main__':
    unittest.main()

