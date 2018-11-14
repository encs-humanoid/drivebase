import unittest
import drivebase.msg
import std_msgs.msg
import numpy as np
import obs_avoid

class TestObstacleAvoidance(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestObstacleAvoidance, self).__init__(*args, **kwargs)
    	self.boundary = []
	self.boundary.append((0, 0))  # back left
	self.boundary.append((1, 0))  # back right
	self.boundary.append((1, 1))  # front right
	self.boundary.append((0, 1))  # front left

	self.obs = drivebase.msg.Obstacle()
	self.obs.header = std_msgs.msg.Header()
	self.obs.header.frame_id = "base_link"
	self.obs.dz = 0


    def test_select_closest_point(self):
    	"""
	Given the boundary points of the robot,
	When an obstacle is near the robot,
	Then the closest point to the obstacle should be selected.
	"""
	self.obs.dx = -1
	self.obs.dy = 0
	self.assertEquals(0, obs_avoid.find_closest_point_on_boundary(self.obs, self.boundary))

	self.obs.dx = 2
	self.assertEquals(1, obs_avoid.find_closest_point_on_boundary(self.obs, self.boundary))

	self.obs.dy = 2
	self.assertEquals(2, obs_avoid.find_closest_point_on_boundary(self.obs, self.boundary))

	self.obs.dx = -1
	self.assertEquals(3, obs_avoid.find_closest_point_on_boundary(self.obs, self.boundary))


    def test_calculate_velocity_adjustment(self):
    	"""
	Given the robot boundary and a linear velocity,
	When an obstacle is in close proximity,
	Then the velocity should be reduced in the direction of the obstacle.
	"""
	# define obstacle position
	obs = self.obs
	obs.dx = 0.5
	obs.dy = 2

	# find closest point on boundary
	boundary = obs_avoid.interpolate_points(self.boundary, 1)
	r_index = obs_avoid.find_closest_point_on_boundary(obs, boundary)
	r = boundary[r_index]
	self.assertEquals("[ 0.5  1. ]", str(r))

	# TODO clean this into a function and share with obs_avoid.py
	v = np.array((0, 1))  # 1 m/s velocity in Y direction
	ov = obs_avoid.obs_vec_xy(obs)
	obs_avoid.normalize(np.array([obs.dx, obs.dy]) - r)
	p = np.dot(v, ov)  			# calculate projection on obstacle vector
	self.assertTrue(np.linalg.norm(p) > 0.97)


if __name__ == '__main__':
    unittest.main()
