import unittest
import drivebase.msg
import std_msgs.msg
import numpy as np
import obs_avoid

class TestObstacleAvoidance(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestObstacleAvoidance, self).__init__(*args, **kwargs)
	"""
	  Y
	  ^
	  | front
	  |
	1 X       X
	  |
    left  | Robot    right
	  |
	0 X-------X------->X
	  0       1
	    back
	"""
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


    def test_should_calculate_vector_from_boundary_to_obstacle(self):
    	"""
	Given the robot boundary and an obstacle position,
	Then calculate the vector from the closest point on the boundary to the obstacle.
	"""
	# define obstacle position as centered in front of the robot
	obs = self.obs
	obs.dx = 0.5
	obs.dy = 2

	# interpolate the midpoints around the boundary
	boundary = obs_avoid.interpolate_points(self.boundary, 1)

	# find closest point on boundary
	r_index = obs_avoid.find_closest_point_on_boundary(obs, boundary)

	# get the clostest point
	r = boundary[r_index]

	# should be the midpoint on the front of the robot
	self.assertEquals("[ 0.5  1. ]", str(r), "expected midpoint on the front of the robot, but was " + str(r))

	# calculate direction from closest point to the obstacle
	ov = obs_avoid.obs_vec_xy(obs, r)

	# should point directly to front
	self.assertEquals("[ 0.  1.]", str(ov))


    def test_calculate_velocity_adjustment(self):
    	"""
	Given the robot boundary and a linear velocity,
	When an obstacle is in close proximity,
	Then the velocity should be reduced in the direction of the obstacle.
	"""
	# define obstacle position as centered in front of the robot
	obs = self.obs
	obs.dx = 0.5
	obs.dy = 2

	# interpolate the midpoints around the boundary
	boundary = obs_avoid.interpolate_points(self.boundary, 1)

	# find closest point on boundary
	r_index = obs_avoid.find_closest_point_on_boundary(obs, boundary)

	# get the closest point
	r = boundary[r_index]

	# should be the midpoint on the front of the robot
	self.assertEquals("[ 0.5  1. ]", str(r), "expected midpoint on the front of the robot, but was " + str(r))

	# calculate direction from closest point to the obstacle
	ov = obs_avoid.obs_vec_xy(obs, r)

	# calculate projection of linear velocity on obstacle direction vector
	# TODO add tangential linear velocity derived from the angular velocity to the simple linear velocity
	v = np.array((0, 1))  # 1 m/s velocity in Y direction
	p = np.dot(v, ov)

	# should be in the same direction
	self.assertTrue(abs(np.linalg.norm(p) - 1) < 1.0e-5, "expected obstacle in same direction as velocity")

	# TODO finish the test with compute_scale and verification of reduced velocity in direction of obstacle

	# TODO split this out into test for the projected velocity in the obstacle direction and separate test for the velocity adjustment

	# TODO eliminate duplication in the test fixture between this and the previous test


if __name__ == '__main__':
    unittest.main()
