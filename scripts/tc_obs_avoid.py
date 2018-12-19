import unittest
import drivebase.msg
import geometry_msgs
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


    def test_select_closest_point(self):
    	"""
	Given the boundary points of the robot,
	When an obstacle is near the robot,
	Then the closest point to the obstacle should be selected.
	"""
	self.obs.point.x = -1
	self.obs.point.y = 0
	self.assertEquals(0, obs_avoid.find_closest_point_on_boundary(self.obs, self.boundary)[1])

	self.obs.point.x = 2
	self.assertEquals(1, obs_avoid.find_closest_point_on_boundary(self.obs, self.boundary)[1])

	self.obs.point.y = 2
	self.assertEquals(2, obs_avoid.find_closest_point_on_boundary(self.obs, self.boundary)[1])

	self.obs.point.x = -1
	self.assertEquals(3, obs_avoid.find_closest_point_on_boundary(self.obs, self.boundary)[1])


    def test_should_calculate_vector_from_boundary_to_obstacle(self):
    	"""
	Given the robot boundary and an obstacle position,
	Then calculate the vector from the closest point on the boundary to the obstacle.
	"""
	# define obstacle position as centered in front of the robot
	obs = self.obs
	obs.point.x = 0.5
	obs.point.y = 2

	# interpolate the midpoints around the boundary
	boundary = obs_avoid.interpolate_points(self.boundary, 1)

	# find closest point on boundary
	r, r_index = obs_avoid.find_closest_point_on_boundary(obs, boundary)

	# should be the midpoint on the front of the robot
	self.assertEquals("[ 0.5  1. ]", str(r), "expected midpoint on the front of the robot, but was " + str(r))

	# calculate direction from closest point to the obstacle
	ov = obs_avoid.obs_vec_xy(obs, r)

	# should point directly to front
	self.assertEquals("[ 0.  1.]", str(ov))

	# define obstacle to the front right of the robot
	obs.point.x = 1
	obs.point.y = 2
	ov, r, r_index = obs_avoid.find_closest_direction_to_obstacle(obs, boundary)
	self.assertEquals("[ 1.  1.]", str(r), "expected front-right of the robot, but was " + str(r))
	self.assertEquals("[ 0.  1.]", str(ov))

	# define obstacle diagonally to the right of the robot
	obs.point.x = 2
	obs.point.y = 2
	ov, r, r_index = obs_avoid.find_closest_direction_to_obstacle(obs, boundary)
	self.assertEquals("[ 1.  1.]", str(r))
	self.assertEquals("[ 0.70710678  0.70710678]", str(ov))

	# define obstacle diagonally to the back left of the robot
	obs.point.x = -1
	obs.point.y = -1
	ov, r, r_index = obs_avoid.find_closest_direction_to_obstacle(obs, boundary)
	self.assertEquals("[ 0.  0.]", str(r))
	self.assertEquals("[-0.70710678 -0.70710678]", str(ov))


    def test_compute_scale(self):
    	"""
	Given a distance
	Then the scale is a piecewise linear function
	The scale should increase from 0 at very close distances less that the stop threshold
	up to the slow factor at the slow threshold and further up to 1 at the fast threshold
	between the thresholds, the scale is linearly interpolated.
	"""
	stop = obs_avoid.STOP_THRESHOLD
	slow = obs_avoid.SLOW_THRESHOLD
	fast = obs_avoid.FAST_THRESHOLD
	factor = obs_avoid.SLOW_FACTOR
	ranges = [0.0, stop / 2, stop, (stop + slow) / 2, slow, (slow + fast) / 2, fast, fast + 0.25, fast + 0.5]
	expected_scales = str([0.0, 0.0, 0.0, factor / 2, factor, (factor + 1.0) / 2, 1.0, 1.0, 1.0])
	self.assertEquals(expected_scales, str([round(obs_avoid.compute_scale(r), 3) for r in ranges]))


    def test_calculate_velocity_adjustment(self):
    	"""
	Given the robot boundary and a linear velocity,
	When an obstacle is in close proximity,
	Then the velocity should be reduced in the direction of the obstacle.
	"""
	# define obstacle position as centered in front of the robot
	obs = self.obs
	obs.point.x = 0.5
	obs.point.y = 2

	# interpolate the midpoints around the boundary
	boundary = obs_avoid.interpolate_points(self.boundary, 1)

	# find direction to obstacle from closest point on boundary
	ov, r, r_index = obs_avoid.find_closest_direction_to_obstacle(obs, boundary)

	# should be the midpoint on the front of the robot
	self.assertEquals("[ 0.5  1. ]", str(r), "expected midpoint on the front of the robot, but was " + str(r))

	# should point straight forward
	self.assertEquals("[ 0.  1.]", str(ov))

	# calculate projection of linear velocity on obstacle direction vector
	# TODO add tangential linear velocity derived from the angular velocity to the simple linear velocity
	v = np.array((0.0, 1.0))  # 1 m/s velocity in Y direction
	w = 0
	p = np.dot(v, ov)

	# should be in the same direction
	self.assertTrue(abs(p - 1) < 1.0e-5, "expected obstacle in same direction as velocity")

	# TODO finish the test with compute_scale and verification of reduced velocity in direction of obstacle
	dv, dw = obs_avoid.calculate_velocity_adjustment(v, w, obs, boundary, self.center_of_rotation)
	self.assertEquals("[-0. -0.]", str(dv))  # obstacle is 1 m ahead, so no adjustment

	# move the obstacle closer
	obs.point.y = 1.25  # obstacle is 25 cm ahead
	dv, dw = obs_avoid.calculate_velocity_adjustment(v, w, obs, boundary, self.center_of_rotation)
	self.assertEquals("[-0.  -0.5]", str(dv))

	# leave the obstacle in the same place, but cut the velocity by 1/2
	v /= 2.0
	dv, dw = obs_avoid.calculate_velocity_adjustment(v, w, obs, boundary, self.center_of_rotation)
	self.assertEquals("[-0.   -0.25]", str(dv))  # this shows that the adjustment is proporational to the velocity

	# double the original velocity
	v *= 4.0
	dv, dw = obs_avoid.calculate_velocity_adjustment(v, w, obs, boundary, self.center_of_rotation)
	self.assertEquals("[-0. -1.]", str(dv))  # at this distance the velocity is cut in half

	# bring the obstacle closer
	obs.point.y = 1.15  # obstacle is 15 cm ahead
	dv, dw = obs_avoid.calculate_velocity_adjustment(v, w, obs, boundary, self.center_of_rotation)
	self.assertEquals("[-0.  -1.6]", str(dv))  # the velocity adjustment is larger when the obstacle is closer

	# TODO split this out into test for the projected velocity in the obstacle direction and separate test for the velocity adjustment

	# TODO eliminate duplication in the test fixture between this and the previous test


    def test_calculate_velocity_adjustment_diagonal(self):
    	"""
	Given the robot boundary and a linear velocity heading diagonally,
	When an obstacle is in close proximity,
	Then the velocity should be reduced in the direction of the obstacle.
	"""
	# define obstacle position as centered in front of the robot
	obs = self.obs
	obs.point.x = 1.25
	obs.point.y = 1.25

	# interpolate the midpoints around the boundary
	boundary = obs_avoid.interpolate_points(self.boundary, 1)

	# robot velocity
	v = np.array((0.707, 0.707))  # 1 m/s velocity in Y direction
	w = 0

	dv, dw = obs_avoid.calculate_velocity_adjustment(v, w, obs, boundary, self.center_of_rotation)
	self.assertEquals("[-0.20707551 -0.20707551]", str(dv))
	
	# move obstacle to the left of the robot
	obs.point.x = -0.25

	dv, dw = obs_avoid.calculate_velocity_adjustment(v, w, obs, boundary, self.center_of_rotation)
	self.assertEquals("[0, 0]", str(dv))

	# change velocity to front-left
	v[0] = -v[0]

	dv, dw = obs_avoid.calculate_velocity_adjustment(v, w, obs, boundary, self.center_of_rotation)
	self.assertEquals("[ 0.20707551 -0.20707551]", str(dv))


    def test_calculate_velocity_adjustment_rotational(self):
    	"""
	Given the robot boundary and angular velocity,
	When an obstacle is in close proximity,
	Then the angular velocity should be reduced in the direction of rotation toward the obstacle.
	"""
	# define obstacle position as close to the corner of the robot
	obs = self.obs
	obs.point.x = 0.8
	obs.point.y = 1.2

	# interpolate the midpoints around the boundary
	boundary = obs_avoid.interpolate_points(self.boundary, 1)

	# robot velocity
	v = np.array((0.0, 0.0))  # no linear velocity
	w = -1.0  # 1 rad/sec clockwise

	# no adjustment since rotating away from obstacle
	dv, dw = obs_avoid.calculate_velocity_adjustment(v, w, obs, boundary, self.center_of_rotation)
	self.assertEquals("[0, 0]", str(dv))
	self.assertEquals("0", str(dw))

	# change the direction of rotation
	w = 1.15123928305  # rad/sec counterclockwise

	dv, dw = obs_avoid.calculate_velocity_adjustment(v, w, obs, boundary, self.center_of_rotation)
	self.assertEquals("[0, 0]", str(dv))
	self.assertEquals("-0.5", str(dw))

	# move obstacle closer to robot corner with same rotational velocity
	obs.point.x = 0.9
	obs.point.y = 1.1

	dv, dw = obs_avoid.calculate_velocity_adjustment(v, w, obs, boundary, self.center_of_rotation)
	self.assertEquals("[0, 0]", str(dv))
	self.assertEquals("-0.96049571322", str(dw))  # should be larger correction than before

	# move obstacle to other side of robot
	obs.point.x = 0.2
	obs.point.y = -0.2

	dv, dw = obs_avoid.calculate_velocity_adjustment(v, w, obs, boundary, self.center_of_rotation)
	self.assertEquals("[0, 0]", str(dv))
	self.assertEquals("-0.5", str(dw))


if __name__ == '__main__':
    unittest.main()
