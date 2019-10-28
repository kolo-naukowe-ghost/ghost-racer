import unittest
import numpy as np
from utils.math_helpers import point_to_straight_distance, get_straight_from_points, angle_between_vectors,\
    angle_between_two_straight


class TestAngleCalculators(unittest.TestCase):
    def test_angle_between_vectors_one(self):
        vec1 = np.array([1, 0])
        vec2 = np.array([0, 1])
        angle = angle_between_vectors(vec1, vec2)

        self.assertAlmostEqual(angle, np.pi / 2)

    def test_angle_between_vectors_two(self):
        vec1 = np.array([1, 1])
        vec2 = np.array([-1, -1])
        angle = angle_between_vectors(vec1, vec2)

        self.assertAlmostEqual(angle, np.pi)

    def test_angle_between_vectors_three(self):
        vec1 = np.array([1, 0])
        vec2 = np.array([1, 1])
        angle = angle_between_vectors(vec1, vec2)

        self.assertAlmostEqual(angle, np.pi / 4)

    def test_angle_between_straights_one(self):
        point1 = np.array([0, 0])
        point2 = np.array([1, 1])

        point3 = np.array([0, 0])
        point4 = np.array([1, -1])

        straight1 = get_straight_from_points(point1, point2)
        straight2 = get_straight_from_points(point3, point4)

        print(straight1, straight2)

        angle = angle_between_two_straight(straight1, straight2)

        self.assertAlmostEqual(angle, 90)

    def test_angle_between_straights_two(self):
        point1 = np.array([0, 0])
        point2 = np.array([1, 0])

        point3 = np.array([0, 0])
        point4 = np.array([0, 1])

        straight1 = get_straight_from_points(point1, point2)
        straight2 = get_straight_from_points(point3, point4)

        print(straight1, straight2)

        angle = angle_between_two_straight(straight1, straight2)

        self.assertAlmostEqual(angle, 90)

    def test_angle_between_straights_three(self):
        point1 = np.array([0, 0])
        point2 = np.array([-1, 1])

        point3 = np.array([0, 0])
        point4 = np.array([1, 1])

        straight1 = get_straight_from_points(point1, point2)
        straight2 = get_straight_from_points(point3, point4)

        print(straight1, straight2)

        angle = angle_between_two_straight(straight1, straight2)

        self.assertAlmostEqual(angle, 90)



if __name__ == '__main__':
    unittest.main()
