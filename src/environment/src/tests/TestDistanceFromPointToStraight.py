import unittest
from utils.math_helpers import point_to_straight_distance, get_straight_from_points


class TestDistanceFromPointToStraight(unittest.TestCase):
    def test_distance_one(self):
        # y = x
        straight = (1, -1, 0)  # a, b, c
        point_1 = (2, 2)  # x, y
        point_2 = (10, 10)
        distance = point_to_straight_distance(straight, point_1)
        self.assertAlmostEqual(distance, 0, places=0)
        distance = point_to_straight_distance(straight, point_2)
        self.assertAlmostEqual(distance, 0, places=0)

    def test_distance_two(self):
        # y = 3x
        straight = (3, -1, 0)
        point_1 = (1, 3)
        point_2 = (-7, -21)
        point_3 = (0, 0)
        point_4 = (1, 0)
        distance = point_to_straight_distance(straight, point_1)
        self.assertAlmostEqual(distance, 0, places=0)
        distance = point_to_straight_distance(straight, point_2)
        self.assertAlmostEqual(distance, 0, places=0)
        distance = point_to_straight_distance(straight, point_3)
        self.assertAlmostEqual(distance, 0, places=0)
        distance = point_to_straight_distance(straight, point_4)
        self.assertAlmostEqual(distance, 0.9486832980505138, places=5)

    def test_distance_three(self):
        # y = .4/.33*x + 1.3/.33
        straight = (.4, -.33, 1.3)
        point_1 = (2, 6.363636363636363)  # point belonging to this straight
        point_2 = (1, 1)  # point not belonging to this straight

        distance = point_to_straight_distance(straight, point_1)
        self.assertAlmostEqual(distance, 0, places=5)
        distance = point_to_straight_distance(straight, point_2)
        self.assertAlmostEqual(distance, 2.6419534789997643, places=5)


if __name__ == '__main__':
    unittest.main()
