import unittest
from utils.math_helpers import point_to_straight_distance, get_straight_from_points


class TestStraightFromPoints(unittest.TestCase):
    def test_calculate_straight_from_points_one(self):
        point_1 = (0, 0)  # x, y
        point_2 = (1, 1)
        a, b, c = get_straight_from_points(point_1, point_2)
        self.assertEqual(a, 1)
        self.assertEqual(b, -1)
        self.assertEqual(c, 0)

    def test_calculate_straight_from_points_two(self):
        point_1 = (0, 0)  # x, y
        point_2 = (1, 0)
        a, b, c = get_straight_from_points(point_1, point_2)
        self.assertEqual(a, 0)
        self.assertEqual(b, 1)
        self.assertEqual(c, 0)

    def test_calculate_straight_from_points_three(self):
        point_1 = (0, 1)  # x, y
        point_2 = (1, 3)
        a, b, c = get_straight_from_points(point_1, point_2)
        self.assertEqual(a, 2)
        self.assertEqual(b, -1)
        self.assertEqual(c, 1)

    def test_calculate_straight_from_points_three(self):
        point_1 = (0, 1)  # x, y
        point_2 = (1, 0)
        a, b, c = get_straight_from_points(point_1, point_2)
        self.assertEqual(a, -1)
        self.assertEqual(b, -1)
        self.assertEqual(c, 1)

if __name__ == '__main__':
    unittest.main()
