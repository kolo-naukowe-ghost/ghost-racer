import unittest
from utils.math_helpers import point_to_straight_distance, get_straight_from_points, get_two_straight_lines_intersection


class TestLinearTransforms(unittest.TestCase):
    def test_intersection(self):
        straight_line_one = (3, -1, 0)
        straight_line_two = (0, -1, 3)

        intersection = get_two_straight_lines_intersection(straight_line_one, straight_line_two)

        self.assertAlmostEqual(intersection[0], 1, 5)
        self.assertAlmostEqual(intersection[1], 3, 5)

        straight_line_one = (3, -1, 1)
        straight_line_two = (-2, -1, 6)

        intersection = get_two_straight_lines_intersection(straight_line_one, straight_line_two)

        self.assertAlmostEqual(intersection[0], 1, 5)
        self.assertAlmostEqual(intersection[1], 4, 5)

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

    def test_calculate_straight_from_points(self):
        point_1 = (0, 0)  # x, y
        point_2 = (1, 1)
        a, b, c = get_straight_from_points(point_1, point_2)
        self.assertEqual(a, 1)
        self.assertEqual(b, -1)
        self.assertEqual(c, 0)

if __name__ == '__main__':
    unittest.main()
