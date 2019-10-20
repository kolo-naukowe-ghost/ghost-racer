from PIL import Image
import numpy as np
import os
from math import sqrt
import rospy

_PATH_TO_IMAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "dotted_board.jpeg")

# _BOARD_CENTER = np.array([1050, 2040]).T
_BOARD_CENTER1 = np.array([500, 460][::-1])
_BOARD_CENTER2 = np.array([500,1170][::-1])
_MIDDLE_BOARD_POINT = 840

def _get_angle_between_vector_and_x_axis(vec):
    angle = np.arctan2(vec[1], vec[0])
    if angle < 0:
        angle = 2 * np.pi + angle
    return angle

def sort_dots(dots, reverse = False):
    angles = [_get_angle_between_vector_and_x_axis(dot) for dot in dots]
    angles_and_dots = [(angle, dot) for angle, dot in zip(angles, dots)]
    print("sorting...")
    angles_and_dots.sort(reverse=reverse)
    print("after sort")
    print(angles_and_dots)
    return np.array([dot for _, dot in angles_and_dots])

def normalized(vec):
    return vec / np.linalg.norm(vec)


def get_straight_from_points(first_point, second_point):
    """
    :param first_point: (y, x)
    :param second_point:  (y, x)
    :return: a, b straight's coefficients
    """
    # Ax + By + C = 0
    delta_y, delta_x = second_point[0] - first_point[0], second_point[1] - first_point[1]
    if np.isclose(delta_x, 0.0):
        return 1, 0, first_point[1]

    if np.isclose(delta_y, 0.0):
        return 0, 1, first_point[0]

    a = delta_y / delta_x
    b = first_point[0] - a*first_point[1]
    return a, b, 0


def point_to_straight_distance(line, point):
    """
    :param line: a tuple of 3 points (a, b, c)
    :param point: a tuple of 2 points (y, x)
    :return:
        distance from a line to the point
    """
    a, b, c = line
    y, x = point
    denominator = sqrt(a**2 + b**2)
    if np.isclose(denominator, 0.0):
        return 0.0
    return np.abs(a * x + b * y + c) / denominator

class BoardPath:

    def __init__(self):
        checkpoints = self._load_board()
        checkpoints = self._order_checkpoints(checkpoints)
        self.dots = checkpoints
        self.current_checkpoint_index = 0
        self._last_car_position = np.zeros((2, ))
        self.car_position = self._last_car_position
        self.car_direction = np.zeros((2, ))
        self.car_size = 100 #hardcoded for debugging

    def _forward_checkpoint(self):
        self.current_checkpoint_index = (self.current_checkpoint_index + 1) % len(self.dots)

    @property
    def next_checkpoint(self):
        return self.dots[(self.current_checkpoint_index + 1) % len(self.dots)]

    @property
    def current_checkpoint(self):
        return self.dots[self.current_checkpoint_index]

    @property
    def last_checkpoint(self):
        return self.dots[self.current_checkpoint_index - 1]

    def _update(self):
        v = self.current_checkpoint - self.car_position
        distance_squared = v.dot(v)
        if distance_squared <= self.car_size**2:
            self._forward_checkpoint()

    def update(self, relative_car_x, relative_car_y):
        self.car_position = np.array([relative_car_y, relative_car_x])
        if self._last_car_position is None:
            self._last_car_position = self.car_position
        direction = self.car_position - self._last_car_position
        vector = normalized(direction)
        if np.any(vector):
            self.car_direction = vector
        self._last_car_position = self.car_position
        self._update()

    def angle_to_next_checkpoint(self):
        dir_to_checkpoint = normalized(self.current_checkpoint - self.car_position)
        car_dir = self.car_direction
        dot = np.dot(dir_to_checkpoint, car_dir)
        angle2 = np.arccos(dot / (np.linalg.norm(dir_to_checkpoint) * np.linalg.norm(car_dir)))
        det = car_dir[1] * dir_to_checkpoint[1] - car_dir[0] * dir_to_checkpoint[0]
        angle = np.arctan2(det, dot)  # atan2(y, x) or atan2(sin, cos)
        angle_rad = np.deg2rad(angle)
        angle2_rad = np.deg2rad(angle2)
        rospy.loginfo('angle is {}, angle 2 {}, cos {}, cos2 {}'.format(angle, angle2, np.cos(angle_rad), np.cos(angle2_rad)))
        return angle


    def distance_to_next_checkpoint(self):
        v = self.current_checkpoint - self.car_position
        return np.sqrt(v.dot(v))

    def distance_to_road(self):
        a, b, c = get_straight_from_points(self.current_checkpoint, self.last_checkpoint)

        if np.isclose(a, 0.0): # parallel to OX
            rospy.loginfo('distance is {}'.format(abs(self.current_checkpoint[1] - self.car_position[1])))
            return abs(self.current_checkpoint[1] - self.car_position[1])
        if np.isclose(b, 0.0): # parallel to OY
            rospy.loginfo('distance is {}'.format(abs(self.current_checkpoint[0] - self.car_position[0])))
            return abs(self.current_checkpoint[0] - self.car_position[0])

        tmp = point_to_straight_distance((a, b, c), self.car_position)
        rospy.loginfo('distance is {}'.format(tmp))
        return tmp

    def _order_checkpoints(self, checkpoints):
        # TODO
        """
        dots = checkpoints
        dots = np.argwhere(pixels > 0)
        upper_dots = dots[dots[:, 0] <= _MIDDLE_BOARD_POINT] - _BOARD_CENTER1
        lower_dots = dots[dots[:, 0] > _MIDDLE_BOARD_POINT] - _BOARD_CENTER2
        upper_dots = sort_dots(upper_dots) + _BOARD_CENTER1
        lower_dots = sort_dots(lower_dots, reverse=True) + _BOARD_CENTER2

        return np.concatenate([upper_dots, lower_dots], 0)
        """
        return checkpoints

    def _load_board(self):
        # TODO
        # load from file
        # y, x
        checkpoints = np.array([
            [526, 342],
            [500, 222],
            [412, 130],
            [318, 102],
            [164, 145],
            [112, 224],
            [88 , 332],
            [118, 446],
            [194, 530],
            [318, 566],
            [390, 576],
            [464, 632],
            [460, 1046],
            [386, 1100],
            [314, 1118],
            [248, 1122],
            [178, 1152],
            [114, 1224],
            [88 , 1308],
            [90 , 1392],
            [122, 1470],
            [182, 1534],
            [248, 1570],
            [326, 1574],
            [408, 1544],
            [468, 1496],
            [510, 1426],
            [524, 1330],
            [526, 1186],
            [526, 982],
            [526, 734],
            [526, 532],
        ])
        checkpoints = [chp[::-1] for chp in checkpoints]
        return checkpoints