import numpy as np
import os
import rospy
from utils.math_helpers import _get_angle_between_vector_and_x_axis, get_straight_from_points, \
    angle_between_two_straight, point_to_straight_distance, normalized

_PATH_TO_IMAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "dotted_board.jpeg")

# _BOARD_CENTER = np.array([1050, 2040]).T
_BOARD_CENTER1 = np.array([500, 460][::-1])
_BOARD_CENTER2 = np.array([500, 1170][::-1])
_MIDDLE_BOARD_POINT = 840


def sort_dots(dots, reverse=False):
    angles = [_get_angle_between_vector_and_x_axis(dot) for dot in dots]
    angles_and_dots = [(angle, dot) for angle, dot in zip(angles, dots)]
    print("sorting...")
    angles_and_dots.sort(reverse=reverse)
    print("after sort")
    print(angles_and_dots)
    return np.array([dot for _, dot in angles_and_dots])


class BoardPath:

    def __init__(self):
        checkpoints = self._load_board()
        checkpoints = self._order_checkpoints(checkpoints)
        self.dots = checkpoints
        self.current_checkpoint_index = 0
        self._last_car_position = np.zeros((2,))
        self.car_position = self._last_car_position
        self.car_direction = np.zeros((2,))
        self.current_road_straight_line = get_straight_from_points(self.current_checkpoint, self.next_checkpoint)
        self.car_size = 100  # hardcoded for debugging

    def _forward_checkpoint(self):
        self.current_checkpoint_index = (self.current_checkpoint_index + 1) % len(self.dots)
        self.current_road_straight_line = get_straight_from_points(self.current_checkpoint, self.next_checkpoint)

    @property
    def next_checkpoint(self):
        return self.dots[(self.current_checkpoint_index + 1) % len(self.dots)]

    @property
    def current_checkpoint(self):
        return self.dots[self.current_checkpoint_index]

    @property
    def last_checkpoint(self):
        return self.dots[self.current_checkpoint_index - 1]

    @property
    def car_direction_straight(self):
        return get_straight_from_points(self.car_position, self.car_front_point)

    @property
    def car_front_point(self):
        return int(self.car_position[0] + 50 * self.car_direction[0]), int(
            self.car_position[1] + 50 * self.car_direction[1])

    @property
    def car_velocity(self):
        if self._car_vel is None:
            return 0
        else:
            return np.sqrt(self._car_vel.dot(self._car_vel))

    @property
    def angle_to_road(self):
        return angle_between_two_straight(self.current_road_straight_line, self.car_direction_straight)

    @property
    def angle_to_road_rad(self):
        '''
        returns angle to road in radians
        '''
        return np.deg2rad(self.angle_to_road)

    def _update(self):
        v = self.current_checkpoint - self.car_position
        distance_squared = v.dot(v)
        if distance_squared <= self.car_size ** 2:
            self._forward_checkpoint()

    def update(self, relative_car_x, relative_car_y):
        car_position = np.array([relative_car_x, relative_car_y])
        if np.allclose(self.car_position, car_position):
            return #there is nothing to update
        self.car_position = np.array([relative_car_x, relative_car_y])
        if self._last_car_position is None:
            self._last_car_position = self.car_position
        self._car_vel = self.car_position - self._last_car_position
        vector = normalized(self._car_vel)
        if np.any(vector):
            self.car_direction = vector
        self._last_car_position = self.car_position
        self._update()

    def get_distance_to_next_checkpoint(self):
        v = self.current_checkpoint - self.car_position
        return np.sqrt(v.dot(v))

    def get_distance_to_road(self):
        a, b, c = get_straight_from_points(self.current_checkpoint, self.last_checkpoint)

        if b == 0:  # parallel to OX
            return abs(self.current_checkpoint[0] - self.car_position[0])
        if a == 0:  # parallel to OY
            return abs(self.current_checkpoint[1] - self.car_position[1])

        tmp = point_to_straight_distance((a, b, c), self.car_position)
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
            [526, 432],
            [526, 342],
            [500, 222],
            [412, 130],
            [318, 102],
            [164, 145],
            [112, 224],
            [88, 332],
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
            [88, 1308],
            [90, 1392],
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
        # change to x, y
        # checkpoints = [chp[::-1] for chp in checkpoints]
        return checkpoints
