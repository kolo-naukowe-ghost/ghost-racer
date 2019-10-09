from PIL import Image
import numpy as np
import os
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

class BoardPath:

    def __init__(self):
        checkpoints = self._load_board()
        checkpoints = self._order_checkpoints(checkpoints)
        self.dots = checkpoints
        self.current_checkpoint_index = 0

        self.car_size = 100 #hardcoded for debugging

    def forward_checkpoint(self):
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
            self.forward_checkpoint()

    def update(self, relative_car_x, relative_car_y):
        self.car_position = np.array([relative_car_x, relative_car_y])
        self._update()

    def angle_to_next_checkpoint(self):
        direction = self.current_checkpoint - self.car_position
        return _get_angle_between_vector_and_x_axis(direction)


    def distance_to_next_checkpoint(self):
        v = self.current_checkpoint - self.car_position
        return np.sqrt(v.dot(v))

    def distance_to_road(self):
        v = self.current_checkpoint - self.last_checkpoint
        if np.isclose(v[0], 0.0):
            return abs(self.current_checkpoint[0] - self.car_position[0])
        a = v[1] / v[0]
        b = self.current_checkpoint[1] - a * self.current_checkpoint[0]
        #aX - y + b = 0
        nominator = abs(a * self.car_position[0] - 1 + b)
        denominator = np.sqrt(a**2 + 1)
        return nominator / denominator

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