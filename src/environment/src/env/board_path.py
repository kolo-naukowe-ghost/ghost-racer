from PIL import Image
import numpy as np
import os
import rospy

_PATH_TO_IMAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "dotted_board.jpeg")

_BOARD_CENTER = np.array([1050, 2040]).T

def _get_angle_between_vector_and_x_axis(vec):
    angle = np.arctan2(vec[1], vec[0])
    if angle < 0:
        angle = 2 * np.pi + angle
    return angle

class BoardPath:

    def __init__(self):
        pixels = self._load_board()
        print(np.max(pixels), np.min(pixels))
        dots = np.argwhere(pixels == 255)
        dots = dots - _BOARD_CENTER
        angles = [_get_angle_between_vector_and_x_axis(dot) for dot in dots]
        dots = dots + _BOARD_CENTER

        angles_and_dots = [(angle, dot) for angle, dot in zip(angles, dots)]
        print("sorting...")
        angles_and_dots.sort()
        print("after sort")
        self.dots = [dot for _, dot in angles_and_dots]
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

    def _load_board(self):
        """
        This function loads image, resize to BOARD_WIDTH x BOARD_HEIGHT
        and after that apply threshold, scale values to 0, 255 and return
        board image as numpy array with uint8 0,255 values
        :return:
        """
        BOARD_WIDTH = 6.95
        BOARD_HEIGHT = 16.7
        board_location = _PATH_TO_IMAGE
        if not os.path.exists(board_location):
            rospy.logerr('Path with image {} , doesn\'t exist.'.format(board_location))
            return None
        board_image = Image.open(board_location).convert('L')
        width, height = int(BOARD_WIDTH * 100), int(BOARD_HEIGHT * 100)
        board_image = board_image.resize((width, height))
        board_image = board_image.point(lambda p: p > 50)
        image_array = np.array(board_image) * 255
        return image_array