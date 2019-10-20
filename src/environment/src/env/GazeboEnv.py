from geometry_msgs.msg import Twist
from gym import spaces

import numpy as np

from env.Env import Env
from env.GazeboMixin import GazeboMixin

import rospy

import os
import cv2

from image_processing.ros_image import RosImage

from BoardPath import BoardPath


class GazeboEnv(Env, GazeboMixin):
    # TODO It should not be hardcoded
    BOARD_WIDTH = 6.95
    BOARD_HEIGHT = 16.7

    LEFT_CAMERA_TOPIC = '/conde_camera_tracking_left/image_raw'
    CENTER_CAMERA_TOPIC = '/conde_camera_signalling_panel/image_raw'
    RIGHT_CAMERA_TOPIC = '/conde_camera_tracking_right/image_raw'

    LINEAR_ACTION = 0.3
    TWIST_ACTION = 0.25

    BOARD_IMAGE_PATH = 'data/board.jpeg'

    def __init__(self):
        self.show_images = True
        super(GazeboEnv, self).__init__()
        metadata = {'render.modes': ['human', 'rgb_array']}
        self.cwd = os.path.abspath(os.path.dirname(__file__))

        self.current_position = None

        self.position_window_name = "ghost's position"
        self.center_image_window_name = "ghost's view"
        self.renderers_initialized = False

        self.action_space = spaces.Discrete(8)
        self.observation_space = spaces.Box(0, 255, [240, 320, 3])  # size of image retrieved from center camera
        self.dump_board_image = True

        self.board = self._load_board()
        self.current_board = None
        self.white_indices = np.argwhere(self.board == 255)

        self.current_car_position = (0, 0)

        # add center camera image
        self.center_image = RosImage(GazeboEnv.CENTER_CAMERA_TOPIC)

        self.board_path = BoardPath()
        self.reset()

        self.observation = None

    def init_renderers(self):
        self.renderers_initialized = True
        flags = 2
        cv2.namedWindow(self.center_image_window_name, flags)
        cv2.namedWindow(self.position_window_name, flags)
        cv2.resizeWindow(self.position_window_name, 500, 1000)

    def center_image_callback(self, raw_image):
        pass

    def step(self, action):
        """

        :param action: int
        :return: observation: State, reward: float, done: bool, info: dict
        """
        info = dict()
        message = self._get_message_from_action(action)

        self._publish_gazebo(message)
        self._ros_sleep()

        self.observation = self._get_observation()

        reward = self._calculate_reward()

        done = 1 / reward > 150  # distance greater than 150 units  # TODO


        return self.observation, reward, done, info

    def reset(self):
        self._reset_gazebo()
        self._get_observation()
        # TODO set state, and reward here

        return self._get_observation()

    def render(self, mode='human'):
        if mode == 'rgb_array':
            return np.array(self.observation)
        elif mode == 'human':
            self.current_board = self.board.copy()

            if not self.renderers_initialized:
                self.init_renderers()

            if RosImage.is_image_valid(self.center_image.image):
                cv2.imshow(self.center_image_window_name, self.center_image.image)

            if RosImage.is_image_valid(self.current_board):
                self._print_car_position_on_board()
                self.draw_cars_direction()
                self.draw_waypoints()
                cv2.imshow(self.position_window_name, self.current_board)
            cv2.waitKey(1)
            return None
        else:
            super(GazeboEnv, self).render(mode=mode)  # just raise an exception

    def _get_observation(self):
        """
        :return: State
        """

        return self.center_image.image

    def _calculate_reward(self):
        car_x, car_y = self._get_car_position()
        relative_car_x = (self.BOARD_HEIGHT / 2 - car_y) * 100
        relative_car_y = (car_x + self.BOARD_WIDTH / 2) * 100
        self.board_path.update(relative_car_x, relative_car_y)

        relative_car_x = int(relative_car_x)
        relative_car_y = int(relative_car_y)

        _, distance = self._get_closest_point_on_board(relative_car_x, relative_car_y)
        self.current_car_position = (relative_car_y, relative_car_x)
        # TODO more sophisticated reward function (?)
        reward = 1 / (distance + 0.001)

        return reward

    def _get_closest_point_on_board(self, relative_car_x, relative_car_y):
        """

        :param relative_car_x: int, relative_car_y: int
        :return: closest_point_index: np.array, distance: float
        """
        return self.board_path.current_checkpoint, self.board_path.distance_to_road()

    def _get_message_from_action(self, action):
        """

        :param action: int
        :return: Twist
        """
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        if action == 0:  # F
            twist.linear.x = self.LINEAR_ACTION
        elif action == 1:  # L
            twist.angular.z = -self.TWIST_ACTION
        elif action == 2:  # B
            twist.linear.x = -self.LINEAR_ACTION
        elif action == 3:  # R
            twist.angular.z = self.TWIST_ACTION
        elif action == 4:  # FL
            twist.linear.x = self.LINEAR_ACTION
            twist.angular.z = -self.TWIST_ACTION
        elif action == 5:  # FR
            twist.linear.x = self.LINEAR_ACTION
            twist.angular.z = self.TWIST_ACTION
        elif action == 6:  # BL
            twist.linear.x = -self.LINEAR_ACTION
            twist.angular.z = -self.TWIST_ACTION
        elif action == 7:  # BR
            twist.linear.x = -self.LINEAR_ACTION
            twist.angular.z = self.TWIST_ACTION
        else:
            raise ValueError

        return twist

    def _get_car_position(self):
        model_state = self._get_model_state('conde')
        position = (0, 0)
        if model_state is not None:
            position = (model_state.pose.position.x, model_state.pose.position.y)
        return position

    def _load_board(self):
        """
        This function loads image, resize to BOARD_WIDTH x BOARD_HEIGHT
        and after that apply threshold, scale values to 0, 255 and return
        board image as numpy array with uint8 0,255 values
        :return:
        """
        board_location = os.path.join(self.cwd, GazeboEnv.BOARD_IMAGE_PATH)
        if not os.path.exists(board_location):
            rospy.logerr('Path with image {} , doesn\'t exist.'.format(board_location))
            return None
        board = cv2.imread(board_location, cv2.IMREAD_GRAYSCALE)
        width, height = int(self.BOARD_WIDTH * 100), int(self.BOARD_HEIGHT * 100)
        board = cv2.resize(board, (width, height))
        board[board < 50] = 0
        board[board >= 50] = 255
        return board

    def _print_car_position_on_board(self):
        distance = 15
        self.current_board[self.current_car_position[1] - distance:self.current_car_position[1] + distance,
            self.current_car_position[0] - distance:self.current_car_position[0] + distance] = 255

    def draw_waypoints(self):
        # draw checkpoints
        if self.board_path.current_checkpoint is not None and self.board_path.next_checkpoint is not None:
            cv2.circle(self.current_board, tuple(self.board_path.current_checkpoint[::-1]), 25, 127, -1)
            cv2.circle(self.current_board, tuple(self.board_path.next_checkpoint[::-1]), 20, 200, -1)

    def draw_cars_direction(self):
        # draw direction
        if self.board_path.car_direction is not None:
            cv2.arrowedLine(self.current_board,
                            self.current_car_position,
                            (self.current_car_position[0] + int(self.board_path.car_direction[1] * 50),
                             self.current_car_position[1] + int(self.board_path.car_direction[0] * 50)),
                            127, 2)
