from geometry_msgs.msg import Twist
from gym import spaces

from env.Env import Env
from env.GazeboMixin import GazeboMixin

import numpy as np

from env.State import State


class GazeboEnv(Env, GazeboMixin):
    LEFT_CAMERA_TOPIC = '/conde_camera_tracking_left/image_raw'
    CENTER_CAMERA_TOPIC = '/conde_camera_signalling_panel/image_raw'
    RIGHT_CAMERA_TOPIC = '/conde_camera_tracking_right/image_raw'

    LINEAR_ACTION = 0.3
    TWIST_ACTION = 0.2

    def __init__(self):
        super(GazeboEnv, self).__init__()
        metadata = {'render.modes': ['human', 'rgb_array']}
        self.action_space = spaces.Discrete(9)
        self.observation = np.random.randint(0, 10, (5, 5))

    def step(self, action: int):
        info = dict()

        message: Twist = self._get_message_from_action()

        self._unpause_gazebo()
        self._publish_gazebo(message)
        self._pause_gazebo()

        observation = self._get_observation()
        reward = self._calculate_reward()

        done = False  # TODO?

        observation, reward, done, info

    def reset(self):
        self._reset_gazebo()
        self._unpause_gazebo()
        self._get_observation()
        # TODO set state, and reward here
        self._pause_gazebo()

    def render(self, mode='human'):
        if mode == 'rgb_array':
            return np.array(self.observation)
        elif mode == 'human':
            # TODO some window?
            return None
        else:
            super(GazeboEnv, self).render(mode=mode)  # just raise an exception

    def _get_observation(self) -> State:
        left_image = self._get_image_data_from_topic(self.LEFT_CAMERA_TOPIC)
        center_image = self._get_image_data_from_topic(self.CENTER_CAMERA_TOPIC)
        right_image = self._get_image_data_from_topic(self.RIGHT_CAMERA_TOPIC)

        state = State(left_image, center_image, right_image)
        return state

    def _calculate_reward(self):
        car_x, car_y = self._get_car_position()

        return 0

    def _get_message_from_action(self, action: int) -> Twist:
        twist = Twist()
        twist.linear = 0
        twist.angular = 0
        if action == 0:  # F
            twist.linear = self.LINEAR_ACTION
        elif action == 1:  # L
            twist.angular = -self.TWIST_ACTION
        elif action == 2:  # B
            twist.linear = -self.LINEAR_ACTION
        elif action == 3:  # R
            twist.angular = self.TWIST_ACTION
        elif action == 4:  # FL
            twist.linear = self.LINEAR_ACTION
            twist.angular = -self.TWIST_ACTION
        elif action == 5:  # FR
            twist.linear = self.LINEAR_ACTION
            twist.angular = self.TWIST_ACTION
        elif action == 6:  # BL
            twist.linear = -self.LINEAR_ACTION
            twist.angular = -self.TWIST_ACTION
        elif action == 7:  # BR
            twist.linear = -self.LINEAR_ACTION
            twist.angular = self.TWIST_ACTION
        else:
            raise ValueError

        return twist

    def _get_car_position(self):
        postion = self._get_model_states()

        # TODO load, scale image - based on param downlaoded from gazebo and calculate threshold
        if postion is not None:
            return postion.x, postion.y
        return None
