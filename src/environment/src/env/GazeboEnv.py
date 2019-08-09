from geometry_msgs.msg import Twist
from gym import spaces

from env.Env import Env
from env.GazeboMixin import GazeboMixin

import numpy as np


class GazeboEnv(Env, GazeboMixin):
    def __init__(self):
        super(GazeboEnv, self).__init__()
        metadata = {'render.modes': ['human', 'rgb_array']}
        self.action_space = spaces.Discrete(9)
        self.observation = np.random.randint(0, 10, (5, 5))

    def step(self, action):
        info = dict()

        message: Twist = self._get_message_from_action()

        self.unpause_gazebo()
        self.publish_gazebo(message)
        self.pause_gazebo()

        observation = self._get_observation()
        reward = self._calculate_reward()

        done = False  # TODO?

        observation, reward, done, info

    def reset(self):
        self.reset_gazebo()
        self.unpause_gazebo()
        self._get_observation()
        # TODO set state, and reward here
        self.pause_gazebo()

    def render(self, mode='human'):
        if mode == 'rgb_array':
            return np.array(self.observation)
        elif mode == 'human':
            # TODO some window?
            return None
        else:
            super(GazeboEnv, self).render(mode=mode)  # just raise an exception

    def _get_observation(self):
        # TODO get data from gazebo
        pass

    def _calculate_reward(self):
        return 0

    def _get_message_from_action(self) -> Twist:
        # TODO action -> twist
        return Twist()
