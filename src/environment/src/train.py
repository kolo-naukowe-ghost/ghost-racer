"""
Code taken from: https://github.com/keras-rl/keras-rl/blob/master/examples/dqn_cartpole.py (with some modifications)
This code should not be used for any kind of rl problem.
"""
from PIL import Image
from rl.callbacks import ModelIntervalCheckpoint, FileLogger
from rl.core import Processor

from env.GazeboEnv import GazeboEnv

import numpy as np
from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten, Permute, Convolution2D
from keras.optimizers import Adam
import keras.backend as K

from rl.agents.dqn import DQNAgent
from rl.policy import BoltzmannQPolicy, LinearAnnealedPolicy, EpsGreedyQPolicy
from rl.memory import SequentialMemory

INPUT_SHAPE = (240, 320)
WINDOW_LENGTH = 1


def train():
    np.random.seed(123)
    env = GazeboEnv()
    env.seed(123)
    nb_actions = env.action_space.n

    input_shape =  (WINDOW_LENGTH,)+INPUT_SHAPE
    print(input_shape)

    class AtariProcessor(Processor):
        def process_observation(self, observation):
            assert observation.ndim == 3  # (height, width, channel)
            img = Image.fromarray(observation)
            img = img.convert('L')  # resize and convert to grayscale
            processed_observation =np.array(img)
            # assert processed_observation.shape == input_shape
            return processed_observation.astype('uint8')  # saves storage in experience memory

        def process_state_batch(self, batch):
            # We could perform this processing step in `process_observation`. In this case, however,
            # we would need to store a `float32` array instead, which is 4x more memory intensive than
            # an `uint8` array. This matters if we store 1M observations.
            processed_batch = batch.astype('float32') / 255.
            return processed_batch

        def process_reward(self, reward):
            return np.clip(reward, -1., 1.)

    model = Sequential()

    model.add(Convolution2D(32, (8, 8), strides=(4, 4), data_format='channels_first', input_shape=input_shape))
    model.add(Activation('relu'))
    model.add(Convolution2D(64, (4, 4), strides=(2, 2)))
    model.add(Activation('relu'))
    model.add(Convolution2D(64, (3, 3), strides=(1, 1)))
    model.add(Activation('relu'))
    model.add(Flatten())
    model.add(Dense(512))
    model.add(Activation('relu'))
    model.add(Dense(nb_actions))
    model.add(Activation('linear'))
    print(model.summary())

    # Finally, we configure and compile our agent. You can use every built-in Keras optimizer and
    # even the metrics!
    memory = SequentialMemory(limit=1000000, window_length=WINDOW_LENGTH)
    processor = AtariProcessor()

    # Select a policy. We use eps-greedy action selection, which means that a random action is selected
    # with probability eps. We anneal eps from 1.0 to 0.1 over the course of 1M steps. This is done so that
    # the agent initially explores the environment (high eps) and then gradually sticks to what it knows
    # (low eps). We also set a dedicated eps value that is used during testing. Note that we set it to 0.05
    # so that the agent still performs some random actions. This ensures that the agent cannot get stuck.
    policy = LinearAnnealedPolicy(EpsGreedyQPolicy(), attr='eps', value_max=1., value_min=.1, value_test=.05,
                                  nb_steps=1000000)

    # The trade-off between exploration and exploitation is difficult and an on-going research topic.
    # If you want, you can experiment with the parameters or use a different policy. Another popular one
    # is Boltzmann-style exploration:
    # policy = BoltzmannQPolicy(tau=1.)
    # Feel free to give it a try!

    dqn = DQNAgent(model=model, nb_actions=nb_actions, policy=policy, memory=memory,
                   processor=processor, nb_steps_warmup=50000, gamma=.99, target_model_update=10000,
                   train_interval=4, delta_clip=1.)
    dqn.compile(Adam(lr=.00025), metrics=['mae'])

    # Okay, now it's time to learn something! We capture the interrupt exception so that training
    # can be prematurely aborted. Notice that now you can use the built-in Keras callbacks!
    weights_filename = 'dqn_{}_weights.h5f'.format('GhostRacer')
    checkpoint_weights_filename = 'dqn_' + 'GhostRacer' + '_weights_{step}.h5f'
    log_filename = 'dqn_{}_log.json'.format('GhostRacer')
    callbacks = [ModelIntervalCheckpoint(checkpoint_weights_filename, interval=100)]
    callbacks += [FileLogger(log_filename, interval=100)]
    dqn.fit(env, callbacks=callbacks, nb_steps=1000, log_interval=100)

    # After training is done, we save the final weights one more time.
    dqn.save_weights(weights_filename, overwrite=True)

    # Finally, evaluate our algorithm for 10 episodes.
    dqn.test(env, nb_episodes=10, visualize=False)
