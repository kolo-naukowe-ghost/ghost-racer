#!/usr/bin/env python

from env.GazeboEnv import GazeboEnv

env = GazeboEnv()
observation, reward, done, info = env.step(0)
print(observation)
