#!/usr/bin/env python

from env.GazeboEnv import GazeboEnv

env = GazeboEnv()
for _ in range(10000):
    observation, reward, done, info = env.step(6)
print(observation)
