#!/usr/bin/env python

from env.GazeboEnv import GazeboEnv

env = GazeboEnv()
for _ in range(1):
    observation, reward, done, info = env.step(6)
print(observation)
