#!/usr/bin/env python

from env.GazeboEnv import GazeboEnv

env = GazeboEnv()
print(env.render(mode='rgb_array'))
