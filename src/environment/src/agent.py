#!/usr/bin/env python

from env.GazeboEnv import GazeboEnv
from train import train
import rospy
import argparse

"""
    Remember to install missing dependencies with "rosdep install environment"
"""

parser = argparse.ArgumentParser()
parser.add_argument('--mode', choices=['train', 'test', 'manual_control'], default='train')
parser.add_argument('--env-name', type=str, default='GhostRacer')
parser.add_argument('--weights', type=str, default=None)
args = parser.parse_args()

def main():
    env = GazeboEnv()
    while True:
        observation, reward, done, info = env.step(1)
    print(observation)


if __name__ == "__main__":
    try:
        train(args)
    except rospy.ROSInterruptException:
        print("Agent has been closed.")
