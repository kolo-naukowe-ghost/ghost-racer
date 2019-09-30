#!/usr/bin/env python

from env.GazeboEnv import GazeboEnv
from train import train


def main():
    env = GazeboEnv()
    for _ in range(100000):
        observation, reward, done, info = env.step(6)


if __name__ == "__main__":
    # main()
    train(test=True)
