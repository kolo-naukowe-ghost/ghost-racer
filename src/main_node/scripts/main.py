#!/usr/bin/env python
import rospy
import subprocess
import signal
import sys
import time
import os

def quit_ros():
    print('Quitting ghost-car')
    subprocess.call(["pkill", "gzclient"])
    subprocess.call(["pkill", "gzserver"])
    time.sleep(1)
    subprocess.call(["rosnode", "kill", "-a"])
    time.sleep(1)
    subprocess.call(["pkill", "xterm"])
    exit(0)

def signal_handler(sig, frame):
    quit_ros()

def action():
    try:
        key = raw_input("type q or press ctrl+c to exit ")
    except Exception, e:
        return
    if key is None:
        return
    if key == 'q':
        quit_ros()

def main():
    rospy.init_node('mainNode', anonymous=False)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        action()
        rate.sleep()


if __name__=="__main__":
    try:
        signal.signal(signal.SIGINT, signal_handler)
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("caught a ros exception")