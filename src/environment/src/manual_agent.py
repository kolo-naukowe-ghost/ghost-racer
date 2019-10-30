#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013 PAL Robotics SL.
# Released under the BSD License.
#
# Authors:
#   * Siegfried-A. Gevatter
# ROS wiki: http://wiki.ros.org/key_teleop
# GitHub repo: https://github.com/ros-teleop/teleop_tools
import calendar
import curses
import math
import time
from copy import deepcopy

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

from env.GazeboEnv import GazeboEnv

class Velocity(object):

    def __init__(self, min_velocity, max_velocity, num_steps):
        assert min_velocity > 0 and max_velocity > 0 and num_steps > 0
        self._min = min_velocity
        self._max = max_velocity
        self._num_steps = num_steps
        if self._num_steps > 1:
            self._step_incr = (max_velocity - min_velocity) / (self._num_steps - 1)
        else:
            # If num_steps is one, we always use the minimum velocity.
            self._step_incr = 0

    def __call__(self, value, step):
        """
        Takes a value in the range [0, 1] and the step and returns the
        velocity (usually m/s or rad/s).
        """
        if step == 0:
            return 0

        assert step > 0 and step <= self._num_steps
        max_value = self._min + self._step_incr * (step - 1)
        return value * max_value


class TextWindow():
    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError('lineno out of bounds')
        height, width = self._screen.getmaxyx()
        y = (height / self._num_lines) * lineno
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(int(y), int(x), text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()




class SimpleKeyTeleop():
    def __init__(self, interface, env):
        self._env = env
        self._interface = interface
        self._pub_cmd = rospy.Publisher('cmd_vel', Twist)

        self._hz = rospy.get_param('~hz', 50)

        self._forward_rate = rospy.get_param('~forward_rate', 0.3)
        self._backward_rate = rospy.get_param('~backward_rate', 0.25)
        self._rotation_rate = rospy.get_param('~rotation_rate', 0.25)
        self._last_pressed = {}
        self._angular = 0
        self._linear = 0
        self._skip_frames = 0
        self._images = list()
        self._angles = list()
        self._record = False

        self._current_camera_image = None

    movement_bindings = {
        curses.KEY_UP: (1, 0),
        curses.KEY_DOWN: (-1, 0),
        curses.KEY_LEFT: (0, 1),
        curses.KEY_RIGHT: (0, -1),
    }

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)
            self._set_velocity()
            self._set_step()
            self._publish()
            rate.sleep()

    def _get_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _set_velocity(self):
        now = rospy.get_time()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < 0.5:
                keys.append(a)
        linear = 0.0
        angular = 0.0
        for k in keys:
            l, a = self.movement_bindings[k]
            linear += l
            angular += a
        if linear > 0:
            linear = linear * self._forward_rate
        else:
            linear = linear * self._backward_rate
        angular = angular * self._rotation_rate
        self._angular = angular
        self._linear = linear

    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            self._running = False
            rospy.signal_shutdown('Bye')
        elif keycode == ord('r'):
            self._env.reset()
        elif keycode in self.movement_bindings:
            self._last_pressed[keycode] = rospy.get_time()

    def _set_step(self):
        actions = {
            "F" : 0,
            "L" : 1,
            "B" : 2,
            "R" : 3,
            "FL" : 4,
            "FR" : 5,
            "BL" : 6,
            "BR" : 7,
            "" : 8,
        }
        lin = ""
        ang = ""
        if self._linear > 0:
            lin = "F"
        elif self._linear < 0:
            lin = "B"
        if self._angular > 0:
            ang = "R"
        elif self._angular < 0:
            ang = "L"
        self._step = actions[lin+ang] 

    def _publish(self):
        observation, reward, done, info = self._env.step(self._step)
        self._env.render()
        
        self._interface.clear()
        for i, (key, val) in enumerate(info.items()):
            if key == 'angle':
                self._interface.write_line(i + 2, "{}: {:9.4f} ({:3d}°)".format(key, val, int(val * 180 / 3.14)))
            else:
                self._interface.write_line(i + 2, "{}: {:9.4f}".format(key, val))

        self._interface.write_line(7, 'Use arrow keys to move, r to restart, q to exit.')
        self._interface.refresh()
    


def main(stdscr):
    app = SimpleKeyTeleop(TextWindow(stdscr), GazeboEnv())
    app.run()


if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
