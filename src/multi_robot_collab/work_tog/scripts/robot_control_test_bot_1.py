#!/usr/bin/env python

import rospy
from robot_control_bot_1 import RobotControl
from VisionPlanner_bot_1 import PathPlanner

import time

robotcontrol = RobotControl()
vs = PathPlanner()


