#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros.

import rospy
from std_msgs.msg import String

class Monitors():
    def __init__(self):
        rospy.init_node('monitorsnode')

if __name__ == '__main__':
    try:
        Monitors()
    except:
        rospy.loginfo("Monitors Node terminated")
        