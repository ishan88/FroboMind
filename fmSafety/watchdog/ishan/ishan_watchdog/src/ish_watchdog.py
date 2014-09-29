#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros.

import rospy
from std_msgs.msg import String

class Watchdog():
    def __init__(self):
        
        rospy.init_node('ish_watchdog', anonymous=False)
        
        rospy.loginfo("Watchdog Node Started")
        
        self.watchdogTopic = rospy.get_param("~watchdogdata", '/fmSafety/watDogDat')
        
        self.pub = rospy.Publisher(self.watchdogTopic,String)
        r = rospy.Rate(1)
        r.sleep()
        
        while not rospy.is_shutdown():
            str = "WatchDogData"
            self.pub.publish(str)
            r.sleep()
        
   
if __name__ == '__main__':
    try:
        Watchdog()
    except:
        rospy.loginfo("Watchdog Node Terminated")
        
        
        
        
        
        
        
        
        
        
        
        