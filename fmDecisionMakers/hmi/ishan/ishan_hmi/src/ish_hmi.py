#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros.

import rospy
from std_msgs.msg import String


class HMI():
    def __init__(self):
        
        rospy.init_node('ish_hmi', anonymous=False)
        
        rospy.on_shutdown(self.shutdown)
        
        rospy.loginfo("HMI Node Started")

        self.hmiTopic = rospy.get_param("~hmidata", '/fmDecisions/hmiDat')
        
        
        r = rospy.Rate(1)
        self.pub = rospy.Publisher(self.hmiTopic, String)
        
        while not rospy.is_shutdown():
            str = "HmiData"
            self.pub.publish(str)
            r.sleep()
   
    def shutdown(self):
        
        rospy.loginfo("Shutting Down")
        rospy.sleep(1)     
        
if __name__ == '__main__':
    try:
        HMI()
    except:
        rospy.loginfo("HMI Node Terminated")