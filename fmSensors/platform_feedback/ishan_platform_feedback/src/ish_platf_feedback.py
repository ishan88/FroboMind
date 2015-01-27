#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros. In this file a test node is created in Local
# Sensing Module.

import rospy
from std_msgs.msg import String

class PlatformFeedback():
    def __init__(self):
        
        rospy.init_node('ish_platf_feedback', anonymous=False)
        
        r = rospy.Rate(1)
        
        rospy.on_shutdown(self.shutdown)
        
        rospy.loginfo(" Platform Feedback Module Started ")
        
        self.platfFeedTopic = rospy.get_param("platformfeedbackdata", '/fmInformation/plaFeeDat')
        
        self.pub = rospy.Publisher(self.platfFeedTopic, String)
    
        while not rospy.is_shutdown():
            str = "PlatformFeedbackData"
            self.pub.publish(str)
            r.sleep()
    
    def shutdown(self):
        
        rospy.loginfo("Shutting Down")
        rospy.sleep(1)       

if __name__ == '__main__':
    try:
        PlatformFeedback()
    except:
        rospy.loginfo("Platform Feedback node terminated")
        