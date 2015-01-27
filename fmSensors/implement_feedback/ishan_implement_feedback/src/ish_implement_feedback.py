#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros. In this file a test node is created in Local
# Sensing Module.

import rospy
from std_msgs.msg import String

class ImplementFeedback():
    def __init__(self):
        
        rospy.init_node('ish_implement_feedback', anonymous=False)
        
        r = rospy.Rate(1)
        
        rospy.on_shutdown(self.shutdown)
        
        rospy.loginfo(" Implement Feedback Module Started ")
        
        self.impFeedTopic = rospy.get_param("implementfeedbackdata", '/fmInformation/impFeeDat')
        
        self.pub = rospy.Publisher(self.impFeedTopic, String)
    
        while not rospy.is_shutdown():
            str = "ImplementFeedbackData"
            self.pub.publish(str)
            r.sleep()
    
    def shutdown(self):
        
        rospy.loginfo("Shutting Down")
        rospy.sleep(1)       

if __name__ == '__main__':
    try:
        ImplementFeedback()
    except:
        rospy.loginfo("Implement Feedback node terminated")
        