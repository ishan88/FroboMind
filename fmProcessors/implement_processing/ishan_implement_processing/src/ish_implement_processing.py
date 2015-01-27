#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros. In this file a test node is created in Local
# Sensing Module.

import rospy
from std_msgs.msg import String

class ImplementProcessing():
    def __init__(self):
        
        rospy.init_node('ish_implement_processing', anonymous=False)
        
        r = rospy.Rate(1)
        
        rospy.on_shutdown(self.shutdown)
        
        rospy.loginfo(" Implement Processing Module Started ")
        
        self.impProcessTopic = rospy.get_param("implementprocessingdata", '/fmKnowledge/impProDat')
        self.impFeedTopic = rospy.get_param("implementfeedbackdata" ,'/fmInformation/impFeeDat')
        
        self.pub = rospy.Publisher(self.impProcessTopic, String)
        rospy.sleep(1)
        
        rospy.Subscriber(self.impFeedTopic, String, self.on_impFeedTopic)
        
        while not rospy.is_shutdown():
            str = "ImplementProcessingData"
            self.pub.publish(str)
            r.sleep()
    
    def on_impFeedTopic(self, msg):
        rospy.loginfo("The message received in Implement Processing from "+self.impFeedTopic+
                      " is "+msg.data)
        rospy.sleep(20)
    
    def shutdown(self):
        
        rospy.loginfo("Shutting Down")
        rospy.sleep(1)       

if __name__ == '__main__':
    try:
        ImplementProcessing()
        rospy.spin()
    except:
        rospy.loginfo("Implement Processing node terminated")
        