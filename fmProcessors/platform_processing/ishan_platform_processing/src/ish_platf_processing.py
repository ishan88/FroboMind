#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros. In this file a test node is created in Local
# Sensing Module.

import rospy
from std_msgs.msg import String

class PlatformProcessing():
    def __init__(self):
        
        rospy.init_node('ish_platf_processing', anonymous=False)
        
        r = rospy.Rate(1)
        
        rospy.on_shutdown(self.shutdown)
        
        rospy.loginfo(" Platform Processing Module Started ")
        
        self.platfProcessTopic = rospy.get_param("platformprocessingdata", '/fmKnowledge/plaProDat')
        self.platfFeedTopic = rospy.get_param("platformfeedbackdata" ,'/fmInformation/plaFeeDat')
        
        self.pub = rospy.Publisher(self.platfProcessTopic, String)
        rospy.sleep(1)
        
        rospy.Subscriber(self.platfFeedTopic, String, self.on_platfFeedTopic)
        
        while not rospy.is_shutdown():
            str = "ImplementProcessingData"
            self.pub.publish(str)
            r.sleep()
    
    def on_platfFeedTopic(self, msg):
        rospy.loginfo("The message received in Platform Processing from "+self.platfFeedTopic+
                      " is "+msg.data)
        rospy.sleep(20)
    
    def shutdown(self):
        
        rospy.loginfo("Shutting Down")
        rospy.sleep(1)       

if __name__ == '__main__':
    try:
        PlatformProcessing()
        rospy.spin()
    except:
        rospy.loginfo("Platform Processing node terminated")
        