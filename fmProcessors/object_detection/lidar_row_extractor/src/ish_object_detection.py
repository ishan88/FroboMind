#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros. In this file a test node is created in Object
# Detection Module

import rospy
from std_msgs.msg import String

class ObjectDetection():
    def __init__(self):
        
        rospy.init_node('ish_object_detection', anonymous=False)
        
        
        rospy.loginfo(" Object Detection Module started")
        
        
        self.locSensTopic = rospy.get_param("~localsensingdata", '/fmInformation/locSenDat')
        
        
        rospy.Subscriber(self.locSensTopic, String, self.locSens_Topic)
        
    
    def locSens_Topic(self,msg):
        rospy.loginfo(" The message received in Object Detection from "+self.locSensTopic+" is "+msg.data)
        rospy.sleep(20)
        
        
        
        

if __name__ == '__main__':
    try:
        ObjectDetection()
        rospy.spin()
    except:
        rospy.loginfo("Node Terminated")