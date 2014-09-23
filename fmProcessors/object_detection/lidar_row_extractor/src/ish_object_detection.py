#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros. In this file a test node is created in Object
# Detection Module

import rospy
from std_msgs.msg import String

class ObjectDetection():
    def __init__(self):
        
        rospy.init_node('ish_object_detection', anonymous=False)
        
        
        rospy.loginfo(" Object Detection Module started")
        
        r = rospy.Rate(1)

        self.locSensTopic = rospy.get_param("~localsensingdata", '/fmInformation/locSenDat')
        self.objDetecTopic = rospy.get_param("~objectdetectiondata", '/fmKnowledge/objDetDat')
        self.locMapTopic = rospy.get_param("~localizationmappingdata", '/fmKnowledge/locMapDat')
        
        
        self.pub = rospy.Publisher(self.objDetecTopic,String)
        rospy.sleep(1)
        
        rospy.Subscriber(self.locSensTopic, String, self.on_locSensTopic)
        rospy.Subscriber(self.locMapTopic, String, self.on_locMapTopic)
        
        
        while not rospy.is_shutdown():
            str = "ObjectDetectionModule Data"
            self.pub.publish(str)
            r.sleep()
    
    def on_locSensTopic(self,msg):
        rospy.loginfo(" The message received in Object Detection from "+self.locSensTopic+" is "+msg.data)
        rospy.sleep(20)
        
    def on_locMapTopic(self,msg):
        rospy.loginfo(" The message received in Object Detection from "+self.locMapTopic+" is "+msg.data)
        rospy.sleep(20)    
        
        

if __name__ == '__main__':
    try:
        ObjectDetection()
        rospy.spin()
    except:
        rospy.loginfo("Node Terminated")