#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros. In this file a test node is created in Local
# Sensing Module.

import rospy
from std_msgs.msg import String

class LocalizationMapping():
    def __init__(self):
        
        rospy.init_node('ish_localization_mapping', anonymous=False)
        
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Localization Mapping Node Started")
         
        r = rospy.Rate(1)
        
        self.globalSensTopic=rospy.get_param("~globalsensingdata", '/fmInformation/gloSenDat')
        self.localSensTopic= rospy.get_param("~localsensingdata", '/fmInformation/locSenDat')
        self.objDetecTopic = rospy.get_param("~objectdetectiondata", '/fmKnowledge/objDetDat')
        self.localMapTopic = rospy.get_param("~localizationmappingdata", '/fmKnowledge/locMapDat')
        self.platFeedTopic = rospy.get_param("~platformfeedbackdata", '/fmInformation/plaFeeDat')
        
        self.pub = rospy.Publisher(self.localMapTopic, String)
        rospy.sleep(1)
        
        rospy.Subscriber(self.globalSensTopic, String, self.on_globalSensTopic)
        self.sub = rospy.Subscriber(self.localSensTopic, String, self.on_localSensTopic)
        self.noOfConnections = self.sub.get_num_connections()
        
        rospy.Subscriber(self.objDetecTopic, String, self.on_objDetecTopic)
        
        rospy.Subscriber(self.platFeedTopic, String, self.on_platFeedTopic)
        
        while not rospy.is_shutdown():
            str = "LocalizationMappingModule Data"
            self.pub.publish(str)
            r.sleep()
        
        
    def on_globalSensTopic(self, msg):
        rospy.loginfo("The message received in Localization Mapping from "+self.globalSensTopic+
                      " is "+msg.data)
        
        rospy.sleep(20)
        
    def on_localSensTopic(self, msg):
        rospy.loginfo(" The message received in localization Mapping from "+self.localSensTopic+
                      " is "+msg.data)
        rospy.sleep(20)
        
    def on_objDetecTopic(self, msg):
        rospy.loginfo(" The message received in Object Detection from "+self.objDetecTopic+
                      " is "+msg.data)
        rospy.sleep(20)   
        
    def on_platFeedTopic(self, msg):
        rospy.loginfo(" The message received in Object Detection from "+self.platFeedTopic+
                      " is "+msg.data)
        rospy.sleep(20)  
        
    def shutdown(self):
        
        rospy.loginfo("Shutting Down")
        rospy.sleep(1)   

if __name__ == '__main__':
    try:
        LocalizationMapping()
        rospy.spin()
    except:
        rospy.loginfo(" Localization Mapping Node Terminated")