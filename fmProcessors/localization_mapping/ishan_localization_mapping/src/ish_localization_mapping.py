#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros. In this file a test node is created in Local
# Sensing Module.

import rospy
from std_msgs.msg import String

class LocalizationMapping():
    def __init__(self):
        
        rospy.init_node('ish_localization_mapping', anonymous=False)
        
        
        rospy.loginfo("Localization Mapping Node Started")
        
        
        self.globalSensTopic = rospy.get_param("~globalsensingdata", '/fmInformation/gloSenDat')
        self.localSensTopic = rospy.get_param("~localsensingdata", '/fmInformation/locSenDat')
        rospy.Subscriber(self.globalSensTopic, String, self.on_globalSensTopic)
        self.sub = rospy.Subscriber(self.localSensTopic, String, self.on_localSensTopic)
        self.noOfConnections = self.sub.get_num_connections()
        
    def on_globalSensTopic(self, msg):
        rospy.loginfo("The message received in localization Mapping from "+self.globalSensTopic+
                      " is "+msg.data)
        rospy.sleep(20)
        
    def on_localSensTopic(self, msg):
        rospy.loginfo(" The message received in localization Mapping from "+self.localSensTopic+
                      " is "+msg.data)
        rospy.sleep(20)

if __name__ == '__main__':
    #try:
        LocalizationMapping()
        rospy.spin()
    #except:
        rospy.loginfo(" Localization Mapping Node Terminated")