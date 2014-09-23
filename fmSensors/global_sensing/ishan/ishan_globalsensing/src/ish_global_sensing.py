#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros. In this file a test node is created in Local
# Sensing Module.

import rospy
from std_msgs.msg import String

class GlobalSensing():
    def __init__(self):
        
        
        rospy.init_node('ish_global_sensing', anonymous=False)
        
        
        rospy.loginfo("Global Sensing Node started")
        
        r = rospy.Rate(1)
        
        rospy.on_shutdown(self.shutdown)
        
        self.globalSensTopic = rospy.get_param("~globalsensingdata", "/fmInformation/gloSenDat")
    
        self.pub = rospy.Publisher(self.globalSensTopic, String)
        
        str = "GlobalSensingData"
        
        while not rospy.is_shutdown():
            self.pub.publish(str)
            r.sleep()
    
    
    
    
    def shutdown(self):
        
        rospy.loginfo("Shutting Down")
        rospy.sleep(1)       
        
               
        
        
if __name__ == '__main__':
    try:
        GlobalSensing()
    except:
        rospy.loginfo(" Global Sensing Node Terminated")
    