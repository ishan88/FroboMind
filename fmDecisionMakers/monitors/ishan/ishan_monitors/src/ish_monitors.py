#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros.

import rospy
from std_msgs.msg import String

class Monitors():
    def __init__(self):
        
        rospy.init_node('ish_monitors', anonymous=False)
        
        
        rospy.loginfo(" Monitors Node Started ")    
                
        
        rospy.on_shutdown(self.shutdown)
    
    
        self.objDetTopic = rospy.get_param("~objectdetectiondata", '/fmKnowledge/objDetDat')
        self.localMapTopic = rospy.get_param("~localizationmappingdata", '/fmKnowledge/locMapDat')
        self.platfProcessTopic = rospy.get_param("~platformprocessingdata", '/fmKnowledge/plaProDat')
        self.impProcessTopic = rospy.get_param("~implementprocessingdata", '/fmKnowledge/impProDat')
        self.monitorTopic = rospy.get_param("~monitorsdata", '/fmDecisions/monDat')
        
        r = rospy.Rate(1)
        self.pub = rospy.Publisher(self.monitorTopic,String)        
        rospy.sleep(1)
       
        rospy.Subscriber(self.objDetTopic,String,self.on_objDetTopic)
        rospy.Subscriber(self.localMapTopic,String, self.on_localMapTopic)
        rospy.Subscriber(self.platfProcessTopic,String,self.on_platfProcessTopic)
        rospy.Subscriber(self.impProcessTopic,String,self.on_impProcessTopic)
        
        while not rospy.is_shutdown():
            str = "MonitorsData"
            self.pub.publish(str)
            r.sleep()
            
            
        
        
        
    def on_objDetTopic(self,msg):
        rospy.sleep(20)
    
    def on_localMapTopic(self,msg):
        rospy.sleep(20)
    
    def on_platfProcessTopic(self,msg):
        rospy.sleep(20)
    
    def on_impProcessTopic(self,msg):
        rospy.sleep(20)
    
    
    
    
    
    def shutdown(self):
        
        rospy.loginfo("Monitors Node Shutdown")
        rospy.sleep(1)
        

if __name__ == '__main__':
    try:
        Monitors()
        rospy.spin()
    except:
        rospy.loginfo("Monitors Node terminated")
        