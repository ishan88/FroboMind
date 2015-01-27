#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros.

import rospy
from std_msgs.msg import String

class IncidentHandler():
    def __init__(self):
        
        rospy.init_node('ish_incidenthandler', anonymous=False)
        
        rospy.loginfo("Incident Handler Node Started")
        
        self.incidentHandlerTopic = rospy.get_param("~incidenthandlerdata", '/fmSafety/incHanDat')
        self.objectDetTopic = rospy.get_param("~objectdetectiondata", '/fmKnowledge/objDetDat')
        self.watchdogTopic = rospy.get_param("~watchdogdata", '/fmSafety/watDogDat')
        self.faultDiagTopic = rospy.get_param("~faultdiagnosisdata", '/fmSafety/fauDiaDat')
        
        
        self.pub = rospy.Publisher(self.incidentHandlerTopic,String)
        r = rospy.Rate(1)
        r.sleep()
        
        
        rospy.Subscriber(self.objectDetTopic, String, self.on_objDetTopic)
        rospy.Subscriber(self.watchdogTopic, String, self.on_watchdogTopic)
        rospy.Subscriber(self.faultDiagTopic, String, self.on_faultDiagTopic)
        
        while not rospy.is_shutdown():
            str = "IncidentHandlerData"
            self.pub.publish(str)
            r.sleep()
        
    def on_objDetTopic(self, msg): 
        rospy.sleep(20) 
        
    def on_watchdogTopic(self, msg): 
        rospy.sleep(20)
        
    def on_faultDiagTopic(self, msg): 
        rospy.sleep(20)
          

if __name__ == '__main__':
    try:
        IncidentHandler()
        rospy.spin()
    except:
        rospy.loginfo("Incident Handler Node Terminated")