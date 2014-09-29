#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros.

import rospy
from std_msgs.msg import String

class ImplementControllers():
    def __init__(self):
        
        rospy.init_node('ish_imp_controllers', anonymous=False)
        
        rospy.loginfo("Implement Controllers Node Started")
        
        self.impExecutorsTopic = rospy.get_param("~implementexecutorsdata", '/fmCommands/impExeDat')
        self.impControllersTopic = rospy.get_param("~implementcontrollersdata", '/fmSignals/impConDat')
        self.incidentHandlerTopic = rospy.get_param("~incidenthandlerdata", '/fmSafety/incHanDat')
        
        self.pub = rospy.Publisher(self.impControllersTopic,String)
        r = rospy.Rate(1)
        r.sleep()
        
        rospy.Subscriber(self.impExecutorsTopic,String,self.on_impExecutorsTopic)
        rospy.Subscriber(self.incidentHandlerTopic,String, self.on_incidentHandlerTopic)
        
        while not rospy.is_shutdown():
            str = "ImplementControllersData"
            self.pub.publish(str)
            r.sleep()
        
    def on_impExecutorsTopic(self,msg):
        rospy.sleep(20)
        
    def on_incidentHandlerTopic(self,msg):
        rospy.sleep(20)    
        

if __name__ == '__main__':
    try:
        ImplementControllers()
    except:
        rospy.loginfo("Implement Controllers Node Terminated")