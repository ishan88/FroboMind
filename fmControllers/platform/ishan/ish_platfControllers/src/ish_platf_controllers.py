#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros.

import rospy
from std_msgs.msg import String

class PlatformControllers():
    def __init__(self):
        
        rospy.init_node('ish_platf_controllers', anonymous=False)
        
        rospy.loginfo("Platform Controllers Node Started")
        
        self.platfExecutorsTopic = rospy.get_param("~platformexecutorsdata", '/fmCommands/plaExeDat')
        self.platfControllersTopic = rospy.get_param("~platformcontrollersdata", '/fmSignals/plaConDat')
        self.incidentHandlerTopic = rospy.get_param("~incidenthandlerdata", '/fmSafety/incHanDat')
        
        self.pub = rospy.Publisher(self.platfControllersTopic,String)
        r = rospy.Rate(1)
        r.sleep()
        
        rospy.Subscriber(self.platfExecutorsTopic,String,self.on_platfExecutorsTopic)
        rospy.Subscriber(self.incidentHandlerTopic,String, self.on_incidentHandlerTopic)
        
        while not rospy.is_shutdown():
            str = "PlatformControllersData"
            self.pub.publish(str)
            r.sleep()
        
    def on_platfExecutorsTopic(self,msg):
        rospy.sleep(20)
        
    def on_incidentHandlerTopic(self,msg):
        rospy.sleep(20)    

if __name__ == '__main__':
    try:
       PlatformControllers()
    except:
        rospy.loginfo("Platform Controllers Node Terminated")