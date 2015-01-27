#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros.

import rospy
from std_msgs.msg import String

class PlatformExecutors():
    def __init__(self):
        
        rospy.init_node('ish_platf_executors', anonymous=False)
        
        rospy.loginfo("Platform Executors Node Started")
        
        self.platfExecutorsTopic = rospy.get_param("~platformexecutorsdata", '/fmCommands/plaExeDat')
        self.behavTopic = rospy.get_param("~behaviourdata", '/fmPlans/behDat')
        
        self.pub = rospy.Publisher(self.platfExecutorsTopic,String)
        r = rospy.Rate(1)
        r.sleep()
        
        rospy.Subscriber(self.behavTopic,String,self.on_behavTopic)
        
        while not rospy.is_shutdown():
            str = "PlatformExecutorsData"
            self.pub.publish(str)
            r.sleep()
        
    def on_behavTopic(self,msg):
        rospy.sleep(20)
        
        

if __name__ == '__main__':
    try:
        PlatformExecutors()
    except:
        rospy.loginfo("Platform Executors Node Terminated")