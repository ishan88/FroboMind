#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros.

import rospy
from std_msgs.msg import String

class ImplementExecutors():
    def __init__(self):
        
        rospy.init_node('ish_imp_executors', anonymous=False)
        
        rospy.loginfo("Implement Executors Node Started")
        
        self.impExecutorsTopic = rospy.get_param("~implementexecutorsdata", '/fmCommands/impExeDat')
        self.behavTopic = rospy.get_param("~behaviourdata", '/fmPlans/behDat')
        
        self.pub = rospy.Publisher(self.impExecutorsTopic,String)
        r = rospy.Rate(1)
        r.sleep()
        
        rospy.Subscriber(self.behavTopic,String,self.on_behavTopic)
        
        while not rospy.is_shutdown():
            str = "ImplementExecutorsData"
            self.pub.publish(str)
            r.sleep()
        
    def on_behavTopic(self,msg):
        rospy.sleep(20)
        
        

if __name__ == '__main__':
    try:
       ImplementExecutors()
    except:
        rospy.loginfo("Implement Executors Node Terminated")