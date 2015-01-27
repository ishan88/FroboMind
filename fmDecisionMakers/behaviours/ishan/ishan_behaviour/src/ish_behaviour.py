#!/usr/bin/env python
# This file is part of test_demo launch file in ishan_ros.

import rospy
from std_msgs.msg import String

class Behaviour():
    def __init__(self):
        
        rospy.init_node('ish_behaviour', anonymous=False)
        
        rospy.loginfo("Behaviour Node Started")
        
        rospy.on_shutdown(self.shutdown)
        
        self.missionPlanTopic = rospy.get_param("~missionplannerdata", '/fmDecisions/misPlaDat')
        self.behaviourTopic = rospy.get_param("~behaviourdata", '/fmPlans/behDat')
        
        r = rospy.Rate(1)
        self.pub = rospy.Publisher(self.behaviourTopic,String)
        rospy.sleep(1)
        
        
        rospy.Subscriber(self.missionPlanTopic,String, self.on_missionPlanTopic)
        
        
        while not rospy.is_shutdown():
            str = "BehaviourData"
            self.pub.publish(str)
            r.sleep()
            
            
            
            
    def shutdown(self):
        
        rospy.loginfo("Shutting Down")
        rospy.sleep(1)
            
    def on_missionPlanTopic(self,msg):
        rospy.sleep(1)
        
        

if __name__ == '__main__':
    try:
        Behaviour()
    except:
        rospy.loginfo("Behaviour Node Terminated")